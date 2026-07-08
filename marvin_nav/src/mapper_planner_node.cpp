// Mapping + Planung in einem Node:
//  - baut die OctoMap in-process aus Punktwolken (ersetzt octomap_server;
//    kein Serialisieren/Deserialisieren der Karte mehr pro Update)
//  - globaler A* auf grober OctoMap-Tiefe
//  - lokales ESDF-Fenster (Felzenszwalb-EDT) + lokaler A* + Glättung
//  - Kartenpersistenz: .bt laden beim Start (Parameter map_file),
//    speichern via Service ~/save_map

#include "uav_planner/nav_node.hpp"

#include <queue>
#include <unordered_map>
#include <cmath>
#include <algorithm>
#include <limits>

#include <tf2/exceptions.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl_conversions/pcl_conversions.h>

namespace {
double yawFromQuat(const geometry_msgs::msg::Quaternion& q) {
    return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}
}

NavNode::NavNode() : Node("nav_node") {
    const auto cloud_topic = declare_parameter("cloud_topic", std::string("/cloud_merged"));
    sensor_frame_ = declare_parameter("sensor_frame", std::string(""));
    map_file_ = declare_parameter("map_file", std::string(""));

    tree_ = std::make_unique<octomap::OcTree>(map_res_);
    if (!map_file_.empty()) {
        if (tree_->readBinary(map_file_)) {
            RCLCPP_INFO(get_logger(), "Karte geladen: %s (%zu Knoten)",
                map_file_.c_str(), tree_->getNumLeafNodes());
            octomap_updated_ = true;
        } else {
            RCLCPP_WARN(get_logger(), "Karte %s nicht lesbar — starte leer",
                map_file_.c_str());
        }
    }
    // Nach dem Laden setzen — readBinary überschreibt die Baum-Parameter
    tree_->setProbHit(prob_hit_);
    tree_->setProbMiss(prob_miss_);
    tree_->setClampingThresMin(clamp_min_);
    tree_->setClampingThresMax(clamp_max_);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        cloud_topic, rclcpp::SensorDataQoS().keep_last(2),
        std::bind(&NavNode::cloudCallback, this, std::placeholders::_1));

    octomap_pub_ = create_publisher<octomap_msgs::msg::Octomap>(
        "/octomap_binary", rclcpp::QoS(1).transient_local());
    octomap_points_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "/octomap_points", rclcpp::QoS(1).transient_local());

    save_map_srv_ = create_service<std_srvs::srv::Trigger>(
        "~/save_map",
        std::bind(&NavNode::saveMapCallback, this,
                  std::placeholders::_1, std::placeholders::_2));

    drone_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "/mavros/local_position/pose",
        rclcpp::QoS(10).best_effort(),
        std::bind(&NavNode::dronePoseCallback, this, std::placeholders::_1));

    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "/goal_pose",
        rclcpp::QoS(1).reliable(),
        std::bind(&NavNode::goalCallback, this, std::placeholders::_1));

    path_pub_ = create_publisher<nav_msgs::msg::Path>("/planned_path", 10);

    // ESDF publishers + lokaler Pfad
    esdf_slice_xy_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("/esdf_slice_xy", 10);
    esdf_slice_xz_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("/esdf_slice_xz", 10);
    local_path_pub_ = create_publisher<nav_msgs::msg::Path>("/local_planned_path", 10);
    esdf_timer_ = create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&NavNode::computeAndPublishEsdf, this));
}

// --- Mapping ---

void NavNode::cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    // ponytail: feste Insertionsrate ~2.5 Hz — Raycasting ist der teure Teil,
    // bei <1 m/s Fluggeschwindigkeit reicht das dicke
    // Node-Uhr statt msg->header.stamp: manche Sensor-Bridges stempeln nicht
    // sauber (bleibt 0 oder springt nicht) — Throttle würde dann nach dem
    // ersten Insert für immer verriegeln
    const rclcpp::Time stamp = now();
    if (last_insert_time_.nanoseconds() > 0 &&
        (stamp - last_insert_time_).seconds() < insert_min_period_)
        return;

    // Cloud nach map transformieren; Sensor-Ursprung fürs Raycasting separat,
    // denn vor-registrierte Clouds (z.B. /cloud_registered) haben als Frame den
    // Odom-Frame, nicht den Sensor
    geometry_msgs::msg::TransformStamped cloud_tf, origin_tf;
    const std::string origin_frame =
        sensor_frame_.empty() ? msg->header.frame_id : sensor_frame_;
    try {
        // Neuestes TF statt exaktem Stempel (wie updateDronePoseFromTf) — sonst
        // "extrapolation into the future" bei jeder TF-Publish-Lücke > Timeout
        cloud_tf = tf_buffer_->lookupTransform("map", msg->header.frame_id,
            tf2::TimePointZero);
        origin_tf = tf_buffer_->lookupTransform("map", origin_frame,
            tf2::TimePointZero);
    } catch (const tf2::TransformException& e) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
            "Cloud verworfen — kein TF nach map: %s", e.what());
        return;
    }
    last_insert_time_ = stamp;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud);
    // Gazebos Depth-Kamera setzt is_dense=true, obwohl No-Return-Pixel als +Inf
    // kodiert sind — bei is_dense=true überspringt PCL die NaN/Inf-Prüfung komplett
    cloud->is_dense = false;
    const size_t raw_size = cloud->size();
    std::vector<int> keep;
    pcl::removeNaNFromPointCloud(*cloud, *cloud, keep);
    if (cloud->empty()) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
            "Cloud verworfen — nur NaN/Inf-Punkte (%zu roh)", raw_size);
        return;
    }

    // Downsampling vor der Insertion — mehr als 1 Punkt pro halbem Voxel bringt nichts
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud(cloud);
    const float leaf = static_cast<float>(map_res_ / 2.0);
    vg.setLeafSize(leaf, leaf, leaf);
    pcl::PointCloud<pcl::PointXYZ> ds;
    vg.filter(ds);

    const Eigen::Isometry3d T = tf2::transformToEigen(cloud_tf);
    octomap::Pointcloud oc;
    oc.reserve(ds.size());
    for (const auto& p : ds.points) {
        const Eigen::Vector3d w = T * Eigen::Vector3d(p.x, p.y, p.z);
        if (w.z() < cloud_z_min_ || w.z() > cloud_z_max_) continue;
        oc.push_back(w.x(), w.y(), w.z());
    }
    if (oc.size() == 0) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
            "Cloud verworfen — alle %zu Punkte außerhalb z=[%.1f, %.1f]",
            ds.size(), cloud_z_min_, cloud_z_max_);
        return;
    }

    const octomap::point3d origin(
        origin_tf.transform.translation.x,
        origin_tf.transform.translation.y,
        origin_tf.transform.translation.z);
    tree_->insertPointCloud(oc, origin, max_range_, false, true);
    octomap_updated_ = true;

    if (++inserts_since_publish_ >= publish_every_n_inserts_) {
        inserts_since_publish_ = 0;
        publishOctomap();
    }
}

void NavNode::publishOctomap() {
    octomap_msgs::msg::Octomap msg;
    if (!octomap_msgs::binaryMapToMsg(*tree_, msg)) return;
    msg.header.frame_id = "map";
    msg.header.stamp = now();
    octomap_pub_->publish(msg);

    // Besetzte Voxel als PointCloud2 — RViz kann das ohne octomap_rviz_plugins
    pcl::PointCloud<pcl::PointXYZ> cells;
    for (auto it = tree_->begin_leafs(), end = tree_->end_leafs(); it != end; ++it) {
        if (!tree_->isNodeOccupied(*it)) continue;
        const auto c = it.getCoordinate();
        cells.emplace_back(c.x(), c.y(), c.z());
    }
    sensor_msgs::msg::PointCloud2 pc;
    pcl::toROSMsg(cells, pc);
    pc.header = msg.header;
    octomap_points_pub_->publish(pc);
}

void NavNode::saveMapCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
    (void)req;
    if (map_file_.empty()) {
        res->success = false;
        res->message = "Parameter 'map_file' nicht gesetzt";
        return;
    }
    if (!tree_ || tree_->size() == 0) {
        res->success = false;
        res->message = "Keine Karte vorhanden";
        return;
    }
    res->success = tree_->writeBinary(map_file_);
    res->message = (res->success ? "Gespeichert: " : "Schreiben fehlgeschlagen: ") + map_file_;
    RCLCPP_INFO(get_logger(), "%s", res->message.c_str());
}

// --- Pose / Ziel ---

void NavNode::dronePoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    drone_pose_ = *msg;
}

bool NavNode::updateDronePoseFromTf() {
    try {
        auto tf = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
        // Stale TF (z.B. FAST-LIO tot, letzter Wert eingefroren) nicht als
        // Pose verwenden — sonst plant der Planer von einer Geisterposition.
        // Pursuit hat dieselbe Prüfung, damit beide konsistent zurückfallen.
        if ((now() - rclcpp::Time(tf.header.stamp)).seconds() > tf_pose_max_age_)
            return false;
        drone_pose_.header.stamp = tf.header.stamp;
        drone_pose_.header.frame_id = "map";
        drone_pose_.pose.position.x = tf.transform.translation.x;
        drone_pose_.pose.position.y = tf.transform.translation.y;
        drone_pose_.pose.position.z = tf.transform.translation.z;
        drone_pose_.pose.orientation = tf.transform.rotation;
        return true;
    } catch (const tf2::TransformException&) {
        return false;  // Fallback: letzte Topic-Pose bleibt in drone_pose_
    }
}

bool NavNode::isGoalDifferent(const geometry_msgs::msg::PoseStamped& new_goal) const {
    double dx = new_goal.pose.position.x - goal_pose_.pose.position.x;
    double dy = new_goal.pose.position.y - goal_pose_.pose.position.y;
    double dz = new_goal.pose.position.z - goal_pose_.pose.position.z;
    if (std::sqrt(dx*dx + dy*dy + dz*dz) > goal_change_thresh_)
        return true;
    // Auch eine reine Yaw-Änderung zählt als neues Ziel
    double dyaw = yawFromQuat(new_goal.pose.orientation)
                - yawFromQuat(goal_pose_.pose.orientation);
    dyaw = std::atan2(std::sin(dyaw), std::cos(dyaw));
    return std::abs(dyaw) > goal_yaw_thresh_;
}

void NavNode::goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    if (!msg->header.frame_id.empty() && msg->header.frame_id != "map") {
        RCLCPP_WARN(get_logger(),
            "Zielpose in Frame '%s' statt 'map' — wird unveraendert uebernommen",
            msg->header.frame_id.c_str());
    }

    // Ungültige (Null-)Quaternion → Identität, sonst ist der Ziel-Yaw undefiniert
    auto& q = msg->pose.orientation;
    if (q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w < 1e-6)
        q.w = 1.0;

    RCLCPP_INFO(get_logger(), "Zielpose empfangen: (%.2f, %.2f, %.2f, Yaw %.1f°)",
        msg->pose.position.x, msg->pose.position.y, msg->pose.position.z,
        yawFromQuat(q) * 180.0 / M_PI);

    // Neues Ziel nur akzeptieren wenn es sich vom aktuellen unterscheidet
    if (state_ == State::NAVIGATING && !isGoalDifferent(*msg)) {
        RCLCPP_INFO(get_logger(), "Neues Ziel zu nah am aktuellen — ignoriert");
        return;
    }

    goal_pose_ = *msg;
    state_ = State::NAVIGATING;
    consecutive_failures_ = 0;
    RCLCPP_INFO(get_logger(), "State -> NAVIGATING");

    // Sofort planen, dann Timer starten für Replan alle 2 s
    AstarPlanner();

    if (!replan_timer_) {
        replan_timer_ = create_wall_timer(
            std::chrono::seconds(2),
            std::bind(&NavNode::replanTimerCallback, this));
    }
}

void NavNode::replanTimerCallback() {
    if (state_ != State::NAVIGATING) {
        return;
    }

    updateDronePoseFromTf();
    double dist = distanceToGoal();
    if (dist < goal_tolerance_) {
        RCLCPP_INFO(get_logger(), "Ziel erreicht (%.2f m) — State -> IDLE", dist);
        state_ = State::IDLE;
        if (replan_timer_) {
            replan_timer_->cancel();
            replan_timer_.reset();
        }
        return;
    }

    // Sticky: committete Route halten solange sie frei ist — verhindert das
    // Hin-und-Her zwischen gleichwertigen Umwegen (links/rechts um eine Saeule).
    // Frisch aufgetauchte Hindernisse triggern den Replan bereits ueber
    // validateGlobalPath(); hier nur bei blockierter/leerer Route neu planen.
    if (globalPathClear()) {
        RCLCPP_DEBUG(get_logger(), "Globaler Pfad frei — committed, kein Replan (%.2f m)", dist);
        return;
    }
    RCLCPP_INFO(get_logger(), "Globaler Pfad blockiert/leer — Replan (%.2f m)", dist);
    AstarPlanner();
}

// Prueft die verbleibende committete Route gegen die aktuelle OctoMap
// (Occupancy + Inflation wie der Planer). Frei → Route halten statt neu zu planen.
bool NavNode::globalPathClear() {
    if (!tree_ || last_global_path_.poses.empty()) return false;

    const unsigned int depth = tree_->getTreeDepth() - 1;
    const int stride = 1 << (tree_->getTreeDepth() - depth);
    constexpr int inflate_r = 1;

    // Nur den Rest ab dem naechstgelegenen Wegpunkt pruefen
    const auto& poses = last_global_path_.poses;
    size_t nearest = 0;
    double best = std::numeric_limits<double>::max();
    for (size_t i = 0; i < poses.size(); ++i) {
        const auto& p = poses[i].pose.position;
        double dx = p.x - drone_pose_.pose.position.x;
        double dy = p.y - drone_pose_.pose.position.y;
        double dz = p.z - drone_pose_.pose.position.z;
        double d = dx*dx + dy*dy + dz*dz;
        if (d < best) { best = d; nearest = i; }
    }

    for (size_t i = nearest; i < poses.size(); ++i) {
        const auto& p = poses[i].pose.position;
        octomap::OcTreeKey k;
        if (!tree_->coordToKeyChecked(p.x, p.y, p.z, depth, k)) continue;
        for (int dx = -inflate_r; dx <= inflate_r; ++dx)
            for (int dy = -inflate_r; dy <= inflate_r; ++dy)
                for (int dz = -inflate_r; dz <= inflate_r; ++dz) {
                    octomap::OcTreeKey nk(k[0] + dx * stride,
                                          k[1] + dy * stride,
                                          k[2] + dz * stride);
                    octomap::OcTreeNode* n = tree_->search(nk, depth);
                    if (n && tree_->isNodeOccupied(n)) return false;  // blockiert
                }
    }
    return true;
}

double NavNode::distanceToGoal() {
    if (drone_pose_.header.stamp.sec == 0 || goal_pose_.header.stamp.sec == 0) {
        return std::numeric_limits<double>::max();
    }

    double dx = goal_pose_.pose.position.x - drone_pose_.pose.position.x;
    double dy = goal_pose_.pose.position.y - drone_pose_.pose.position.y;
    double dz = goal_pose_.pose.position.z - drone_pose_.pose.position.z;
    return std::sqrt(dx*dx + dy*dy + dz*dz);
}

// --- Recovery ---

void NavNode::planFailed(const char* what) {
    if (state_ != State::NAVIGATING) return;
    ++consecutive_failures_;
    if (consecutive_failures_ < max_failures_) return;

    RCLCPP_ERROR(get_logger(),
        "Planung %d-mal in Folge fehlgeschlagen (zuletzt: %s) — Abbruch, Halte-Signal",
        consecutive_failures_, what);
    state_ = State::IDLE;
    consecutive_failures_ = 0;
    if (replan_timer_) {
        replan_timer_->cancel();
        replan_timer_.reset();
    }
    // Leerer Pfad = Stopp-Signal: Pursuit hält die aktuelle Position
    nav_msgs::msg::Path empty;
    empty.header.stamp = now();
    empty.header.frame_id = "map";
    local_path_pub_->publish(empty);
}

// --- Globaler Planer ---

void NavNode::AstarPlanner() {
    if (!tree_ || tree_->size() == 0) {
        RCLCPP_WARN(get_logger(), "Keine OctoMap vorhanden");
        planFailed("keine Karte");
        return;
    }
    updateDronePoseFromTf();
    if (drone_pose_.header.stamp.sec == 0) {
        RCLCPP_WARN(get_logger(), "Noch keine Drohnenpose empfangen — Planung uebersprungen");
        planFailed("keine Pose");
        return;
    }

    const unsigned int search_depth = tree_->getTreeDepth() - 1;  // Tiefe 15 → 0.6m Voxel
    const double step_size = tree_->getNodeSize(search_depth);
    // Keys liegen immer in max-Tiefen-Auflösung: ein Voxel auf search_depth
    // entspricht 'stride' Key-Einheiten. Nachbarn/Inflation müssen damit skalieren,
    // sonst bleiben Keys unausgerichtet und der Goal-Vergleich schlägt fehl.
    const int stride = 1 << (tree_->getTreeDepth() - search_depth);

    // Convert start/goal to OcTreeKeys at coarser depth
    octomap::OcTreeKey start_key, goal_key;
    if (!tree_->coordToKeyChecked(
            drone_pose_.pose.position.x,
            drone_pose_.pose.position.y,
            drone_pose_.pose.position.z, search_depth, start_key) ||
        !tree_->coordToKeyChecked(
            goal_pose_.pose.position.x,
            goal_pose_.pose.position.y,
            goal_pose_.pose.position.z, search_depth, goal_key)) {
        RCLCPP_ERROR(get_logger(), "Start oder Ziel liegt außerhalb der Karte");
        planFailed("Start/Ziel ausserhalb der Karte");
        return;
    }

    // Collision-Check (Inflation) + Unknown-Penalty (nur Zentral-Voxel)
    // Rückgabe: Zusatzkosten für Voxel, < 0 = blockiert
    constexpr int inflate_r = 1;  // in Voxeln dieser Tiefe (≈0.6 m Abstand)
    using KeyHash = octomap::OcTreeKey::KeyHash;
    std::unordered_map<octomap::OcTreeKey, double, KeyHash> cost_cache;

    auto voxel_cost = [&](const octomap::OcTreeKey& k) -> double {
        auto it = cost_cache.find(k);
        if (it != cost_cache.end()) return it->second;

        // Inflation: nur Occupied-Check
        for (int dx = -inflate_r; dx <= inflate_r; ++dx) {
            for (int dy = -inflate_r; dy <= inflate_r; ++dy) {
                for (int dz = -inflate_r; dz <= inflate_r; ++dz) {
                    octomap::OcTreeKey nk(k[0] + dx * stride,
                                          k[1] + dy * stride,
                                          k[2] + dz * stride);
                    octomap::OcTreeNode* node = tree_->search(nk, search_depth);
                    if (node && tree_->isNodeOccupied(node)) {
                        cost_cache[k] = -1.0;
                        return -1.0;  // Blockiert
                    }
                }
            }
        }

        // Unknown-Penalty: nur Zentral-Voxel prüfen
        double cost = 0.0;
        octomap::OcTreeNode* center = tree_->search(k, search_depth);
        if (!center) cost = unknown_penalty_weight_;

        cost_cache[k] = cost;
        return cost;
    };

    // Start/Ziel in blockiertem Voxel (Drohne oder Ziel nahe Wand) → sonst
    // scheitert jede Planung. Nächstes freies Voxel in expandierenden Schalen suchen.
    auto nearestFree = [&](octomap::OcTreeKey& k, const char* name) -> bool {
        if (voxel_cost(k) >= 0.0) return true;
        for (int r = 1; r <= 2; ++r) {
            for (int dx = -r; dx <= r; ++dx) {
                for (int dy = -r; dy <= r; ++dy) {
                    for (int dz = -r; dz <= r; ++dz) {
                        if (std::max({std::abs(dx), std::abs(dy), std::abs(dz)}) != r)
                            continue;  // nur Schalen-Rand
                        octomap::OcTreeKey nk(k[0] + dx * stride,
                                              k[1] + dy * stride,
                                              k[2] + dz * stride);
                        if (voxel_cost(nk) >= 0.0) {
                            RCLCPP_WARN(get_logger(),
                                "%s blockiert — auf freies Nachbarvoxel verschoben", name);
                            k = nk;
                            return true;
                        }
                    }
                }
            }
        }
        RCLCPP_ERROR(get_logger(), "%s blockiert — kein freies Voxel in der Naehe", name);
        return false;
    };

    if (!nearestFree(start_key, "Start") || !nearestFree(goal_key, "Ziel")) {
        planFailed("Start/Ziel blockiert");
        return;
    }

    // Weighted A* heuristic: w > 1.0 biases toward goal (faster, suboptimal by factor w)
    constexpr double w = 2.5;
    auto heuristic = [](const octomap::OcTreeKey& a, const octomap::OcTreeKey& b) -> double {
        double dx = static_cast<int>(a[0]) - static_cast<int>(b[0]);
        double dy = static_cast<int>(a[1]) - static_cast<int>(b[1]);
        double dz = static_cast<int>(a[2]) - static_cast<int>(b[2]);
        return w * std::sqrt(dx*dx + dy*dy + dz*dz);
    };

    struct AStarNode {
        octomap::OcTreeKey key;
        double f;
        bool operator>(const AStarNode& o) const { return f > o.f; }
    };

    std::priority_queue<AStarNode, std::vector<AStarNode>, std::greater<AStarNode>> open;
    std::unordered_map<octomap::OcTreeKey, double, KeyHash> g_score;
    std::unordered_map<octomap::OcTreeKey, octomap::OcTreeKey, KeyHash> came_from;

    g_score[start_key] = 0.0;
    open.push({start_key, heuristic(start_key, goal_key)});

    bool found = false;
    size_t expanded = 0;
    const size_t max_expansions = 500000;

    while (!open.empty() && expanded < max_expansions) {
        AStarNode current = open.top();
        open.pop();

        if (current.key == goal_key) {
            found = true;
            break;
        }

        double cur_g = g_score[current.key];

        // Skip if we already found a better path to this node
        if (current.f > cur_g + heuristic(current.key, goal_key) + 1e-6)
            continue;

        ++expanded;

        // 26-connected neighbors
        for (int dx = -1; dx <= 1; ++dx) {
            for (int dy = -1; dy <= 1; ++dy) {
                for (int dz = -1; dz <= 1; ++dz) {
                    if (dx == 0 && dy == 0 && dz == 0) continue;

                    octomap::OcTreeKey nbr_key(
                        current.key[0] + dx * stride,
                        current.key[1] + dy * stride,
                        current.key[2] + dz * stride);

                    double vc = voxel_cost(nbr_key);
                    if (vc < 0) continue;  // Blockiert

                    double move_dist = std::sqrt(double(dx*dx + dy*dy + dz*dz));
                    double tentative_g = cur_g + move_dist + vc;

                    auto it = g_score.find(nbr_key);
                    if (it != g_score.end() && tentative_g >= it->second)
                        continue;

                    g_score[nbr_key] = tentative_g;
                    came_from[nbr_key] = current.key;
                    open.push({nbr_key, tentative_g + heuristic(nbr_key, goal_key)});
                }
            }
        }
    }

    if (!found) {
        RCLCPP_ERROR(get_logger(), "Kein Pfad gefunden (%zu Knoten expandiert)", expanded);
        planFailed("global kein Pfad");
        return;
    }

    // Reconstruct path
    std::vector<octomap::OcTreeKey> key_path;
    octomap::OcTreeKey k = goal_key;
    while (!(k == start_key)) {
        key_path.push_back(k);
        k = came_from[k];
    }
    key_path.push_back(start_key);
    std::reverse(key_path.begin(), key_path.end());

    // Convert to nav_msgs::msg::Path
    nav_msgs::msg::Path path_msg;
    path_msg.header.stamp = now();
    path_msg.header.frame_id = "map";

    for (const auto& key : key_path) {
        octomap::point3d coord = tree_->keyToCoord(key, search_depth);
        geometry_msgs::msg::PoseStamped ps;
        ps.header = path_msg.header;
        ps.pose.position.x = coord.x();
        ps.pose.position.y = coord.y();
        ps.pose.position.z = coord.z();
        ps.pose.orientation.w = 1.0;
        path_msg.poses.push_back(ps);
    }

    // Ziel-Yaw an den letzten Wegpunkt hängen — Pursuit hält beim Erreichen
    // die Orientierung des letzten Pfadpunkts
    path_msg.poses.back().pose.orientation = goal_pose_.pose.orientation;

    last_global_path_ = path_msg;
    path_pub_->publish(path_msg);
    RCLCPP_INFO(get_logger(),
        "Pfad gefunden: %zu Wegpunkte, %zu Knoten expandiert, Länge: %.2f m",
        key_path.size(), expanded, (key_path.size() - 1) * step_size);
}

// --- ESDF ---

// Felzenszwalb & Huttenlocher 1D squared-distance transform (in-place)
void NavNode::edt1d(std::vector<double>& f, int n) {
    std::vector<int> v(n);
    std::vector<double> z(n + 1);
    constexpr double INF = 1e20;
    int k = 0;
    v[0] = 0;
    z[0] = -INF;
    z[1] = INF;

    for (int q = 1; q < n; ++q) {
        double s = ((f[q] + double(q) * q) - (f[v[k]] + double(v[k]) * v[k]))
                   / (2.0 * q - 2.0 * v[k]);
        while (s <= z[k]) {
            --k;
            s = ((f[q] + double(q) * q) - (f[v[k]] + double(v[k]) * v[k]))
                / (2.0 * q - 2.0 * v[k]);
        }
        ++k;
        v[k] = q;
        z[k] = s;
        z[k + 1] = INF;
    }

    k = 0;
    std::vector<double> result(n);
    for (int q = 0; q < n; ++q) {
        while (z[k + 1] < q) ++k;
        double d = q - v[k];
        result[q] = d * d + f[v[k]];
    }
    f = result;
}

// 3D EDT: apply 1D transform along each axis sequentially
void NavNode::edt3d(std::vector<double>& grid, int nx, int ny, int nz) {
    // Along X
    std::vector<double> strip(std::max({nx, ny, nz}));
    for (int z = 0; z < nz; ++z) {
        for (int y = 0; y < ny; ++y) {
            strip.resize(nx);
            for (int x = 0; x < nx; ++x)
                strip[x] = grid[(z * ny + y) * nx + x];
            edt1d(strip, nx);
            for (int x = 0; x < nx; ++x)
                grid[(z * ny + y) * nx + x] = strip[x];
        }
    }
    // Along Y
    for (int z = 0; z < nz; ++z) {
        for (int x = 0; x < nx; ++x) {
            strip.resize(ny);
            for (int y = 0; y < ny; ++y)
                strip[y] = grid[(z * ny + y) * nx + x];
            edt1d(strip, ny);
            for (int y = 0; y < ny; ++y)
                grid[(z * ny + y) * nx + x] = strip[y];
        }
    }
    // Along Z
    for (int y = 0; y < ny; ++y) {
        for (int x = 0; x < nx; ++x) {
            strip.resize(nz);
            for (int z = 0; z < nz; ++z)
                strip[z] = grid[(z * ny + y) * nx + x];
            edt1d(strip, nz);
            for (int z = 0; z < nz; ++z)
                grid[(z * ny + y) * nx + x] = strip[z];
        }
    }
}

void NavNode::computeAndPublishEsdf() {
    updateDronePoseFromTf();
    if (!tree_ || tree_->size() == 0 || drone_pose_.header.stamp.sec == 0) return;

    const double cx = drone_pose_.pose.position.x;
    const double cy = drone_pose_.pose.position.y;
    const double cz = drone_pose_.pose.position.z;

    // ESDF aktualisieren bei Distanz >= 2.5m ODER neuen Kartendaten
    {
        double dx = cx - esdf_center_x_;
        double dy = cy - esdf_center_y_;
        double dz = cz - esdf_center_z_;
        bool moved = std::sqrt(dx*dx + dy*dy + dz*dz) >= esdf_recompute_dist_;
        if (esdf_initialized_ && !moved && !octomap_updated_)
            return;
        octomap_updated_ = false;
    }

    esdf_nx_ = static_cast<int>(esdf_size_x_ / esdf_res_);
    esdf_ny_ = static_cast<int>(esdf_size_y_ / esdf_res_);
    esdf_nz_ = static_cast<int>(esdf_size_z_ / esdf_res_);
    const int nx = esdf_nx_, ny = esdf_ny_, nz = esdf_nz_;

    // Ursprung (min-Ecke) des lokalen Feldes
    esdf_ox_ = cx - esdf_size_x_ / 2.0;
    esdf_oy_ = cy - esdf_size_y_ / 2.0;
    esdf_oz_ = cz - esdf_size_z_ / 2.0;

    constexpr double INF = 1e20;
    // Distanzen NUR zu bekannt-besetzten Voxeln; Unbekannt läuft über die Maske
    esdf_grid_.assign(nx * ny * nz, INF);
    esdf_unknown_.assign(nx * ny * nz, 1);

    // Ein Baumdurchlauf über das Fenster statt einer search()-Anfrage pro Zelle
    const octomap::point3d pmin(esdf_ox_, esdf_oy_, esdf_oz_);
    const octomap::point3d pmax(esdf_ox_ + esdf_size_x_,
                                esdf_oy_ + esdf_size_y_,
                                esdf_oz_ + esdf_size_z_);
    // Zellen, deren Zentrum in [lo, hi] liegt
    auto cellRange = [&](double lo, double hi, double o, int n, int& i0, int& i1) {
        i0 = std::max(0, static_cast<int>(std::ceil((lo - o) / esdf_res_ - 0.5)));
        i1 = std::min(n - 1, static_cast<int>(std::floor((hi - o) / esdf_res_ - 0.5)));
    };
    for (auto it = tree_->begin_leafs_bbx(pmin, pmax), end = tree_->end_leafs_bbx();
         it != end; ++it) {
        const double half = it.getSize() / 2.0;
        const octomap::point3d c = it.getCoordinate();
        int x0, x1, y0, y1, z0, z1;
        cellRange(c.x() - half, c.x() + half, esdf_ox_, nx, x0, x1);
        cellRange(c.y() - half, c.y() + half, esdf_oy_, ny, y0, y1);
        cellRange(c.z() - half, c.z() + half, esdf_oz_, nz, z0, z1);
        const bool occ = tree_->isNodeOccupied(*it);
        for (int z = z0; z <= z1; ++z) {
            for (int y = y0; y <= y1; ++y) {
                for (int x = x0; x <= x1; ++x) {
                    const int idx = (z * ny + y) * nx + x;
                    esdf_unknown_[idx] = 0;
                    if (occ) esdf_grid_[idx] = 0.0;
                }
            }
        }
    }

    // 3D EDT berechnen (Ergebnis: quadrierte Distanzen in Voxel-Einheiten)
    edt3d(esdf_grid_, nx, ny, nz);

    // Zentrum merken
    esdf_center_x_ = cx;
    esdf_center_y_ = cy;
    esdf_center_z_ = cz;
    esdf_initialized_ = true;

    publishEsdfSlices();

    RCLCPP_DEBUG(get_logger(), "ESDF aktualisiert (%dx%dx%d, Zentrum: %.1f/%.1f/%.1f)",
        nx, ny, nz, cx, cy, cz);

    if (state_ == State::NAVIGATING && !last_global_path_.poses.empty()) {
        // Neu aufgetauchte Hindernisse auf dem globalen Pfad → sofort global replanen
        validateGlobalPath();
        localPlanner();
    }
}

void NavNode::publishEsdfSlices() {
    const int nx = esdf_nx_, ny = esdf_ny_, nz = esdf_nz_;

    // Squared voxel distances → metrische Distanzen
    auto distMetric = [&](int idx) -> double {
        return std::sqrt(esdf_grid_[idx]) * esdf_res_;
    };

    // Distanz → Costmap-Wert [0..100]
    auto distToCost = [&](double dist) -> int8_t {
        if (dist <= 0.0) return 100;
        if (dist >= esdf_max_dist_) return 0;
        return static_cast<int8_t>(100.0 * (1.0 - dist / esdf_max_dist_));
    };

    auto stamp = now();
    const double cy = esdf_center_y_;
    const double cz = esdf_center_z_;

    // --- XY-Slice (horizontal, auf Drohnenhöhe) ---
    {
        int iz = static_cast<int>((cz - esdf_oz_) / esdf_res_);
        iz = std::clamp(iz, 0, nz - 1);

        nav_msgs::msg::OccupancyGrid og;
        og.header.stamp = stamp;
        og.header.frame_id = "map";
        og.info.resolution = esdf_res_;
        og.info.width = nx;
        og.info.height = ny;
        og.info.origin.position.x = esdf_ox_;
        og.info.origin.position.y = esdf_oy_;
        og.info.origin.position.z = cz;
        og.info.origin.orientation.w = 1.0;
        og.data.resize(nx * ny);

        for (int y = 0; y < ny; ++y)
            for (int x = 0; x < nx; ++x)
                og.data[y * nx + x] = distToCost(distMetric((iz * ny + y) * nx + x));

        esdf_slice_xy_pub_->publish(og);
    }

    // --- XZ-Slice (vertikal, durch Drohnen-Y) ---
    {
        int iy = static_cast<int>((cy - esdf_oy_) / esdf_res_);
        iy = std::clamp(iy, 0, ny - 1);

        nav_msgs::msg::OccupancyGrid og;
        og.header.stamp = stamp;
        og.header.frame_id = "map";
        og.info.resolution = esdf_res_;
        og.info.width = nx;
        og.info.height = nz;
        og.info.origin.position.x = esdf_ox_;
        og.info.origin.position.y = cy;
        og.info.origin.position.z = esdf_oz_;
        og.info.origin.orientation.w = 1.0;
        og.data.resize(nx * nz);

        for (int z = 0; z < nz; ++z)
            for (int x = 0; x < nx; ++x)
                og.data[z * nx + x] = distToCost(distMetric((z * ny + iy) * nx + x));

        esdf_slice_xz_pub_->publish(og);
    }
}

// Prüft den verbleibenden globalen Pfad gegen die aktuelle Karte (via ESDF).
// Zwischen den 2-s-Replans neu aufgetauchte Hindernisse → sofort neu planen.
void NavNode::validateGlobalPath() {
    const auto& poses = last_global_path_.poses;

    // Nächstgelegenen Wegpunkt zur Drohne suchen — nur der Rest des Pfads zählt
    size_t nearest = 0;
    double best = std::numeric_limits<double>::max();
    for (size_t i = 0; i < poses.size(); ++i) {
        const auto& p = poses[i].pose.position;
        double dx = p.x - drone_pose_.pose.position.x;
        double dy = p.y - drone_pose_.pose.position.y;
        double dz = p.z - drone_pose_.pose.position.z;
        double d = dx*dx + dy*dy + dz*dz;
        if (d < best) { best = d; nearest = i; }
    }

    for (size_t i = nearest; i < poses.size(); ++i) {
        const auto& p = poses[i].pose.position;
        const int ix = static_cast<int>((p.x - esdf_ox_) / esdf_res_);
        const int iy = static_cast<int>((p.y - esdf_oy_) / esdf_res_);
        const int iz = static_cast<int>((p.z - esdf_oz_) / esdf_res_);
        if (ix < 0 || ix >= esdf_nx_ || iy < 0 || iy >= esdf_ny_ ||
            iz < 0 || iz >= esdf_nz_)
            continue;  // außerhalb des Fensters nicht prüfbar
        const int idx = (iz * esdf_ny_ + iy) * esdf_nx_ + ix;
        if (esdf_unknown_[idx]) continue;  // Unbekannt blockiert nicht
        if (std::sqrt(esdf_grid_[idx]) * esdf_res_ < hard_collision_dist_) {
            RCLCPP_WARN(get_logger(),
                "Globaler Pfad durch neues Hindernis blockiert (Wegpunkt %zu) — Replan", i);
            AstarPlanner();
            return;
        }
    }
}

// --- Lokaler Planer ---

double NavNode::sampleEsdf(double wx, double wy, double wz) const {
    const int ix = static_cast<int>((wx - esdf_ox_) / esdf_res_);
    const int iy = static_cast<int>((wy - esdf_oy_) / esdf_res_);
    const int iz = static_cast<int>((wz - esdf_oz_) / esdf_res_);
    if (ix < 0 || ix >= esdf_nx_ || iy < 0 || iy >= esdf_ny_ ||
        iz < 0 || iz >= esdf_nz_)
        return 0.0;  // Außerhalb = unsicher
    return std::sqrt(esdf_grid_[(iz * esdf_ny_ + iy) * esdf_nx_ + ix]) * esdf_res_;
}

void NavNode::localPlanner() {
    const int nx = esdf_nx_, ny = esdf_ny_, nz = esdf_nz_;
    const double ox = esdf_ox_, oy = esdf_oy_, oz = esdf_oz_;

    // Lokales Ziel: letzter Punkt des globalen Pfads innerhalb des Fensters
    const double max_x = ox + esdf_size_x_;
    const double max_y = oy + esdf_size_y_;
    const double max_z = oz + esdf_size_z_;

    int local_goal_idx = -1;
    for (int i = static_cast<int>(last_global_path_.poses.size()) - 1; i >= 0; --i) {
        auto& p = last_global_path_.poses[i].pose.position;
        if (p.x >= ox && p.x < max_x &&
            p.y >= oy && p.y < max_y &&
            p.z >= oz && p.z < max_z) {
            // Nur akzeptieren wenn der Punkt genug Abstand zu bekannten Hindernissen hat
            double dist = sampleEsdf(p.x, p.y, p.z);
            if (dist > hard_collision_dist_) {
                local_goal_idx = i;
                break;
            }
        }
    }

    if (local_goal_idx < 0) {
        // Committete Route liegt nicht mehr befahrbar im lokalen Fenster — meist
        // weil die Drohne abgedriftet ist oder der Wald sich zugesetzt hat. Frische
        // globale Route von der aktuellen Position holen (stellt das alte "immer
        // neu planen" fuer genau diesen Fall wieder her, ohne den Sticky-Replan
        // sonst aufzugeben). Hilft der Replan, resettet der naechste Zyklus den
        // Zaehler; bleibt es haengen, greift nach max_failures_ der Abbruch.
        RCLCPP_WARN(get_logger(),
            "Kein Wegpunkt im lokalen Fenster — globaler Replan von aktueller Position");
        AstarPlanner();
        planFailed("lokal kein Wegpunkt");
        return;
    }

    auto& gp = last_global_path_.poses[local_goal_idx].pose.position;

    // Start = Drohnenposition, Ziel = lokaler Austritt
    auto worldToIdx = [&](double wx, double wy, double wz) -> std::array<int, 3> {
        return {static_cast<int>((wx - ox) / esdf_res_),
                static_cast<int>((wy - oy) / esdf_res_),
                static_cast<int>((wz - oz) / esdf_res_)};
    };
    auto idxToWorld = [&](int ix, int iy, int iz) -> std::array<double, 3> {
        return {ox + (ix + 0.5) * esdf_res_,
                oy + (iy + 0.5) * esdf_res_,
                oz + (iz + 0.5) * esdf_res_};
    };
    auto inBounds = [&](int ix, int iy, int iz) -> bool {
        return ix >= 0 && ix < nx && iy >= 0 && iy < ny && iz >= 0 && iz < nz;
    };
    auto esdfDist = [&](int ix, int iy, int iz) -> double {
        return std::sqrt(esdf_grid_[(iz * ny + iy) * nx + ix]) * esdf_res_;
    };

    auto [sx, sy, sz] = worldToIdx(drone_pose_.pose.position.x,
                                    drone_pose_.pose.position.y,
                                    drone_pose_.pose.position.z);
    auto [gx, gy, gz] = worldToIdx(gp.x, gp.y, gp.z);

    sx = std::clamp(sx, 0, nx - 1);
    sy = std::clamp(sy, 0, ny - 1);
    sz = std::clamp(sz, 0, nz - 1);
    gx = std::clamp(gx, 0, nx - 1);
    gy = std::clamp(gy, 0, ny - 1);
    gz = std::clamp(gz, 0, nz - 1);

    // Steht die Drohne bereits unter dem Mindestabstand (Drift, frische Karte),
    // nicht komplett blockieren — Flucht erlauben, solange es nicht näher ans
    // Hindernis geht als die aktuelle Position.
    const double block_dist = std::min(hard_collision_dist_, esdfDist(sx, sy, sz));

    // 26-connected A* mit ESDF-Kostenfunktion
    struct Cell {
        int x, y, z;
        bool operator==(const Cell& o) const { return x == o.x && y == o.y && z == o.z; }
    };
    struct CellHash {
        size_t operator()(const Cell& c) const {
            return std::hash<int>()(c.x) ^ (std::hash<int>()(c.y) << 10) ^
                   (std::hash<int>()(c.z) << 20);
        }
    };
    struct OpenNode {
        Cell cell;
        double f;
        bool operator>(const OpenNode& o) const { return f > o.f; }
    };

    Cell start{sx, sy, sz}, goal{gx, gy, gz};

    std::priority_queue<OpenNode, std::vector<OpenNode>, std::greater<OpenNode>> open;
    std::unordered_map<Cell, double, CellHash> g_score;
    std::unordered_map<Cell, Cell, CellHash> came_from;

    auto heuristic = [&](const Cell& a, const Cell& b) -> double {
        double dx = a.x - b.x, dy = a.y - b.y, dz = a.z - b.z;
        return std::sqrt(dx*dx + dy*dy + dz*dz);
    };

    // Traversal-Kosten: Bewegung + ESDF-Penalty + Unknown-Penalty
    auto traversalCost = [&](const Cell& c, double move_dist) -> double {
        const int idx = (c.z * ny + c.y) * nx + c.x;
        double d = std::sqrt(esdf_grid_[idx]) * esdf_res_;
        if (d < block_dist) return 1e10;  // Quasi-blockiert
        double penalty = 0.0;
        if (d < safety_dist_) {
            double ratio = (safety_dist_ - d) / safety_dist_;
            penalty = obstacle_penalty_weight_ * ratio * ratio;
        }
        // Gleiche Unknown-Politik wie der globale Planer: erlaubt, aber bestraft
        if (esdf_unknown_[idx]) penalty += unknown_penalty_weight_;
        return move_dist + penalty;
    };

    g_score[start] = 0.0;
    open.push({start, heuristic(start, goal)});

    bool found = false;
    size_t expanded = 0;
    const size_t max_exp = 100000;

    while (!open.empty() && expanded < max_exp) {
        auto current = open.top();
        open.pop();

        if (current.cell == goal) { found = true; break; }

        double cur_g = g_score[current.cell];
        if (current.f > cur_g + heuristic(current.cell, goal) + 1e-6) continue;
        ++expanded;

        // 26-connected
        for (int dx = -1; dx <= 1; ++dx) {
            for (int dy = -1; dy <= 1; ++dy) {
                for (int dz = -1; dz <= 1; ++dz) {
                    if (dx == 0 && dy == 0 && dz == 0) continue;
                    Cell nbr{current.cell.x + dx, current.cell.y + dy, current.cell.z + dz};
                    if (!inBounds(nbr.x, nbr.y, nbr.z)) continue;

                    double move_dist = std::sqrt(double(dx*dx + dy*dy + dz*dz));
                    double cost = traversalCost(nbr, move_dist);
                    if (cost > 1e9) continue;

                    double tent_g = cur_g + cost;
                    auto it = g_score.find(nbr);
                    if (it != g_score.end() && tent_g >= it->second) continue;

                    g_score[nbr] = tent_g;
                    came_from[nbr] = current.cell;
                    open.push({nbr, tent_g + heuristic(nbr, goal)});
                }
            }
        }
    }

    if (!found) {
        RCLCPP_WARN(get_logger(), "Lokaler Pfad nicht gefunden (%zu expandiert)", expanded);
        planFailed("lokal kein Pfad");
        return;
    }

    // Pfad rekonstruieren (Weltkoordinaten)
    std::vector<std::array<double, 3>> path;
    Cell c = goal;
    while (!(c == start)) {
        auto w = idxToWorld(c.x, c.y, c.z);
        path.push_back(w);
        c = came_from[c];
    }
    path.push_back(idxToWorld(start.x, start.y, start.z));
    std::reverse(path.begin(), path.end());

    // Pfad glätten
    smoothPath(path);

    // Publizieren
    nav_msgs::msg::Path path_msg;
    path_msg.header.stamp = now();
    path_msg.header.frame_id = "map";
    for (const auto& pt : path) {
        geometry_msgs::msg::PoseStamped ps;
        ps.header = path_msg.header;
        ps.pose.position.x = pt[0];
        ps.pose.position.y = pt[1];
        ps.pose.position.z = pt[2];
        ps.pose.orientation.w = 1.0;
        path_msg.poses.push_back(ps);
    }

    // Endet der lokale Pfad am globalen Ziel → Ziel-Yaw übernehmen
    if (local_goal_idx == static_cast<int>(last_global_path_.poses.size()) - 1)
        path_msg.poses.back().pose.orientation =
            last_global_path_.poses.back().pose.orientation;

    local_path_pub_->publish(path_msg);
    consecutive_failures_ = 0;  // voller Planungszyklus erfolgreich

    RCLCPP_INFO(get_logger(), "Lokaler Pfad: %zu Wegpunkte (%zu expandiert)", path.size(), expanded);
}

void NavNode::smoothPath(std::vector<std::array<double, 3>>& path) {
    if (path.size() < 3) return;

    const int n = static_cast<int>(path.size());
    const double grad_step = esdf_res_ * 0.5;  // Finite-Differenzen-Schritt

    for (int iter = 0; iter < smooth_iterations_; ++iter) {
        for (int i = 1; i < n - 1; ++i) {
            auto& p = path[i];

            // Glättungsterm: Zug Richtung Mittelpunkt der Nachbarn
            std::array<double, 3> smooth_delta;
            for (int d = 0; d < 3; ++d)
                smooth_delta[d] = (path[i-1][d] + path[i+1][d]) / 2.0 - p[d];

            // ESDF-Gradient (numerisch, zentrale Differenzen)
            double dist_here = sampleEsdf(p[0], p[1], p[2]);
            std::array<double, 3> obs_delta = {0, 0, 0};
            if (dist_here < safety_dist_) {
                for (int d = 0; d < 3; ++d) {
                    std::array<double, 3> p_plus = p, p_minus = p;
                    p_plus[d] += grad_step;
                    p_minus[d] -= grad_step;
                    double d_plus = sampleEsdf(p_plus[0], p_plus[1], p_plus[2]);
                    double d_minus = sampleEsdf(p_minus[0], p_minus[1], p_minus[2]);
                    double grad = (d_plus - d_minus) / (2.0 * grad_step);
                    // Abstoßung: in Richtung steigender Distanz
                    double strength = (safety_dist_ - dist_here) / safety_dist_;
                    obs_delta[d] = grad * strength * obstacle_weight_ * esdf_res_;
                }
            }

            // Anwenden
            for (int d = 0; d < 3; ++d)
                p[d] += smooth_weight_ * smooth_delta[d] + obs_delta[d];

            // Sicherheitscheck: nicht in Hindernis schieben
            double new_dist = sampleEsdf(p[0], p[1], p[2]);
            if (new_dist < hard_collision_dist_) {
                // Rollback
                for (int d = 0; d < 3; ++d)
                    p[d] -= smooth_weight_ * smooth_delta[d] + obs_delta[d];
            }
        }
    }
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NavNode>());
    rclcpp::shutdown();
    return 0;
}
