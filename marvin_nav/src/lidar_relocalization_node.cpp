// Lidar-Relokalisierung / Online-Drift-Korrektur (REP-105):
// matcht den aktuellen registrierten Scan (im Odom-Frame, z.B. FAST-LIOs
// /cloud_registered in camera_init) per GICP gegen eine Referenzkarte und
// publiziert die Korrektur als TF map -> odom_frame.
//
// Referenzkarte wahlweise:
//  - map_pcd gesetzt: gespeicherte Karte (Relokalisierung, Karte eingefroren)
//  - sonst (Echtzeit-Modus): Karte wird im Flug aus eigenen Keyframes
//    aufgebaut. Ein Keyframe wird nur beim ERSTbesuch einer Gegend eingefuegt
//    und danach nie mehr veraendert — beim Wiederbesuch zieht GICP die
//    Live-Pose zurueck auf die Erstbesuchs-Geometrie. Das deckelt den Drift
//    in Echtzeit (impliziter Loop-Closure-Effekt fuer die Pose) und haelt sie
//    konsistent zu der Geometrie, aus der auch die Planner-OctoMap entstand.
// ponytail: kein Pose-Graph — bereits kartierte Bereiche werden bei grossem
// Zwischen-Drift nicht rueckwirkend entzerrt. Upgrade-Pfad: GTSAM-Backend
// mit gleicher TF-Schnittstelle (map -> odom_frame).

#include <chrono>
#include <cmath>
#include <limits>
#include <unordered_set>
#include <pcl/common/common.h>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_msgs/msg/float32.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_eigen/tf2_eigen.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl/registration/gicp.h>
#include <pcl_conversions/pcl_conversions.h>

// GICP liefert float-Matrizen, deren Rotationsteil nicht exakt orthonormal ist ->
// sauber in Translation + normierte Quaternion zerlegen
inline Eigen::Isometry3d rigid(const Eigen::Matrix4f& m) {
    const Eigen::Matrix4d d = m.cast<double>();
    return Eigen::Translation3d(Eigen::Vector3d(d.block<3, 1>(0, 3)))
         * Eigen::Quaterniond(Eigen::Matrix3d(d.block<3, 3>(0, 0))).normalized();
}

class LidarRelocalizationNode : public rclcpp::Node {
public:
    LidarRelocalizationNode() : Node("lidar_relocalization") {
        map_frame_ = declare_parameter("map_frame", std::string("map"));
        odom_frame_ = declare_parameter("odom_frame", std::string("camera_init"));
        const auto map_pcd = declare_parameter("map_pcd", std::string(""));
        const auto scan_topic = declare_parameter("scan_topic", std::string("/cloud_registered"));
        const auto odom_topic = declare_parameter("odom_topic", std::string("/Odometry"));
        voxel_size_ = declare_parameter("voxel_size", 0.4);
        fitness_max_ = declare_parameter("fitness_max", 0.5);  // mittl. quadr. Fehler [m²]
        // Strenger fuer die Hypothesensuche: dort konkurrieren falsche Lagen. Sim: richtig
        // ~0.03, falsche Minima 0.14-0.26. Auf echter Hardware nachmessen (Log "Hypothesensuche").
        search_fitness_max_ = declare_parameter("search_fitness_max", 0.1);
        // Anteil der Scanpunkte mit Kartenpunkt in < inlier_dist nach dem Match. Robuster als
        // die Fitness (die zaehlt nur Paare < 1 m und bleibt bei halb danebenliegendem Scan klein).
        // Sim richtig ~0.9+, falsch ~0.1-0.5. Auf Hardware nachmessen (/relocalization/inlier_ratio).
        inlier_dist_ = declare_parameter("inlier_dist", 0.5);
        min_inlier_ = declare_parameter("min_inlier", 0.6);
        jump_reject_ = declare_parameter("jump_reject", 1.0);   // Korrektursprung > x m verwerfen [m]
        // 0.1 rad pro Match: echte FAST-LIO-Drift ist um Groessenordnungen kleiner;
        // 0.3 liess GICP beim Drehen auf der Stelle in kleinen Schritten auf -17° kriechen
        rot_jump_reject_ = declare_parameter("rot_jump_reject", 0.1); // dito Rotation [rad]
        smooth_alpha_ = declare_parameter("smooth_alpha", 0.1); // Glaettung pro Broadcast-Tick [0..1]
        keyframe_dist_ = declare_parameter("keyframe_dist", 2.0); // Keyframe-Abstand / Erstbesuchs-Radius [m]
        // Auch neue Blickrichtung = Erstbesuch: sonst kennt die Karte beim Drehen
        // auf der Stelle nur den Startscan (gekipptes Lidar sieht je Yaw andere Hallenteile)
        keyframe_yaw_ = declare_parameter("keyframe_yaw", 0.8); // [rad] ~45°
        match_period_ = declare_parameter("match_period", 2.0);
        // Ziel fuer ~/save_map: Referenzkarte liegt schon im korrigierten map-
        // Frame — anders als FAST-LIOs PCD (camera_init, mit Drift), passt also
        // zur OctoMap des Planers.
        map_save_path_ = declare_parameter("map_save_path", std::string(""));

        // Startschätzung (z.B. bekannter Startplatz auf der Karte)
        const double ix = declare_parameter("initial_x", 0.0);
        const double iy = declare_parameter("initial_y", 0.0);
        const double iz = declare_parameter("initial_z", 0.0);
        const double iyaw = declare_parameter("initial_yaw", 0.0);
        // Montage-Pitch der FAST-LIO-IMU: camera_init erbt beim Init die IMU-
        // Lage, das Ry(pitch) hier levelt map (und damit Karte + Planer).
        const double ipitch = declare_parameter("initial_pitch", 0.0);
        initial_pitch_ = ipitch;
        estimate_ = Eigen::Translation3d(ix, iy, iz)
                  * Eigen::AngleAxisd(iyaw, Eigen::Vector3d::UnitZ())
                  * Eigen::AngleAxisd(ipitch, Eigen::Vector3d::UnitY());
        broadcast_ = estimate_;  // eingeschwungen starten (kein Sprung vom Ursprung)

        map_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
        online_ = map_pcd.empty();
        if (!online_) {
            if (pcl::io::loadPCDFile(map_pcd, *map_cloud_) == 0 && !map_cloud_->empty()) {
                downsample(map_cloud_);
                gicp_.setInputTarget(map_cloud_);
                have_map_ = true;
                RCLCPP_INFO(get_logger(), "Karte geladen: %s (%zu Punkte nach Downsampling)",
                    map_pcd.c_str(), map_cloud_->size());
            } else {
                RCLCPP_ERROR(get_logger(),
                    "Karte %s nicht lesbar — publiziere nur Identitaet", map_pcd.c_str());
            }
        } else {
            RCLCPP_INFO(get_logger(),
                "Echtzeit-Modus: Referenzkarte wird aus Keyframes aufgebaut (%s -> %s)",
                map_frame_.c_str(), odom_frame_.c_str());
        }

        gicp_.setMaxCorrespondenceDistance(declare_parameter("max_corr_dist", 2.0));  // [m]
        gicp_.setMaximumIterations(50);

        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        inlier_pub_ = create_publisher<std_msgs::msg::Float32>("/relocalization/inlier_ratio", 10);
        pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
            "/relocalization/pose", 10);
        scan_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            scan_topic, rclcpp::SensorDataQoS(),
            [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) {
                std::lock_guard<std::mutex> lk(mtx_);
                last_scan_ = msg;
            });
        // Neue Schaetzung (RViz "2D Pose Estimate" / Web-UI): x, y, Yaw der Drohne
        // JETZT im map-Frame. Die map->odom-Schaetzung wird um die Drohne herum so
        // verschoben/gedreht, dass sie dort steht (Hoehe, Roll/Pitch bleiben). Danach
        // Hypothesensuche (initialized_ = false), Broadcast springt sofort.
        initialpose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/initialpose", 10,
            [this](geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
                const auto& p = msg->pose.pose.position;
                const auto& q = msg->pose.pose.orientation;
                const double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
                {
                    std::lock_guard<std::mutex> lk(mtx_);
                    if (have_odom_) {
                        const Eigen::Isometry3d body = estimate_ * odomBody();
                        const Eigen::Vector3d b = body.translation();
                        const Eigen::Vector3d fwd = body.rotation() * Eigen::Vector3d::UnitX();
                        estimate_ = Eigen::Translation3d(p.x, p.y, b.z())
                            * Eigen::AngleAxisd(yaw - std::atan2(fwd.y(), fwd.x()), Eigen::Vector3d::UnitZ())
                            * Eigen::Translation3d(-b) * estimate_;
                    } else {  // FAST-LIO laeuft noch nicht: Drohne steht im odom-Ursprung
                        estimate_ = Eigen::Translation3d(p.x, p.y, p.z)
                            * Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())
                            * Eigen::AngleAxisd(initial_pitch_, Eigen::Vector3d::UnitY());
                    }
                    initialized_ = false;
                    reset_broadcast_ = true;
                }
                RCLCPP_INFO(get_logger(), "Neue Startschaetzung: (%.2f, %.2f, %.2f) Yaw %.0f°",
                    p.x, p.y, p.z, yaw * 180.0 / M_PI);
            });
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            odom_topic, rclcpp::SensorDataQoS(),
            [this](nav_msgs::msg::Odometry::SharedPtr msg) {
                std::lock_guard<std::mutex> lk(mtx_);
                odom_pos_ = Eigen::Vector3d(msg->pose.pose.position.x,
                                            msg->pose.pose.position.y,
                                            msg->pose.pose.position.z);
                const auto& q = msg->pose.pose.orientation;
                odom_rot_ = Eigen::Quaterniond(q.w, q.x, q.y, q.z);
                have_odom_ = true;
            });

        // GICP darf den 10-Hz-TF-Broadcast nicht blockieren (stale TF ->
        // pursuit/planner fallen auf MAVROS-Pose zurueck). Match-Timer laeuft
        // in eigener Callback-Group, main() spinnt multithreaded.
        match_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        match_timer_ = create_wall_timer(
            std::chrono::duration<double>(match_period_),
            std::bind(&LidarRelocalizationNode::match, this), match_group_);
        // Gleiche MutuallyExclusive-Group wie match(): map_cloud_ wird nie
        // parallel geschrieben, kein Lock noetig.
        save_srv_ = create_service<std_srvs::srv::Trigger>("~/save_map",
            [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                   std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
                res->message = map_save_path_;
                res->success = !map_save_path_.empty() && !map_cloud_->empty() &&
                    pcl::io::savePCDFileBinary(map_save_path_, *map_cloud_) == 0;
                if (!res->success) res->message = "Speichern fehlgeschlagen (map_save_path='" +
                    map_save_path_ + "', " + std::to_string(map_cloud_->size()) + " Punkte)";
            },
            rmw_qos_profile_services_default, match_group_);
        // TF kontinuierlich mit aktuellem Stempel, damit Listener nicht in Timeouts laufen
        tf_timer_ = create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&LidarRelocalizationNode::broadcast, this));
    }

private:
    void downsample(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
        pcl::VoxelGrid<pcl::PointXYZ> vg;
        vg.setInputCloud(cloud);
        const float l = static_cast<float>(voxel_size_);
        vg.setLeafSize(l, l, l);
        auto out = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
        vg.filter(*out);
        cloud = out;
    }

    void broadcast() {
        Eigen::Isometry3d target;
        {
            std::lock_guard<std::mutex> lk(mtx_);
            target = estimate_;
            if (reset_broadcast_) {  // neue Startschaetzung: springen statt glaetten
                broadcast_ = estimate_;
                reset_broadcast_ = false;
            }
        }
        // Broadcast-Pose sanft an die akzeptierte Schaetzung heranfuehren, damit
        // eine 2-s-Korrektur nicht als Sprung im TF-Baum (und damit im Planer-
        // Weltframe) landet. Bei 10 Hz Broadcast in ~1 s eingeschwungen.
        const Eigen::Vector3d t = broadcast_.translation()
            + smooth_alpha_ * (target.translation() - broadcast_.translation());
        // Normieren ist Pflicht: nicht-normierte Quaternionen schaukeln sich ueber
        // Quaternion->Matrix->Quaternion pro Tick auf (|q| bis 1.13, Saegezahn 20-40° Yaw)
        const Eigen::Quaterniond q =
            Eigen::Quaterniond(broadcast_.linear()).normalized()
                .slerp(smooth_alpha_, Eigen::Quaterniond(target.linear()).normalized()).normalized();
        broadcast_ = Eigen::Translation3d(t) * q;

        geometry_msgs::msg::TransformStamped tf = tf2::eigenToTransform(broadcast_);
        tf.header.stamp = now();
        tf.header.frame_id = map_frame_;
        tf.child_frame_id = odom_frame_;
        tf_broadcaster_->sendTransform(tf);
    }

    void match() {
        sensor_msgs::msg::PointCloud2::SharedPtr scan_msg;
        Eigen::Isometry3d guess;
        {
            std::lock_guard<std::mutex> lk(mtx_);
            scan_msg = last_scan_;
            guess = estimate_;
        }
        if (!scan_msg) return;

        auto scan = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*scan_msg, *scan);
        std::vector<int> keep;
        pcl::removeNaNFromPointCloud(*scan, *scan, keep);
        downsample(scan);
        if (scan->size() < 100) return;

        if (have_map_) {
            const auto t0 = std::chrono::steady_clock::now();
            // Scan liegt in Odom-Koordinaten → GICP-Ergebnis ist direkt map->odom
            gicp_.setInputSource(scan);
            pcl::PointCloud<pcl::PointXYZ> aligned;
            bool initialized;
            Eigen::Vector3d drone;
            {
                std::lock_guard<std::mutex> lk(mtx_);
                initialized = initialized_;
                drone = guess * odomBody().translation();
            }
            // Noch nicht lokalisiert (Start / neue Schaetzung): Startschaetzung ist
            // nur ungefaehr (Klick in der UI) -> Hypothesen um die Drohne herum,
            // beste Fitness gewinnt. Danach nur noch lokales Nachfuehren.
            // ponytail: festes Raster +-40° / +-1.5 m; fuer "irgendwo auf der Karte"
            // braeuchte es globale Deskriptoren (Scan Context o.ae.).
            if (!initialized) {
                double fit;
                guess = searchHypotheses(guess, drone, fit);
                // Vorgabe passt nicht -> ganze Karte absuchen (nur bei geladener Karte;
                // im Echtzeit-Modus ist die Karte per Definition um die Drohne herum)
                if (fit > search_fitness_max_ && !online_) guess = globalSearch(scan, guess, drone, fit);
                if (fit > search_fitness_max_) {
                    RCLCPP_WARN(get_logger(), "Keine passende Lage gefunden (beste Fitness %.3f > %.3f) — "
                        "Startpose in der UI setzen", fit, search_fitness_max_);
                    return;
                }
            }
            gicp_.align(aligned, guess.matrix().cast<float>());
            const double took =
                std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
            if (took > match_period_) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000,
                    "GICP braucht %.1f s > Periode %.1f s — voxel_size erhoehen "
                    "oder match_period anpassen", took, match_period_);
            }

            if (gicp_.hasConverged()) {
                acceptMatch(guess, aligned);
            } else {
                RCLCPP_WARN(get_logger(), "GICP nicht konvergiert — behalte letzte Korrektur");
            }
        }

        // Nach der Korrektur einfuegen, damit der Keyframe die bestmoegliche
        // Pose bekommt. Nur im Echtzeit-Modus — eine geladene Karte waechst nicht.
        if (online_) maybeInsertKeyframe(scan);
    }

    // odom->body aus /Odometry (Aufrufer haelt mtx_)
    Eigen::Isometry3d odomBody() const {
        return Eigen::Translation3d(odom_pos_) * odom_rot_;
    }

    Eigen::Isometry3d searchHypotheses(const Eigen::Isometry3d& guess, const Eigen::Vector3d& drone,
                                       double& best_fit) {
        Eigen::Isometry3d best = guess;
        best_fit = std::numeric_limits<double>::max();
        pcl::PointCloud<pcl::PointXYZ> aligned;
        const double offs[5][2] = {{0, 0}, {1.5, 0}, {-1.5, 0}, {0, 1.5}, {0, -1.5}};
        for (const double dyaw : {0.0, 0.35, -0.35, 0.7, -0.7}) {
            for (const auto& o : offs) {
                // um die Drohne drehen, dann verschieben
                const Eigen::Isometry3d h = Eigen::Translation3d(drone + Eigen::Vector3d(o[0], o[1], 0))
                    * Eigen::AngleAxisd(dyaw, Eigen::Vector3d::UnitZ())
                    * Eigen::Translation3d(-drone) * guess;
                gicp_.align(aligned, h.matrix().cast<float>());
                if (!gicp_.hasConverged()) continue;
                const double fit = gicp_.getFitnessScore(1.0);
                if (fit < best_fit) {
                    best_fit = fit;
                    best = rigid(gicp_.getFinalTransformation());
                }
            }
        }
        const Eigen::Vector3d fwd = (best * odomBodyLocked()).rotation() * Eigen::Vector3d::UnitX();
        const Eigen::Vector3d pos = best * odomBodyLocked().translation();
        RCLCPP_INFO(get_logger(), "Hypothesensuche: Drohne bei (%.2f, %.2f) Yaw %.0f°, Fitness %.3f",
            pos.x(), pos.y(), std::atan2(fwd.y(), fwd.x()) * 180.0 / M_PI, best_fit);
        return best;
    }

    // Globale Relokalisierung: Raster ueber die ganze Karte (1 m, 20°) mit billigem
    // Voxel-Treffer-Score, die besten Kandidaten per GICP verfeinern.
    // ponytail: Brute Force, bei Hallengroesse ~1-3 s einmalig; fuer grosse Karten
    // Scan-Context-Deskriptoren pro Keyframe statt Raster.
    Eigen::Isometry3d globalSearch(const pcl::PointCloud<pcl::PointXYZ>::Ptr& scan,
                                   const Eigen::Isometry3d& guess, const Eigen::Vector3d& drone, double& best_fit) {
        const auto t0 = std::chrono::steady_clock::now();
        constexpr double kVox = 0.3, kStep = 1.0, kYawStep = 20.0 * M_PI / 180.0;
        constexpr size_t kPts = 300, kCoarsePts = 50, kRescore = 500, kTop = 10;
        const auto key = [](double x, double y, double z) {
            return (static_cast<int64_t>(std::floor(x / kVox)) & 0x1FFFFF) << 42
                 | (static_cast<int64_t>(std::floor(y / kVox)) & 0x1FFFFF) << 21
                 | (static_cast<int64_t>(std::floor(z / kVox)) & 0x1FFFFF);
        };
        if (occ_.empty()) {  // Karte ist eingefroren -> einmal aufbauen, um 1 Voxel geweitet
            for (const auto& p : map_cloud_->points)
                for (int dx = -1; dx <= 1; ++dx) for (int dy = -1; dy <= 1; ++dy) for (int dz = -1; dz <= 1; ++dz)
                    occ_.insert(key(p.x + dx * kVox, p.y + dy * kVox, p.z + dz * kVox));
        }
        // Scanpunkte relativ zur Drohne (unter der alten Schaetzung), ausgeduennt
        std::vector<Eigen::Vector3d> rel;
        const size_t stride = std::max<size_t>(1, scan->size() / kPts);
        for (size_t i = 0; i < scan->size(); i += stride) {
            const auto& p = scan->points[i];
            rel.push_back(guess * Eigen::Vector3d(p.x, p.y, p.z) - drone);
        }
        pcl::PointXYZ lo, hi;
        pcl::getMinMax3D(*map_cloud_, lo, hi);
        struct Cand { int hits; double yaw, x, y; };
        const auto score = [&](const Cand& c, size_t step) {
            const Eigen::Matrix3d R = Eigen::AngleAxisd(c.yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
            int hits = 0;
            for (size_t i = 0; i < rel.size(); i += step) {
                const Eigen::Vector3d q = R * rel[i];
                hits += occ_.count(key(c.x + q.x(), c.y + q.y(), drone.z() + q.z()));
            }
            return hits;
        };
        const auto by_hits = [](const Cand& a, const Cand& b) { return a.hits > b.hits; };
        // grob mit ~50 Punkten ueber alle Hypothesen, die besten 500 mit allen Punkten nachbewerten
        std::vector<Cand> cands;
        const size_t coarse = std::max<size_t>(1, rel.size() / kCoarsePts);
        for (double yaw = 0; yaw < 2 * M_PI - 1e-6; yaw += kYawStep)
            for (double x = lo.x; x <= hi.x; x += kStep)
                for (double y = lo.y; y <= hi.y; y += kStep) {
                    Cand c{0, yaw, x, y};
                    c.hits = score(c, coarse);
                    cands.push_back(c);
                }
        const size_t n_all = cands.size(), resc = std::min(kRescore, n_all);
        std::partial_sort(cands.begin(), cands.begin() + resc, cands.end(), by_hits);
        cands.resize(resc);
        for (auto& c : cands) c.hits = score(c, 1);
        const size_t top = std::min(kTop, cands.size());
        std::partial_sort(cands.begin(), cands.begin() + top, cands.end(), by_hits);
        Eigen::Isometry3d best = guess;
        best_fit = std::numeric_limits<double>::max();
        pcl::PointCloud<pcl::PointXYZ> aligned;
        for (size_t i = 0; i < top; ++i) {
            const auto& c = cands[i];
            const Eigen::Isometry3d h = Eigen::Translation3d(c.x, c.y, drone.z())
                * Eigen::AngleAxisd(c.yaw, Eigen::Vector3d::UnitZ()) * Eigen::Translation3d(-drone) * guess;
            gicp_.align(aligned, h.matrix().cast<float>());
            if (!gicp_.hasConverged()) continue;
            const double fit = gicp_.getFitnessScore(1.0);
            if (fit < best_fit) {
                best_fit = fit;
                best = rigid(gicp_.getFinalTransformation());
            }
        }
        const Eigen::Isometry3d body = best * odomBodyLocked();
        const Eigen::Vector3d fwd = body.rotation() * Eigen::Vector3d::UnitX();
        RCLCPP_INFO(get_logger(), "Globale Suche (%zu Hypothesen, %.1f s): Drohne bei (%.2f, %.2f) Yaw %.0f°, "
            "Fitness %.3f, Trefferquote bester Kandidat %.0f %%", n_all,
            std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count(),
            body.translation().x(), body.translation().y(), std::atan2(fwd.y(), fwd.x()) * 180.0 / M_PI,
            best_fit, 100.0 * cands[0].hits / std::max<size_t>(1, rel.size()));
        return best;
    }

    Eigen::Isometry3d odomBodyLocked() {
        std::lock_guard<std::mutex> lk(mtx_);
        return odomBody();
    }

    // Anteil der (in map ausgerichteten) Scanpunkte mit Kartenpunkt in < inlier_dist_
    double inlierRatio(const pcl::PointCloud<pcl::PointXYZ>& aligned) {
        const auto tree = gicp_.getSearchMethodTarget();
        std::vector<int> idx(1);
        std::vector<float> d2(1);
        size_t in = 0;
        const float r2 = static_cast<float>(inlier_dist_ * inlier_dist_);
        for (const auto& p : aligned.points)
            if (tree->nearestKSearch(p, 1, idx, d2) > 0 && d2[0] < r2) ++in;
        return aligned.empty() ? 0.0 : static_cast<double>(in) / aligned.size();
    }

    void acceptMatch(const Eigen::Isometry3d& guess, const pcl::PointCloud<pcl::PointXYZ>& aligned) {
        const double fitness = gicp_.getFitnessScore(1.0);
        if (fitness > fitness_max_) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000,
                "Match verworfen (Fitness %.3f > %.3f)", fitness, fitness_max_);
            return;
        }

        const double inlier = inlierRatio(aligned);
        std_msgs::msg::Float32 q;
        q.data = static_cast<float>(inlier);
        inlier_pub_->publish(q);
        if (inlier < min_inlier_) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                "Match verworfen (nur %.0f %% des Scans auf der Karte < %.0f %%)", 100 * inlier, 100 * min_inlier_);
            return;
        }
        const Eigen::Isometry3d neu = rigid(gicp_.getFinalTransformation());
        const double jump = (neu.translation() - guess.translation()).norm();
        const double rot_jump =
            Eigen::AngleAxisd(neu.rotation() * guess.rotation().transpose()).angle();
        // Grosser Sprung trotz bestandener Fitness = fast immer Fehlregistrierung
        // (falsches lokales Minimum, z.B. verdrehte Pose in symmetrischer Halle).
        // Verwerfen und letzte gute Korrektur behalten — sonst wird auch der
        // GICP-Startwert fuers naechste Mal vergiftet.
        // ponytail: harte Schwelle; falls echte Relok-Recovery >1 m noetig wird,
        // jump_reject hochsetzen oder Hysterese (N Zyklen in Folge) ergaenzen.
        bool initialized;  // auch /initialpose-Callback schreibt -> unter Lock lesen
        {
            std::lock_guard<std::mutex> lk(mtx_);
            initialized = initialized_;
        }
        if (initialized && (jump > jump_reject_ || rot_jump > rot_jump_reject_)) {
            RCLCPP_WARN(get_logger(),
                "Relokalisierungs-Sprung %.2f m / %.2f rad — verworfen", jump, rot_jump);
            return;
        }
        {
            std::lock_guard<std::mutex> lk(mtx_);
            estimate_ = neu;
            initialized_ = true;
        }

        geometry_msgs::msg::PoseStamped pose;
        pose.header.stamp = now();
        pose.header.frame_id = map_frame_;
        pose.pose = tf2::toMsg(neu);
        pose_pub_->publish(pose);

        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 10000,
            "Relokalisiert: Korrektur (%.2f, %.2f, %.2f), Fitness %.3f, %.0f %% auf Karte",
            neu.translation().x(), neu.translation().y(),
            neu.translation().z(), fitness, 100 * inlier);
    }

    void maybeInsertKeyframe(const pcl::PointCloud<pcl::PointXYZ>::Ptr& scan) {
        Eigen::Isometry3d est;
        Eigen::Vector3d odom_pos;
        Eigen::Quaterniond odom_rot;
        {
            std::lock_guard<std::mutex> lk(mtx_);
            if (!have_odom_) return;
            est = estimate_;
            odom_pos = odom_pos_;
            odom_rot = odom_rot_;
        }
        const Eigen::Vector3d p = est * odom_pos;
        const Eigen::Vector3d fwd = est.rotation() * (odom_rot * Eigen::Vector3d::UnitX());
        const double yaw = std::atan2(fwd.y(), fwd.x());
        // Erstbesuchs-Gate: in schon kartierter Gegend nichts einfuegen, sonst
        // wuerden neue (gedriftete) Punkte die eingefrorene Referenz verwaessern
        // und GICP bestaetigt sich nur noch selbst.
        for (const auto& k : keyframes_) {
            const double dyaw = std::remainder(yaw - k.w(), 2.0 * M_PI);
            if ((k.head<3>() - p).norm() < keyframe_dist_ && std::abs(dyaw) < keyframe_yaw_) return;
        }

        pcl::PointCloud<pcl::PointXYZ> in_map;
        pcl::transformPointCloud(*scan, in_map, est.matrix().cast<float>());
        *map_cloud_ += in_map;
        downsample(map_cloud_);
        // ponytail: kompletter Target-Rebuild pro Keyframe (KdTree + Kovarianzen).
        // Reicht fuer Hallen-Groesse bei 2-s-Takt; bei grossen Karten auf
        // inkrementelle Zielstruktur umbauen.
        gicp_.setInputTarget(map_cloud_);
        keyframes_.emplace_back(p.x(), p.y(), p.z(), yaw);
        have_map_ = true;
        RCLCPP_INFO(get_logger(), "Keyframe %zu bei (%.1f, %.1f, %.1f) yaw %.0f°, Karte %zu Punkte",
            keyframes_.size(), p.x(), p.y(), p.z(), yaw * 180.0 / M_PI, map_cloud_->size());
    }

    std::string map_frame_, odom_frame_, map_save_path_;
    double voxel_size_ = 0.4;
    double fitness_max_ = 0.5;
    double search_fitness_max_ = 0.1;
    double inlier_dist_ = 0.5;
    double min_inlier_ = 0.6;
    double jump_reject_ = 1.0;
    double rot_jump_reject_ = 0.1;
    double smooth_alpha_ = 0.1;
    double keyframe_dist_ = 2.0;
    double keyframe_yaw_ = 0.8;
    double match_period_ = 2.0;
    bool online_ = true;
    bool have_map_ = false;      // nur im Match-Thread benutzt
    bool initialized_ = false;   // unter mtx_ (Match-Thread + /initialpose)

    std::mutex mtx_;  // schuetzt estimate_, last_scan_, odom_pos_, odom_rot_, have_odom_
    Eigen::Isometry3d estimate_ = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d broadcast_ = Eigen::Isometry3d::Identity();  // nur TF-Timer
    Eigen::Vector3d odom_pos_ = Eigen::Vector3d::Zero();
    Eigen::Quaterniond odom_rot_ = Eigen::Quaterniond::Identity();
    bool have_odom_ = false;
    bool reset_broadcast_ = false;  // unter mtx_
    double initial_pitch_ = 0.0;
    sensor_msgs::msg::PointCloud2::SharedPtr last_scan_;

    // Nur im Match-Thread benutzt (Callback-Group ist MutuallyExclusive):
    pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud_;
    std::vector<Eigen::Vector4d> keyframes_;  // x, y, z, yaw in map
    std::unordered_set<int64_t> occ_;          // belegte Kartenvoxel fuer die globale Suche

    rclcpp::CallbackGroup::SharedPtr match_group_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr scan_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initialpose_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr inlier_pub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_srv_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr match_timer_, tf_timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LidarRelocalizationNode>();
    rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 2);
    exec.add_node(node);
    exec.spin();
    rclcpp::shutdown();
    return 0;
}
