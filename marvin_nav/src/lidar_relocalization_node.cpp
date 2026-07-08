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
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
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
        jump_reject_ = declare_parameter("jump_reject", 1.0);   // Korrektursprung > x m verwerfen [m]
        rot_jump_reject_ = declare_parameter("rot_jump_reject", 0.3); // dito Rotation [rad]
        smooth_alpha_ = declare_parameter("smooth_alpha", 0.1); // Glaettung pro Broadcast-Tick [0..1]
        keyframe_dist_ = declare_parameter("keyframe_dist", 2.0); // Keyframe-Abstand / Erstbesuchs-Radius [m]
        match_period_ = declare_parameter("match_period", 2.0);

        // Startschätzung (z.B. bekannter Startplatz auf der Karte)
        const double ix = declare_parameter("initial_x", 0.0);
        const double iy = declare_parameter("initial_y", 0.0);
        const double iz = declare_parameter("initial_z", 0.0);
        const double iyaw = declare_parameter("initial_yaw", 0.0);
        estimate_ = Eigen::Translation3d(ix, iy, iz)
                  * Eigen::AngleAxisd(iyaw, Eigen::Vector3d::UnitZ());
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

        gicp_.setMaxCorrespondenceDistance(2.0);
        gicp_.setMaximumIterations(50);

        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
            "/relocalization/pose", 10);
        scan_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            scan_topic, rclcpp::SensorDataQoS(),
            [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) {
                std::lock_guard<std::mutex> lk(mtx_);
                last_scan_ = msg;
            });
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            odom_topic, rclcpp::SensorDataQoS(),
            [this](nav_msgs::msg::Odometry::SharedPtr msg) {
                std::lock_guard<std::mutex> lk(mtx_);
                odom_pos_ = Eigen::Vector3d(msg->pose.pose.position.x,
                                            msg->pose.pose.position.y,
                                            msg->pose.pose.position.z);
                have_odom_ = true;
            });

        // GICP darf den 10-Hz-TF-Broadcast nicht blockieren (stale TF ->
        // pursuit/planner fallen auf MAVROS-Pose zurueck). Match-Timer laeuft
        // in eigener Callback-Group, main() spinnt multithreaded.
        match_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        match_timer_ = create_wall_timer(
            std::chrono::duration<double>(match_period_),
            std::bind(&LidarRelocalizationNode::match, this), match_group_);
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
        }
        // Broadcast-Pose sanft an die akzeptierte Schaetzung heranfuehren, damit
        // eine 2-s-Korrektur nicht als Sprung im TF-Baum (und damit im Planer-
        // Weltframe) landet. Bei 10 Hz Broadcast in ~1 s eingeschwungen.
        const Eigen::Vector3d t = broadcast_.translation()
            + smooth_alpha_ * (target.translation() - broadcast_.translation());
        const Eigen::Quaterniond q =
            Eigen::Quaterniond(broadcast_.rotation())
                .slerp(smooth_alpha_, Eigen::Quaterniond(target.rotation()));
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
            gicp_.align(aligned, guess.matrix().cast<float>());
            const double took =
                std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
            if (took > match_period_) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000,
                    "GICP braucht %.1f s > Periode %.1f s — voxel_size erhoehen "
                    "oder match_period anpassen", took, match_period_);
            }

            if (gicp_.hasConverged()) {
                acceptMatch(guess);
            } else {
                RCLCPP_WARN(get_logger(), "GICP nicht konvergiert — behalte letzte Korrektur");
            }
        }

        // Nach der Korrektur einfuegen, damit der Keyframe die bestmoegliche
        // Pose bekommt. Nur im Echtzeit-Modus — eine geladene Karte waechst nicht.
        if (online_) maybeInsertKeyframe(scan);
    }

    void acceptMatch(const Eigen::Isometry3d& guess) {
        const double fitness = gicp_.getFitnessScore(1.0);
        if (fitness > fitness_max_) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000,
                "Match verworfen (Fitness %.3f > %.3f)", fitness, fitness_max_);
            return;
        }

        const Eigen::Isometry3d neu(gicp_.getFinalTransformation().cast<double>());
        const double jump = (neu.translation() - guess.translation()).norm();
        const double rot_jump =
            Eigen::AngleAxisd(neu.rotation() * guess.rotation().transpose()).angle();
        // Grosser Sprung trotz bestandener Fitness = fast immer Fehlregistrierung
        // (falsches lokales Minimum, z.B. verdrehte Pose in symmetrischer Halle).
        // Verwerfen und letzte gute Korrektur behalten — sonst wird auch der
        // GICP-Startwert fuers naechste Mal vergiftet.
        // ponytail: harte Schwelle; falls echte Relok-Recovery >1 m noetig wird,
        // jump_reject hochsetzen oder Hysterese (N Zyklen in Folge) ergaenzen.
        if (initialized_ && (jump > jump_reject_ || rot_jump > rot_jump_reject_)) {
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
            "Relokalisiert: Korrektur (%.2f, %.2f, %.2f), Fitness %.3f",
            neu.translation().x(), neu.translation().y(),
            neu.translation().z(), fitness);
    }

    void maybeInsertKeyframe(const pcl::PointCloud<pcl::PointXYZ>::Ptr& scan) {
        Eigen::Isometry3d est;
        Eigen::Vector3d odom_pos;
        {
            std::lock_guard<std::mutex> lk(mtx_);
            if (!have_odom_) return;
            est = estimate_;
            odom_pos = odom_pos_;
        }
        const Eigen::Vector3d p = est * odom_pos;
        // Erstbesuchs-Gate: in schon kartierter Gegend nichts einfuegen, sonst
        // wuerden neue (gedriftete) Punkte die eingefrorene Referenz verwaessern
        // und GICP bestaetigt sich nur noch selbst.
        for (const auto& k : keyframe_positions_) {
            if ((k - p).norm() < keyframe_dist_) return;
        }

        pcl::PointCloud<pcl::PointXYZ> in_map;
        pcl::transformPointCloud(*scan, in_map, est.matrix().cast<float>());
        *map_cloud_ += in_map;
        downsample(map_cloud_);
        // ponytail: kompletter Target-Rebuild pro Keyframe (KdTree + Kovarianzen).
        // Reicht fuer Hallen-Groesse bei 2-s-Takt; bei grossen Karten auf
        // inkrementelle Zielstruktur umbauen.
        gicp_.setInputTarget(map_cloud_);
        keyframe_positions_.push_back(p);
        have_map_ = true;
        RCLCPP_INFO(get_logger(), "Keyframe %zu bei (%.1f, %.1f, %.1f), Karte %zu Punkte",
            keyframe_positions_.size(), p.x(), p.y(), p.z(), map_cloud_->size());
    }

    std::string map_frame_, odom_frame_;
    double voxel_size_ = 0.4;
    double fitness_max_ = 0.5;
    double jump_reject_ = 1.0;
    double rot_jump_reject_ = 0.3;
    double smooth_alpha_ = 0.1;
    double keyframe_dist_ = 2.0;
    double match_period_ = 2.0;
    bool online_ = true;
    bool have_map_ = false;      // nur im Match-Thread benutzt
    bool initialized_ = false;   // unter mtx_ geschrieben, nur Match-Thread liest

    std::mutex mtx_;  // schuetzt estimate_, last_scan_, odom_pos_, have_odom_
    Eigen::Isometry3d estimate_ = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d broadcast_ = Eigen::Isometry3d::Identity();  // nur TF-Timer
    Eigen::Vector3d odom_pos_ = Eigen::Vector3d::Zero();
    bool have_odom_ = false;
    sensor_msgs::msg::PointCloud2::SharedPtr last_scan_;

    // Nur im Match-Thread benutzt (Callback-Group ist MutuallyExclusive):
    pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud_;
    std::vector<Eigen::Vector3d> keyframe_positions_;

    rclcpp::CallbackGroup::SharedPtr match_group_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr scan_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
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
