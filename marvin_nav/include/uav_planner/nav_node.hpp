#ifndef UAV_PLANNER__NAV_NODE_HPP_
#define UAV_PLANNER__NAV_NODE_HPP_

#include <memory>
#include <string>
#include <vector>
#include <array>
#include <cstdint>

#include <rclcpp/rclcpp.hpp>
#include <octomap/octomap.h>
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/conversions.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

class NavNode : public rclcpp::Node {
public:
    NavNode();

    enum class State { IDLE, NAVIGATING };

private:
    // --- Mapping: OctoMap in-process (ersetzt octomap_server) ---
    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void saveMapCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                         std::shared_ptr<std_srvs::srv::Trigger::Response> res);
    void publishOctomap();

    std::unique_ptr<octomap::OcTree> tree_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr octomap_pub_;
    // Besetzte Voxel-Zentren für RViz (PointCloud2-Display, kein Octomap-Plugin nötig)
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr octomap_points_pub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_map_srv_;
    std::string map_file_;       // .bt laden beim Start / speichern via ~/save_map
    std::string sensor_frame_;   // Frame des Sensor-Ursprungs; leer = Frame der Cloud
    rclcpp::Time last_insert_time_{0, 0, RCL_ROS_TIME};
    int inserts_since_publish_ = 0;

    // Sensor-Modell — identisch zum vorher getunten octomap_server-Setup
    static constexpr double map_res_ = 0.3;                 // OctoMap Voxelgröße [m]
    static constexpr double max_range_ = 25.0;              // Maximale Sensor-Reichweite [m]
    static constexpr double prob_hit_ = 0.75;               // Wahrscheinlichkeit für belegte Voxel
    static constexpr double prob_miss_ = 0.25;              // Wahrscheinlichkeit für freie Voxel
    static constexpr double clamp_min_ = 0.2;               // Minimale log-odds Wahrscheinlichkeit
    static constexpr double clamp_max_ = 0.90;              // Maximale log-odds Wahrscheinlichkeit
    static constexpr double cloud_z_min_ = -2.0;            // Minimale Z-Koordinate der Cloud [m]
    static constexpr double cloud_z_max_ = 15.0;            // Maximale Z-Koordinate der Cloud [m]
    static constexpr double insert_min_period_ = 0.4;       // Min. Zeitabstand zwischen Inserts [s] — Raycasting ist teuer
    static constexpr int publish_every_n_inserts_ = 5;      // Publish /octomap_binary nach N Inserts — RViz-Drosselung

    // --- TF ---
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
    // map->base_link aus TF (relokalisierungs-korrigiert); Fallback: MAVROS-Topic
    bool updateDronePoseFromTf();

    // --- Pose / Ziel ---
    void dronePoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    geometry_msgs::msg::PoseStamped drone_pose_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr drone_pose_sub_;
    void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    geometry_msgs::msg::PoseStamped goal_pose_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    double distanceToGoal();
    bool isGoalDifferent(const geometry_msgs::msg::PoseStamped& new_goal) const;

    // --- Globaler Planer ---
    void AstarPlanner();
    void replanTimerCallback();
    void planFailed(const char* what);
    bool globalPathClear();  // committete Route noch frei? (Sticky-Replan)

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

    // State machine
    State state_ = State::IDLE;
    rclcpp::TimerBase::SharedPtr replan_timer_;
    int consecutive_failures_ = 0;
    static constexpr int max_failures_ = 5;              // danach Abbruch + Halte-Signal
    static constexpr double goal_tolerance_ = 0.3;       // Ziel erreicht
    static constexpr double goal_change_thresh_ = 0.5;   // Neues Ziel muss so weit abweichen
    static constexpr double goal_yaw_thresh_ = 0.2;      // ... oder im Yaw abweichen [rad]
    static constexpr double unknown_penalty_weight_ = 0.3; // Kosten für unbekannte Voxel (global UND lokal)
    static constexpr double tf_pose_max_age_ = 1.0;      // TF map->base_link älter als das [s] = stale -> Topic-Fallback

    // --- ESDF (lokales Fenster) ---
    void computeAndPublishEsdf();
    static void edt1d(std::vector<double>& f, int n);
    void edt3d(std::vector<double>& grid, int nx, int ny, int nz);
    void publishEsdfSlices();
    void validateGlobalPath();

    rclcpp::TimerBase::SharedPtr esdf_timer_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr esdf_slice_xy_pub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr esdf_slice_xz_pub_;

    // ESDF-Zustand als Member — Distanzen NUR zu bekannt-besetzten Voxeln,
    // Unbekannt wird über die Maske bestraft statt hart blockiert (gleiche
    // Politik wie im globalen Planer, sonst Deadlock an der Kartengrenze)
    std::vector<double> esdf_grid_;      // quadrierte Distanzen [voxel²]
    std::vector<uint8_t> esdf_unknown_;  // 1 = Zelle von keinem bekannten Blatt abgedeckt
    int esdf_nx_ = 0, esdf_ny_ = 0, esdf_nz_ = 0;
    double esdf_ox_ = 0.0, esdf_oy_ = 0.0, esdf_oz_ = 0.0;

    double esdf_center_x_ = 0.0;
    double esdf_center_y_ = 0.0;
    double esdf_center_z_ = 0.0;
    bool esdf_initialized_ = false;
    bool octomap_updated_ = false;                        // Neue Daten seit letzter ESDF-Berechnung
    static constexpr double esdf_recompute_dist_ = 2.5;   // Neuberechnung ab dieser Entfernung [m]

    static constexpr double esdf_res_ = 0.25;        // ESDF Voxelgröße [m]
    static constexpr double esdf_size_x_ = 20.0;     // Feldgröße X [m]
    static constexpr double esdf_size_y_ = 20.0;     // Feldgröße Y [m]
    static constexpr double esdf_size_z_ = 10.0;     // Feldgröße Z [m] — hoch genug zum Drüberfliegen
    static constexpr double esdf_max_dist_ = 2.0;    // Max-Distanz für Costmap-Mapping [m]

    // --- Lokaler Planer (ESDF-basiert) ---
    void localPlanner();
    void smoothPath(std::vector<std::array<double, 3>>& path);
    double sampleEsdf(double wx, double wy, double wz) const;

    nav_msgs::msg::Path last_global_path_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr local_path_pub_;

    static constexpr double safety_dist_ = 1.5;             // Sicherheitsabstand [m]
    static constexpr double obstacle_penalty_weight_ = 80.0;
    static constexpr double hard_collision_dist_ = 0.6;    // Absoluter Mindestabstand [m]
    static constexpr int smooth_iterations_ = 30;
    static constexpr double smooth_weight_ = 0.3;           // Glättungsgewicht
    static constexpr double obstacle_weight_ = 1.0;         // Hindernisabstoßungs-Gewicht
};

#endif  // UAV_PLANNER__NAV_NODE_HPP_
