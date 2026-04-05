#ifndef UAV_PLANNER__NAV_NODE_HPP_
#define UAV_PLANNER__NAV_NODE_HPP_

#include <memory>
#include <vector>
#include <array>

#include <rclcpp/rclcpp.hpp>
#include <octomap/octomap.h>
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/conversions.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

class NavNode : public rclcpp::Node {
public:
    NavNode();

    enum class State { IDLE, NAVIGATING };

private:
    void octomapCallback(const octomap_msgs::msg::Octomap::SharedPtr msg);

    std::unique_ptr<octomap::OcTree> tree_;
    rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr octomap_sub_;
    void dronePoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    geometry_msgs::msg::PoseStamped drone_pose_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr drone_pose_sub_;
    void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    geometry_msgs::msg::PoseStamped goal_pose_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    double distanceToGoal();
    bool isGoalDifferent(const geometry_msgs::msg::PoseStamped& new_goal) const;

    void AstarPlanner();
    void replanTimerCallback();

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

    // State machine
    State state_ = State::IDLE;
    rclcpp::TimerBase::SharedPtr replan_timer_;
    static constexpr double goal_tolerance_ = 0.3;      // Ziel erreicht
    static constexpr double goal_change_thresh_ = 0.5;   // Neues Ziel muss so weit abweichen

    // ESDF
    void computeAndPublishEsdf();
    static void edt1d(std::vector<double>& f, int n);
    void edt3d(std::vector<double>& grid, int nx, int ny, int nz);

    rclcpp::TimerBase::SharedPtr esdf_timer_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr esdf_slice_xy_pub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr esdf_slice_xz_pub_;

    static constexpr double esdf_res_ = 0.25;        // ESDF Voxelgröße [m]
    static constexpr double esdf_size_x_ = 8.0;     // Feldgröße X [m]
    static constexpr double esdf_size_y_ = 8.0;     // Feldgröße Y [m]
    static constexpr double esdf_size_z_ = 3.0;     // Feldgröße Z [m]
    static constexpr double esdf_max_dist_ = 2.0;   // Max-Distanz für Costmap-Mapping [m]

    // Lokaler Planer (ESDF-basiert)
    void localPlanner(const std::vector<double>& esdf_grid,
                      int nx, int ny, int nz,
                      double ox, double oy, double oz);
    void smoothPath(std::vector<std::array<double, 3>>& path,
                    const std::vector<double>& esdf_grid,
                    int nx, int ny, int nz,
                    double ox, double oy, double oz);
    double sampleEsdf(const std::vector<double>& esdf_grid,
                      int nx, int ny, int nz,
                      double ox, double oy, double oz,
                      double wx, double wy, double wz) const;

    nav_msgs::msg::Path last_global_path_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr local_path_pub_;

    static constexpr double safety_dist_ = 0.7;            // Sicherheitsabstand [m]
    static constexpr double obstacle_penalty_weight_ = 15.0;
    static constexpr double hard_collision_dist_ = 0.15;    // Absoluter Mindestabstand [m]
    static constexpr int smooth_iterations_ = 80;
    static constexpr double smooth_weight_ = 0.35;          // Glättungsgewicht
    static constexpr double obstacle_weight_ = 0.5;         // Hindernisabstoßungs-Gewicht
};

#endif  // UAV_PLANNER__NAV_NODE_HPP_
