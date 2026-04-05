// package.xml: <depend>octomap</depend> <depend>octomap_msgs</depend>
// CMakeLists:  find_package(octomap REQUIRED) + find_package(octomap_msgs REQUIRED)
//              target_link_libraries(... ${OCTOMAP_LIBRARIES})
//              target_include_directories(... ${OCTOMAP_INCLUDE_DIRS})

#include "uav_planner/nav_node.hpp"

#include <queue>
#include <unordered_map>
#include <cmath>
#include <algorithm>
#include <limits>

NavNode::NavNode() : Node("nav_node") {
    octomap_sub_ = create_subscription<octomap_msgs::msg::Octomap>(
        "/octomap_binary",
        rclcpp::QoS(1).reliable().transient_local(),
        std::bind(&NavNode::octomapCallback, this, std::placeholders::_1));

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
        std::chrono::seconds(1),
        std::bind(&NavNode::computeAndPublishEsdf, this));
}

void NavNode::octomapCallback(const octomap_msgs::msg::Octomap::SharedPtr msg) {
    tree_.reset(dynamic_cast<octomap::OcTree*>(
        octomap_msgs::msgToMap(*msg)));

    if (!tree_)
        RCLCPP_ERROR(get_logger(), "Deserialisierung fehlgeschlagen");
    else
        RCLCPP_INFO(get_logger(), "Karte empfangen — %zu Knoten",
            tree_->getNumLeafNodes());
}

void NavNode::dronePoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    drone_pose_ = *msg;
    RCLCPP_INFO(get_logger(), "Drohnenpose aktualisiert: (%.2f, %.2f, %.2f)",
        drone_pose_.pose.position.x,
        drone_pose_.pose.position.y,
        drone_pose_.pose.position.z);
}

bool NavNode::isGoalDifferent(const geometry_msgs::msg::PoseStamped& new_goal) const {
    double dx = new_goal.pose.position.x - goal_pose_.pose.position.x;
    double dy = new_goal.pose.position.y - goal_pose_.pose.position.y;
    double dz = new_goal.pose.position.z - goal_pose_.pose.position.z;
    return std::sqrt(dx*dx + dy*dy + dz*dz) > goal_change_thresh_;
}

void NavNode::goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    RCLCPP_INFO(get_logger(), "Zielpose empfangen: (%.2f, %.2f, %.2f)",
        msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);

    // Neues Ziel nur akzeptieren wenn es sich vom aktuellen unterscheidet
    if (state_ == State::NAVIGATING && !isGoalDifferent(*msg)) {
        RCLCPP_INFO(get_logger(), "Neues Ziel zu nah am aktuellen — ignoriert");
        return;
    }

    goal_pose_ = *msg;
    state_ = State::NAVIGATING;
    RCLCPP_INFO(get_logger(), "State -> NAVIGATING");

    // Sofort planen, dann Timer starten für Replan jede Sekunde
    AstarPlanner();

    if (!replan_timer_) {
        replan_timer_ = create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&NavNode::replanTimerCallback, this));
    }
}

void NavNode::replanTimerCallback() {
    if (state_ != State::NAVIGATING) {
        return;
    }

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

    RCLCPP_INFO(get_logger(), "Replan (Entfernung zum Ziel: %.2f m)", dist);
    AstarPlanner();
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

void NavNode::AstarPlanner() {
    RCLCPP_INFO(get_logger(), "A*-Planer aufgerufen");

    if (!tree_) {
        RCLCPP_WARN(get_logger(), "Keine OctoMap vorhanden");
        return;
    }

    const double res = tree_->getResolution();
    const unsigned int search_depth = tree_->getTreeDepth() - 1;  // Tiefe 15 → 0.6m Voxel
    const double step_size = tree_->getNodeSize(search_depth);

    RCLCPP_INFO(get_logger(), "Planung auf Tiefe %u (Voxel: %.2f m)", search_depth, step_size);

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
        return;
    }

    // Collision check at coarser depth — one search() replaces entire inflation loop
    auto is_collision_free = [&](const octomap::OcTreeKey& k) -> bool {
        octomap::OcTreeNode* node = tree_->search(k, search_depth);
        return !node || !tree_->isNodeOccupied(node);
    };

    if (!is_collision_free(start_key)) {
        RCLCPP_ERROR(get_logger(), "Startposition ist blockiert");
        return;
    }
    if (!is_collision_free(goal_key)) {
        RCLCPP_ERROR(get_logger(), "Zielposition ist blockiert");
        return;
    }

    // Weighted A* heuristic: w > 1.0 biases toward goal (faster, suboptimal by factor w)
    constexpr double w = 2.0;
    auto heuristic = [](const octomap::OcTreeKey& a, const octomap::OcTreeKey& b) -> double {
        double dx = static_cast<int>(a[0]) - static_cast<int>(b[0]);
        double dy = static_cast<int>(a[1]) - static_cast<int>(b[1]);
        double dz = static_cast<int>(a[2]) - static_cast<int>(b[2]);
        return w * std::sqrt(dx*dx + dy*dy + dz*dz);
    };

    // 6-connected neighbors (face-adjacent) for speed
    static constexpr int dirs[6][3] = {
        {1,0,0}, {-1,0,0}, {0,1,0}, {0,-1,0}, {0,0,1}, {0,0,-1}
    };

    using KeyHash = octomap::OcTreeKey::KeyHash;

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

        for (const auto& d : dirs) {
            octomap::OcTreeKey nbr_key(
                current.key[0] + d[0],
                current.key[1] + d[1],
                current.key[2] + d[2]);

            if (!is_collision_free(nbr_key))
                continue;

            double tentative_g = cur_g + 1.0;  // uniform cost for 6-connected

            auto it = g_score.find(nbr_key);
            if (it != g_score.end() && tentative_g >= it->second)
                continue;

            g_score[nbr_key] = tentative_g;
            came_from[nbr_key] = current.key;
            open.push({nbr_key, tentative_g + heuristic(nbr_key, goal_key)});
        }
    }

    if (!found) {
        RCLCPP_ERROR(get_logger(), "Kein Pfad gefunden (%zu Knoten expandiert)", expanded);
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
    if (!tree_ || drone_pose_.header.stamp.sec == 0) return;

    const double cx = drone_pose_.pose.position.x;
    const double cy = drone_pose_.pose.position.y;
    const double cz = drone_pose_.pose.position.z;

    const int nx = static_cast<int>(esdf_size_x_ / esdf_res_);
    const int ny = static_cast<int>(esdf_size_y_ / esdf_res_);
    const int nz = static_cast<int>(esdf_size_z_ / esdf_res_);

    // Ursprung (min-Ecke) des lokalen Feldes
    const double ox = cx - esdf_size_x_ / 2.0;
    const double oy = cy - esdf_size_y_ / 2.0;
    const double oz = cz - esdf_size_z_ / 2.0;

    constexpr double INF = 1e20;
    std::vector<double> grid(nx * ny * nz, INF);

    // Octomap → lokales Grid: besetzte Voxel markieren (Distanz = 0)
    for (auto it = tree_->begin_leafs(), end = tree_->end_leafs(); it != end; ++it) {
        if (!tree_->isNodeOccupied(*it)) continue;

        double wx = it.getX(), wy = it.getY(), wz = it.getZ();
        int ix = static_cast<int>((wx - ox) / esdf_res_);
        int iy = static_cast<int>((wy - oy) / esdf_res_);
        int iz = static_cast<int>((wz - oz) / esdf_res_);

        if (ix >= 0 && ix < nx && iy >= 0 && iy < ny && iz >= 0 && iz < nz)
            grid[(iz * ny + iy) * nx + ix] = 0.0;
    }

    // 3D EDT berechnen (Ergebnis: quadrierte Distanzen in Voxel-Einheiten)
    edt3d(grid, nx, ny, nz);

    // Squared voxel distances → metrische Distanzen
    // grid[i] enthält jetzt d²(voxel), also dist_m = sqrt(grid[i]) * esdf_res_
    auto distMetric = [&](int idx) -> double {
        return std::sqrt(grid[idx]) * esdf_res_;
    };

    // Distanz → Costmap-Wert [0..100]
    auto distToCost = [&](double dist) -> int8_t {
        if (dist <= 0.0) return 100;
        if (dist >= esdf_max_dist_) return 0;
        return static_cast<int8_t>(100.0 * (1.0 - dist / esdf_max_dist_));
    };

    auto stamp = now();

    // --- XY-Slice (horizontal, auf Drohnenhöhe) ---
    {
        int iz = static_cast<int>((cz - oz) / esdf_res_);
        iz = std::clamp(iz, 0, nz - 1);

        nav_msgs::msg::OccupancyGrid og;
        og.header.stamp = stamp;
        og.header.frame_id = "map";
        og.info.resolution = esdf_res_;
        og.info.width = nx;
        og.info.height = ny;
        og.info.origin.position.x = ox;
        og.info.origin.position.y = oy;
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
        int iy = static_cast<int>((cy - oy) / esdf_res_);
        iy = std::clamp(iy, 0, ny - 1);

        nav_msgs::msg::OccupancyGrid og;
        og.header.stamp = stamp;
        og.header.frame_id = "map";
        og.info.resolution = esdf_res_;
        og.info.width = nx;
        og.info.height = nz;
        og.info.origin.position.x = ox;
        og.info.origin.position.y = cy;
        og.info.origin.position.z = oz;
        og.info.origin.orientation.w = 1.0;
        og.data.resize(nx * nz);

        for (int z = 0; z < nz; ++z)
            for (int x = 0; x < nx; ++x)
                og.data[z * nx + x] = distToCost(distMetric((z * ny + iy) * nx + x));

        esdf_slice_xz_pub_->publish(og);
    }

    RCLCPP_INFO(get_logger(), "ESDF aktualisiert (%dx%dx%d, Zentrum: %.1f/%.1f/%.1f)",
        nx, ny, nz, cx, cy, cz);

    // Lokale Planung auf dem ESDF
    if (state_ == State::NAVIGATING && !last_global_path_.poses.empty())
        localPlanner(grid, nx, ny, nz, ox, oy, oz);
}

// --- Lokaler Planer ---

double NavNode::sampleEsdf(const std::vector<double>& esdf_grid,
                           int nx, int ny, int nz,
                           double ox, double oy, double oz,
                           double wx, double wy, double wz) const {
    int ix = static_cast<int>((wx - ox) / esdf_res_);
    int iy = static_cast<int>((wy - oy) / esdf_res_);
    int iz = static_cast<int>((wz - oz) / esdf_res_);
    if (ix < 0 || ix >= nx || iy < 0 || iy >= ny || iz < 0 || iz >= nz)
        return 0.0;  // Außerhalb = unsicher
    return std::sqrt(esdf_grid[(iz * ny + iy) * nx + ix]) * esdf_res_;
}

void NavNode::localPlanner(const std::vector<double>& esdf_grid,
                           int nx, int ny, int nz,
                           double ox, double oy, double oz) {
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
            local_goal_idx = i;
            break;
        }
    }

    if (local_goal_idx < 0) {
        RCLCPP_WARN(get_logger(), "Kein globaler Wegpunkt im lokalen Fenster");
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
        return std::sqrt(esdf_grid[(iz * ny + iy) * nx + ix]) * esdf_res_;
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

    // Traversal-Kosten: Bewegung + ESDF-Penalty
    auto traversalCost = [&](const Cell& c, double move_dist) -> double {
        double d = esdfDist(c.x, c.y, c.z);
        if (d < hard_collision_dist_) return 1e10;  // Quasi-blockiert
        double penalty = 0.0;
        if (d < safety_dist_) {
            double ratio = (safety_dist_ - d) / safety_dist_;
            penalty = obstacle_penalty_weight_ * ratio * ratio;
        }
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
    smoothPath(path, esdf_grid, nx, ny, nz, ox, oy, oz);

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
    local_path_pub_->publish(path_msg);

    RCLCPP_INFO(get_logger(), "Lokaler Pfad: %zu Wegpunkte (%zu expandiert)", path.size(), expanded);
}

void NavNode::smoothPath(std::vector<std::array<double, 3>>& path,
                         const std::vector<double>& esdf_grid,
                         int nx, int ny, int nz,
                         double ox, double oy, double oz) {
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
            double dist_here = sampleEsdf(esdf_grid, nx, ny, nz, ox, oy, oz,
                                          p[0], p[1], p[2]);
            std::array<double, 3> obs_delta = {0, 0, 0};
            if (dist_here < safety_dist_) {
                for (int d = 0; d < 3; ++d) {
                    std::array<double, 3> p_plus = p, p_minus = p;
                    p_plus[d] += grad_step;
                    p_minus[d] -= grad_step;
                    double d_plus = sampleEsdf(esdf_grid, nx, ny, nz, ox, oy, oz,
                                               p_plus[0], p_plus[1], p_plus[2]);
                    double d_minus = sampleEsdf(esdf_grid, nx, ny, nz, ox, oy, oz,
                                                p_minus[0], p_minus[1], p_minus[2]);
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
            double new_dist = sampleEsdf(esdf_grid, nx, ny, nz, ox, oy, oz,
                                         p[0], p[1], p[2]);
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