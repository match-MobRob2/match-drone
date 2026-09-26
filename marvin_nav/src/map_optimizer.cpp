// Offline Pose-Graph-Optimierung einer Mapping-Aufnahme (GTSAM).
//
//   ros2 run marvin_nav map_optimizer <map_dir>
//
// Liest die Keyframes des keyframe_recorder (<map_dir>/keyframes: poses.csv,
// kf_NNNNN.pcd im FAST-LIO-body-Frame, mount_pitch), baut einen Pose-Graphen
//   - Odometrie-Kanten zwischen aufeinanderfolgenden Keyframes (FAST-LIO relativ)
//   - Loop-Closures: GICP Keyframe j gegen Submap um Keyframe i, wenn beide
//     raeumlich nah, aber zeitlich weit auseinander sind (Huber-robust)
// optimiert ihn und schreibt aus den optimierten Posen neu:
//   <map_dir>/map.pcd       Referenzkarte fuer lidar_relocalization (map_pcd)
//   <map_dir>/map.bt        OctoMap fuer den Planer (map_file)
//   <map_dir>/poses_opt.csv optimierte Keyframe-Posen (map-Frame)
// map = Ry(mount_pitch) * camera_init — derselbe gelevelte Frame, den die
// Relokalisierung beim Start eines Mapping-Flugs aufspannt.
//
// Bearbeitung ueber <map_dir>/map.yaml (wird beim ersten Lauf angelegt):
//   origin: neuer Nullpunkt + Blickrichtung, angegeben im bisherigen Frame
//   crop:   Boxen im neuen Frame, deren Punkte rausfliegen (z.B. temporaere Objekte)
// Die Keyframes bleiben unangetastet — jede Bearbeitung ist per Neulauf umkehrbar.

#include <cmath>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include <ctime>
#include <filesystem>

#include <Eigen/Geometry>
#include <yaml-cpp/yaml.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <octomap/octomap.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/gicp.h>

using Cloud = pcl::PointCloud<pcl::PointXYZ>;

namespace {
// Loop-Closure-Suche
constexpr double kLoopRadius = 8.0;     // Kandidaten naeher als x m (Odometrie-Positionen) [m]
constexpr double kLoopMinGap = 20.0;    // ... und mindestens x s spaeter aufgenommen [s]
constexpr int kLoopSkip = 5;            // nach einer Loop-Kante x Keyframes Pause
constexpr int kSubmapHalf = 3;          // Submap = Keyframe i +- x
constexpr double kGicpVoxel = 0.25;     // Downsampling fuer GICP [m]
constexpr double kGicpMaxCorr = 1.0;    // [m]
constexpr double kFitnessMax = 0.05;    // mittl. quadr. Fehler [m²]
constexpr double kMaxCorrTrans = 8.0;   // Loop darf Odometrie max. so weit korrigieren [m]
constexpr double kMaxCorrRot = 0.8;     // ... bzw. so weit drehen [rad]
// GICP-Starthypothesen um die Hochachse: bei 20-30° Yaw-Drift konvergiert GICP
// von der Odometrie-Schaetzung allein nicht
constexpr double kYawHypoStep = 10.0 * M_PI / 180.0;
constexpr int kYawHypoN = 4;            // +-4 Schritte = +-40°
// OctoMap — wie nav_node.hpp (map_res_, max_range_, prob_*, clamp_*, cloud_z_*)
constexpr double kMapRes = 0.3, kMaxRange = 25.0, kProbHit = 0.75, kProbMiss = 0.25;
constexpr double kClampMin = 0.2, kClampMax = 0.90, kZMin = -2.0, kZMax = 15.0;
constexpr double kPcdVoxel = 0.1;       // Aufloesung der gespeicherten map.pcd [m]

struct Keyframe {
    double stamp;
    Eigen::Isometry3d odom;  // camera_init <- body (FAST-LIO roh)
    Cloud::Ptr cloud;        // body-Frame
};

Cloud::Ptr downsample(const Cloud::Ptr& in, double leaf) {
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud(in);
    vg.setLeafSize(leaf, leaf, leaf);
    Cloud::Ptr out(new Cloud);
    vg.filter(*out);
    return out;
}

struct MapEdit {
    Eigen::Isometry3d to_new = Eigen::Isometry3d::Identity();  // bisheriger Frame -> neuer Frame
    std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> crop;  // (min, max) im neuen Frame
};

// map.yaml lesen; fehlt sie, kommentierte Vorlage schreiben (Identitaet, keine Ausschnitte)
MapEdit loadMapEdit(const std::string& dir) {
    const std::string path = dir + "/map.yaml";
    if (!std::filesystem::exists(path)) {
        char date[16];
        const std::time_t now = std::time(nullptr);
        std::strftime(date, sizeof(date), "%Y-%m-%d", std::localtime(&now));
        std::ofstream(path)
            << "name: " << std::filesystem::path(dir).filename().string() << "\n"
            << "created: " << date << "\n"
            << "# Neuer Nullpunkt + Blickrichtung (x-Achse), angegeben im BISHERIGEN Karten-Frame.\n"
            << "# Danach map_optimizer erneut laufen lassen; initial_x/y/yaw beim Fliegen\n"
            << "# beziehen sich dann auf den neuen Nullpunkt.\n"
            << "origin: {x: 0.0, y: 0.0, z: 0.0, yaw_deg: 0.0}\n"
            << "# Ausgeschnittene Bereiche im NEUEN Frame (achsparallel), z.B.:\n"
            << "#   - {min: [2.0, -1.0, -1.0], max: [4.0, 1.0, 3.0]}\n"
            << "crop: []\n";
    }
    MapEdit e;
    const YAML::Node y = YAML::LoadFile(path);
    if (const auto o = y["origin"]) {
        const Eigen::Isometry3d origin =
            Eigen::Translation3d(o["x"].as<double>(0), o["y"].as<double>(0), o["z"].as<double>(0)) *
            Eigen::AngleAxisd(o["yaw_deg"].as<double>(0) * M_PI / 180.0, Eigen::Vector3d::UnitZ());
        e.to_new = origin.inverse();
    }
    for (const auto& b : y["crop"]) {
        const auto lo = b["min"].as<std::vector<double>>(), hi = b["max"].as<std::vector<double>>();
        e.crop.emplace_back(Eigen::Vector3d(lo[0], lo[1], lo[2]), Eigen::Vector3d(hi[0], hi[1], hi[2]));
    }
    return e;
}

gtsam::Pose3 toGtsam(const Eigen::Isometry3d& T) { return gtsam::Pose3(T.matrix()); }
Eigen::Isometry3d toEigen(const gtsam::Pose3& P) { return Eigen::Isometry3d(P.matrix()); }
}  // namespace

int main(int argc, char** argv) {
    if (argc < 2) {
        std::fprintf(stderr, "Aufruf: map_optimizer <map_dir>\n");
        return 1;
    }
    const std::string dir = argv[1];
    const std::string kf_dir = dir + "/keyframes";

    double mount_pitch = 0.0;
    std::ifstream(kf_dir + "/mount_pitch") >> mount_pitch;

    // --- Keyframes laden ---
    std::vector<Keyframe> kfs;
    std::ifstream csv(kf_dir + "/poses.csv");
    std::string line;
    std::getline(csv, line);  // Header
    while (std::getline(csv, line)) {
        std::stringstream ss(line);
        std::string tok;
        std::vector<double> v;
        while (std::getline(ss, tok, ',')) v.push_back(std::stod(tok));
        if (v.size() != 9) continue;
        Keyframe k;
        k.stamp = v[1];
        k.odom = Eigen::Translation3d(v[2], v[3], v[4]) * Eigen::Quaterniond(v[8], v[5], v[6], v[7]);
        k.cloud.reset(new Cloud);
        char name[32];
        std::snprintf(name, sizeof(name), "/kf_%05d.pcd", static_cast<int>(v[0]));
        if (pcl::io::loadPCDFile(kf_dir + name, *k.cloud) != 0) continue;
        kfs.push_back(k);
    }
    if (kfs.size() < 2) {
        std::fprintf(stderr, "Zu wenige Keyframes in %s\n", kf_dir.c_str());
        return 1;
    }
    std::printf("%zu Keyframes geladen, mount_pitch %.3f rad\n", kfs.size(), mount_pitch);

    // --- Graph: Prior + Odometrie ---
    using gtsam::symbol_shorthand::X;
    gtsam::NonlinearFactorGraph graph;
    gtsam::Values initial;
    // Tangentenraum-Reihenfolge Pose3: (rx, ry, rz, tx, ty, tz)
    const auto prior_noise = gtsam::noiseModel::Diagonal::Sigmas(
        (gtsam::Vector6() << 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6).finished());
    // Pro Keyframe-Schritt (~1 m / 20°). Rotation grosszuegig: FAST-LIO verliert
    // in Kurven ~1°/Keyframe — zu steif, und der Graph ignoriert jeden Loop
    const auto odom_noise = gtsam::noiseModel::Diagonal::Sigmas(
        (gtsam::Vector6() << 0.02, 0.02, 0.02, 0.05, 0.05, 0.05).finished());
    // Huber statt Cauchy: ein echter Loop mit 20-30° Korrektur ist kein Ausreisser,
    // Cauchy wuerde ihn auf ~0 gewichten; Huber zieht linear weiter
    const auto loop_noise = gtsam::noiseModel::Robust::Create(
        gtsam::noiseModel::mEstimator::Huber::Create(1.345),
        gtsam::noiseModel::Diagonal::Sigmas(
            (gtsam::Vector6() << 0.01, 0.01, 0.01, 0.05, 0.05, 0.05).finished()));

    graph.add(gtsam::PriorFactor<gtsam::Pose3>(X(0), toGtsam(kfs[0].odom), prior_noise));
    for (size_t i = 0; i < kfs.size(); ++i) {
        initial.insert(X(i), toGtsam(kfs[i].odom));
        if (i > 0)
            graph.add(gtsam::BetweenFactor<gtsam::Pose3>(
                X(i - 1), X(i), toGtsam(kfs[i - 1].odom.inverse() * kfs[i].odom), odom_noise));
    }

    // --- Loop-Closures ---
    std::vector<Cloud::Ptr> small(kfs.size());
    for (size_t i = 0; i < kfs.size(); ++i) small[i] = downsample(kfs[i].cloud, kGicpVoxel);

    pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp;
    gicp.setMaxCorrespondenceDistance(kGicpMaxCorr);
    gicp.setMaximumIterations(50);

    int loops = 0, tried = 0;
    int last_loop = -1000;
    for (size_t j = 0; j < kfs.size(); ++j) {
        if (static_cast<int>(j) - last_loop < kLoopSkip) continue;
        int best = -1;
        double best_d = kLoopRadius;
        for (size_t i = 0; i < j; ++i) {
            if (kfs[j].stamp - kfs[i].stamp < kLoopMinGap) break;  // i aufsteigend in der Zeit
            const double d = (kfs[i].odom.translation() - kfs[j].odom.translation()).norm();
            if (d < best_d) { best_d = d; best = static_cast<int>(i); }
        }
        if (best < 0) continue;
        ++tried;

        // Submap um best im body-Frame von best (Odometrie ist lokal genau)
        Cloud::Ptr target(new Cloud);
        const auto inv_best = kfs[best].odom.inverse();
        for (int k = std::max(0, best - kSubmapHalf);
             k <= std::min(static_cast<int>(kfs.size()) - 1, best + kSubmapHalf); ++k) {
            Cloud tmp;
            pcl::transformPointCloud(*small[k], tmp, (inv_best * kfs[k].odom).matrix().cast<float>());
            *target += tmp;
        }
        const Eigen::Isometry3d guess = inv_best * kfs[j].odom;
        gicp.setInputTarget(downsample(target, kGicpVoxel));
        gicp.setInputSource(small[j]);
        // Hochachse im body-Frame von best: camera_init ist um mount_pitch gekippt
        // (map = Ry(pitch) * camera_init, map-z = Schwerkraft)
        const Eigen::Vector3d up = kfs[best].odom.rotation().transpose() *
            (Eigen::AngleAxisd(mount_pitch, Eigen::Vector3d::UnitY()).inverse() * Eigen::Vector3d::UnitZ());
        double fitness = 1e9;
        Eigen::Isometry3d rel = guess;
        for (int h = -kYawHypoN; h <= kYawHypoN; ++h) {
            // um den Ursprung von best drehen (Translation dreht mit)
            const Eigen::Isometry3d g = Eigen::Isometry3d(Eigen::AngleAxisd(h * kYawHypoStep, up)) * guess;
            Cloud aligned;
            gicp.align(aligned, g.matrix().cast<float>());
            if (!gicp.hasConverged()) continue;
            const double f = gicp.getFitnessScore(kGicpMaxCorr);
            if (f < fitness) {
                fitness = f;
                rel = Eigen::Isometry3d(gicp.getFinalTransformation().cast<double>());
            }
        }
        const Eigen::Isometry3d corr = guess.inverse() * rel;
        const double dt = corr.translation().norm();
        const double dr = Eigen::AngleAxisd(corr.rotation()).angle();
        if (fitness > kFitnessMax || dt > kMaxCorrTrans || dr > kMaxCorrRot) continue;

        graph.add(gtsam::BetweenFactor<gtsam::Pose3>(X(best), X(j), toGtsam(rel), loop_noise));
        ++loops;
        last_loop = static_cast<int>(j);
        std::printf("  Loop %3d <- %3d  (%.1f m, %.0f s)  Korrektur %.3f m / %.2f°  Fitness %.4f\n",
                    best, static_cast<int>(j), best_d, kfs[j].stamp - kfs[best].stamp,
                    dt, dr * 180.0 / M_PI, fitness);
    }
    std::printf("%d Loop-Closures (von %d Kandidaten)\n", loops, tried);

    // --- Optimieren ---
    const gtsam::Values result = gtsam::LevenbergMarquardtOptimizer(graph, initial).optimize();
    std::printf("Graph-Fehler: %.3f -> %.3f\n", graph.error(initial), graph.error(result));

    // --- Ausgabe im gelevelten map-Frame (+ Bearbeitung aus map.yaml) ---
    const MapEdit edit = loadMapEdit(dir);
    const Eigen::Isometry3d level =
        edit.to_new * Eigen::Isometry3d(Eigen::AngleAxisd(mount_pitch, Eigen::Vector3d::UnitY()));
    auto cropped = [&](const pcl::PointXYZ& p) {
        for (const auto& [lo, hi] : edit.crop)
            if (p.x >= lo.x() && p.x <= hi.x() && p.y >= lo.y() && p.y <= hi.y() && p.z >= lo.z() && p.z <= hi.z())
                return true;
        return false;
    };
    if (!edit.to_new.isApprox(Eigen::Isometry3d::Identity()) || !edit.crop.empty())
        std::printf("map.yaml: Nullpunkt verschoben/gedreht: %s, %zu Ausschnitt(e)\n",
                    edit.to_new.isApprox(Eigen::Isometry3d::Identity()) ? "nein" : "ja", edit.crop.size());
    Cloud::Ptr map(new Cloud);
    octomap::OcTree tree(kMapRes);
    tree.setProbHit(kProbHit);
    tree.setProbMiss(kProbMiss);
    tree.setClampingThresMin(kClampMin);
    tree.setClampingThresMax(kClampMax);
    std::ofstream out_poses(dir + "/poses_opt.csv");
    out_poses << "id,stamp,x,y,z,qx,qy,qz,qw,shift_m\n";
    double max_shift = 0.0;
    for (size_t i = 0; i < kfs.size(); ++i) {
        const Eigen::Isometry3d T = level * toEigen(result.at<gtsam::Pose3>(X(i)));
        const double shift = (T.translation() - (level * kfs[i].odom).translation()).norm();
        max_shift = std::max(max_shift, shift);
        Cloud in_map;
        pcl::transformPointCloud(*kfs[i].cloud, in_map, T.matrix().cast<float>());
        if (!edit.crop.empty())
            in_map.erase(std::remove_if(in_map.begin(), in_map.end(), cropped), in_map.end());
        *map += in_map;

        octomap::Pointcloud oc;
        for (const auto& p : in_map.points)
            if (p.z >= kZMin && p.z <= kZMax) oc.push_back(p.x, p.y, p.z);
        const auto& o = T.translation();
        tree.insertPointCloud(oc, octomap::point3d(o.x(), o.y(), o.z()), kMaxRange, false, true);

        const Eigen::Quaterniond q(T.rotation());
        out_poses << i << ',' << std::fixed << kfs[i].stamp << ',' << o.x() << ',' << o.y() << ','
                  << o.z() << ',' << q.x() << ',' << q.y() << ',' << q.z() << ',' << q.w() << ','
                  << shift << '\n';
    }
    map = downsample(map, kPcdVoxel);
    pcl::io::savePCDFileBinary(dir + "/map.pcd", *map);
    tree.updateInnerOccupancy();
    tree.writeBinary(dir + "/map.bt");
    std::printf("Max. Pose-Verschiebung durch Optimierung: %.3f m\n", max_shift);
    std::printf("Geschrieben: %s/map.pcd (%zu Punkte), %s/map.bt, %s/poses_opt.csv\n",
                dir.c_str(), map->size(), dir.c_str(), dir.c_str());
    return 0;
}
