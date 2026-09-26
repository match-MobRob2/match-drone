// Selbsttest TF-Glaettung der Relokalisierung (von Hand, nicht CI):
//   g++ -O2 -I/usr/include/eigen3 quat_smoothing_check.cpp -o /tmp/qsc && /tmp/qsc
// Nachgebaut wie broadcast() in lidar_relocalization_node.cpp: Ziel = GICP-Ergebnis
// als float-Matrix (Rotation nicht exakt orthonormal). Ohne Normieren schaukelte sich
// |q| auf und die TF sprang saegezahnfoermig um 20-40° Yaw (Sim, 2026-09-24).
#include <Eigen/Geometry>
#include <cassert>
#include <cmath>
#include <cstdio>

int main() {
    // Soll map->camera_init: Rz(-90.7°) * Ry(0.52), leicht verrauscht und in float
    Eigen::Matrix4f f = (Eigen::Translation3d(3.27, 0.56, 0.0)
        * Eigen::AngleAxisd(-90.7 * M_PI / 180, Eigen::Vector3d::UnitZ())
        * Eigen::AngleAxisd(0.52, Eigen::Vector3d::UnitY())).matrix().cast<float>();
    f.block<3, 3>(0, 0) *= 1.004f;  // typische Nicht-Orthonormalitaet eines GICP-Ergebnisses
    const Eigen::Matrix4d d = f.cast<double>();
    const Eigen::Isometry3d raw(d);  // alte Uebernahme
    const Eigen::Isometry3d target = Eigen::Translation3d(Eigen::Vector3d(d.block<3, 1>(0, 3)))
        * Eigen::Quaterniond(Eigen::Matrix3d(d.block<3, 3>(0, 0))).normalized();  // neu: rigid()

    Eigen::Isometry3d b_old = Eigen::Isometry3d::Identity(), b_new = Eigen::Isometry3d::Identity();
    double max_norm_old = 0;
    for (int i = 0; i < 500; ++i) {
        const Eigen::Quaterniond qo = Eigen::Quaterniond(b_old.rotation()).slerp(0.1, Eigen::Quaterniond(raw.rotation()));
        b_old = Eigen::Translation3d(b_old.translation()) * qo;
        max_norm_old = std::max(max_norm_old, Eigen::Quaterniond(b_old.linear()).norm());
        const Eigen::Quaterniond qn = Eigen::Quaterniond(b_new.linear()).normalized()
            .slerp(0.1, Eigen::Quaterniond(target.linear()).normalized()).normalized();
        b_new = Eigen::Translation3d(b_new.translation()) * qn;
        assert(std::abs(Eigen::Quaterniond(b_new.linear()).norm() - 1.0) < 1e-9);
    }
    const double err = Eigen::AngleAxisd(b_new.linear() * target.linear().transpose()).angle() * 180 / M_PI;
    std::printf("alt: max |q| %.4f   neu: |q| = 1, Restfehler %.2e°\n", max_norm_old, err);
    assert(err < 1e-6);
    std::puts("OK");
}
