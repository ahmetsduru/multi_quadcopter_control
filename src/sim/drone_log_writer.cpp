#include "multi_quadcopter_control/sim/drone_log_writer.h"

#include <iomanip>

namespace mqc_sim {

DroneLogWriter::DroneLogWriter(const std::string& path) {
    file_.open(path, std::ios::out);
    if (!file_.is_open()) return;

    file_ << "time";
    for (const char* name : {"a_pos", "a_vel", "a_acc", "a_ang_vel", "a_eul_ang",
                             "r_pos", "r_vel", "r_acc", "r_eul_ang", "r_torq", "dist_f"}) {
        file_ << ", " << name << ".x"
              << ", " << name << ".y"
              << ", " << name << ".z";
    }
    file_ << ", r_thr, r_psi\n";
}

void DroneLogWriter::write(const LogRow& r) {
    if (!file_.is_open()) return;

    file_ << std::fixed << std::setprecision(9) << r.time;
    file_.unsetf(std::ios::floatfield);

    auto vec = [&](const Eigen::Vector3d& v) {
        file_ << ", " << v.x() << ", " << v.y() << ", " << v.z();
    };
    vec(r.a_pos);
    vec(r.a_vel);
    vec(r.a_acc);
    vec(r.a_ang_vel);
    vec(r.a_eul_ang);
    vec(r.r_pos);
    vec(r.r_vel);
    vec(r.r_acc);
    vec(r.r_eul_ang);
    vec(r.r_torq);
    vec(r.dist_f);
    file_ << ", " << r.r_thr << ", " << r.r_psi << "\n";
}

void DroneLogWriter::close() {
    if (file_.is_open()) file_.close();
}

}  // namespace mqc_sim
