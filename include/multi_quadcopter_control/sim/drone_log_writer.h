// Per-drone log file writer, same column layout as src/data_logger.cpp
#ifndef MQC_SIM_DRONE_LOG_WRITER_H
#define MQC_SIM_DRONE_LOG_WRITER_H

#include <Eigen/Dense>
#include <fstream>
#include <string>

namespace mqc_sim {

struct LogRow {
    double time = 0.0;
    Eigen::Vector3d a_pos, a_vel, a_acc, a_ang_vel, a_eul_ang;
    Eigen::Vector3d r_pos, r_vel, r_acc, r_eul_ang, r_torq, dist_f;
    double r_thr = 0.0;
    double r_psi = 0.0;
};

class DroneLogWriter {
public:
    explicit DroneLogWriter(const std::string& path);
    bool isOpen() const { return file_.is_open(); }
    void write(const LogRow& row);
    void close();

private:
    std::ofstream file_;
};

}  // namespace mqc_sim

#endif  // MQC_SIM_DRONE_LOG_WRITER_H
