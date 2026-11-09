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
    Eigen::Vector3d ndob_f = Eigen::Vector3d::Zero();  // NDOB estimate (written only in NDOB mode)
    double r_thr = 0.0;
    double r_psi = 0.0;
};

class DroneLogWriter {
public:
    // ndob_columns: add ndob_f.x/y/z after dist_f (same layout as the NDOB-era data_logger)
    explicit DroneLogWriter(const std::string& path, bool ndob_columns = false);
    bool isOpen() const { return file_.is_open(); }
    void write(const LogRow& row);
    void close();

private:
    std::ofstream file_;
    bool ndob_columns_ = false;
};

}  // namespace mqc_sim

#endif  // MQC_SIM_DRONE_LOG_WRITER_H
