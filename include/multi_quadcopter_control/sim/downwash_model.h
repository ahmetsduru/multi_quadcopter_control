// Relative downwash disturbance model (ROS-free).
// Same model as src/relative_disturbance_generator.cpp:
//   F = 4 * R_top * [ K n / (dz^2 + z0^2) * exp(-r^2 / 2 sigma^2) * (1 + turb_intensity * pink_noise) ]
#ifndef MQC_SIM_DOWNWASH_MODEL_H
#define MQC_SIM_DOWNWASH_MODEL_H

#include <Eigen/Dense>
#include <array>
#include <random>
#include <string>
#include <vector>

namespace mqc_sim {

// Voss-McCartney pink noise generator
class PinkNoiseGenerator {
public:
    // seed < 0 -> random seed (std::random_device), seed >= 0 -> reproducible sequence
    explicit PinkNoiseGenerator(long long seed = -1);
    double next();

private:
    static const int NUM_ROWS = 16;
    std::array<double, NUM_ROWS> rows_;
    int counter_ = 0;
    std::mt19937 gen_;
    std::uniform_real_distribution<double> dist_;
};

struct PinkNoise3D {
    PinkNoiseGenerator px, py, pz;
    explicit PinkNoise3D(long long base_seed = -1);
    Eigen::Vector3d getVector() { return Eigen::Vector3d(px.next(), py.next(), pz.next()); }
};

struct DownwashParams {
    double c_coup = 0.09;
    double r_lat = 0.05;
    double z0 = 0.22360679774997896;  // sqrt(0.05) -> z0^2 = 0.05
    double sigma = 0.15;
    double turb_intensity = 0.70;
    double max_vertical_range = -1.0;  // <= 0: unlimited
    long long noise_seed = 42;         // < 0: random
};

struct DroneKinematics {
    Eigen::Vector3d position_world;  // includes the initial offset
    Eigen::Matrix3d R_body_to_world;
};

class DownwashModel {
public:
    explicit DownwashModel(const DownwashParams& p) : p_(p) {}

    // Register drones in a fixed order (index used in compute()).
    void addDrone(const std::string& name, double mass);

    // Disturbance force (world frame) on every drone. Advances each drone's noise exactly once.
    // If static_force is given it receives the deterministic downwash part (without turbulence),
    // i.e. the modelled disturbance used as controller feedforward.
    std::vector<Eigen::Vector3d> compute(const std::vector<DroneKinematics>& drones,
                                         std::vector<Eigen::Vector3d>* static_force = nullptr);

private:
    DownwashParams p_;
    std::vector<std::string> names_;
    std::vector<Eigen::Matrix3d> k_matrices_;
    std::vector<PinkNoise3D> noise_;
    PinkNoise3D makeNoiseFor(const std::string& drone_name) const;
};

}  // namespace mqc_sim

#endif  // MQC_SIM_DOWNWASH_MODEL_H
