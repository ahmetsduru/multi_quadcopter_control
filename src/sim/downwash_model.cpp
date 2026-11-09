#include "multi_quadcopter_control/sim/downwash_model.h"

#include <cmath>
#include <stdexcept>

namespace mqc_sim {

PinkNoiseGenerator::PinkNoiseGenerator(long long seed) : counter_(0) {
    if (seed < 0) {
        std::random_device rd;
        gen_ = std::mt19937(rd());
    } else {
        gen_ = std::mt19937(static_cast<std::mt19937::result_type>(seed));
    }
    dist_ = std::uniform_real_distribution<double>(-1.0, 1.0);
    for (int i = 0; i < NUM_ROWS; ++i) rows_[i] = dist_(gen_);
}

double PinkNoiseGenerator::next() {
    int last_counter = counter_;
    counter_++;

    int number_of_changes = counter_ ^ last_counter;
    int row_to_change = 0;
    while (number_of_changes >>= 1) row_to_change++;

    if (row_to_change < NUM_ROWS) {
        rows_[row_to_change] = dist_(gen_);
    }

    double sum = 0.0;
    for (int i = 0; i < NUM_ROWS; ++i) sum += rows_[i];

    // 16 x U(-1,1) / 5 -> std ~0.46, extremes +/-3.2
    return sum / 5.0;
}

PinkNoise3D::PinkNoise3D(long long base_seed)
    : px(base_seed < 0 ? -1 : base_seed * 3 + 0),
      py(base_seed < 0 ? -1 : base_seed * 3 + 1),
      pz(base_seed < 0 ? -1 : base_seed * 3 + 2) {}

// Same seed derivation as relative_disturbance_generator.cpp (depends only on the drone name)
PinkNoise3D DownwashModel::makeNoiseFor(const std::string& drone_name) const {
    if (p_.noise_seed < 0) return PinkNoise3D(-1);
    long long id = 0;
    for (char c : drone_name) id = id * 131 + static_cast<unsigned char>(c);
    id &= 0x3FFFFFFF;
    return PinkNoise3D((p_.noise_seed * 1000003LL + id) & 0x3FFFFFFFFFFFLL);
}

void DownwashModel::addDrone(const std::string& name, double mass) {
    const double g = 9.81;
    const double thrust_per_rotor = mass * g / 4.0;
    const double k_z = thrust_per_rotor * p_.c_coup;
    const double k_x = k_z * p_.r_lat;
    const double k_y = k_z * p_.r_lat;

    Eigen::Matrix3d k;
    k << k_x, 0.0, 0.0,
         0.0, k_y, 0.0,
         0.0, 0.0, k_z;

    names_.push_back(name);
    k_matrices_.push_back(k);
    noise_.push_back(makeNoiseFor(name));
}

std::vector<Eigen::Vector3d> DownwashModel::compute(const std::vector<DroneKinematics>& drones,
                                                    std::vector<Eigen::Vector3d>* static_force) {
    if (drones.size() != names_.size()) {
        throw std::runtime_error("DownwashModel::compute: drone count mismatch");
    }
    const size_t N = drones.size();
    std::vector<Eigen::Vector3d> force(N, Eigen::Vector3d::Zero());
    if (static_force) static_force->assign(N, Eigen::Vector3d::Zero());

    // One noise sample per drone per call
    std::vector<Eigen::Vector3d> noise_now(N);
    for (size_t i = 0; i < N; ++i) noise_now[i] = noise_[i].getVector();

    const double two_sigma_sq = 2.0 * p_.sigma * p_.sigma;
    const double z0_sq = p_.z0 * p_.z0;

    for (size_t top = 0; top < N; ++top) {
        const Eigen::Vector3d& P_top = drones[top].position_world;
        const Eigen::Matrix3d& R_top_to_world = drones[top].R_body_to_world;
        const Eigen::Matrix3d R_world_to_top = R_top_to_world.transpose();
        const Eigen::Matrix3d& k_matrix = k_matrices_[top];

        for (size_t bot = 0; bot < N; ++bot) {
            if (top == bot) continue;

            const Eigen::Vector3d P_rel_body = R_world_to_top * (drones[bot].position_world - P_top);

            if (P_rel_body.z() >= 0) continue;  // must be below the source drone
            const double d_z = -P_rel_body.z();
            if (p_.max_vertical_range > 0.0 && d_z > p_.max_vertical_range) continue;

            const double dist_xy_sq = P_rel_body.x() * P_rel_body.x() + P_rel_body.y() * P_rel_body.y();
            const double exp_power_term = dist_xy_sq / two_sigma_sq;
            if (exp_power_term >= 5.0) continue;

            const Eigen::Vector3d n_vec_body = P_rel_body.normalized();
            const double decay_factor = (1.0 / (d_z * d_z + z0_sq)) * std::exp(-exp_power_term);

            const Eigen::Vector3d f_static_body = k_matrix * n_vec_body * decay_factor;
            const Eigen::Vector3d f_turb_body =
                f_static_body.cwiseAbs().cwiseProduct(noise_now[bot] * p_.turb_intensity);
            const Eigen::Vector3d f_total_body = (f_static_body + f_turb_body) * 4.0;

            force[bot] += R_top_to_world * f_total_body;
            if (static_force) (*static_force)[bot] += R_top_to_world * (f_static_body * 4.0);
        }
    }
    return force;
}

}  // namespace mqc_sim
