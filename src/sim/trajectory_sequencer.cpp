#include "multi_quadcopter_control/sim/trajectory_sequencer.h"

#include <cmath>
#include <iostream>

namespace mqc_sim {

TrajectorySequencer::TrajectorySequencer(const std::vector<TrajectorySpec>& specs, bool is_psi_active)
    : specs_(specs), m_is_psi_active(is_psi_active) {
    // Constructor of TrajectoryGenerator: request the first trajectory
    have_current_ = fetchNext();
    if (!have_current_) {
        finished_ = true;
    }
}

// WaypointServer::getTrajectoryCallback + TrajectoryGenerator::getTrajectoryFromServer
bool TrajectorySequencer::fetchNext() {
    if (next_index_ >= specs_.size()) {
        m_no_more_trajectory = true;
        return false;
    }
    const TrajectorySpec& s = specs_[next_index_];
    if (s.points_x.empty() || s.points_y.empty() || s.points_z.empty() || s.times.empty()) {
        m_no_more_trajectory = true;
        return false;
    }
    current_ = s;
    next_index_++;
    return true;
}

bool TrajectorySequencer::plan() {
    const std::string& method = current_.method;
    double extra_time;
    if (method == "natural_cubic_spline") {
        coeffs_per_seg_ = 4; switch_segment_ = 5; extra_time = 4.0; updates_psi_ = false;
    } else if (method == "cubic_spline") {
        coeffs_per_seg_ = 4; switch_segment_ = 3; extra_time = 7.0; updates_psi_ = true;
    } else if (method == "minimum_jerk") {
        coeffs_per_seg_ = 6; switch_segment_ = 5; extra_time = 7.0; updates_psi_ = true;
    } else if (method == "minimum_snap") {
        coeffs_per_seg_ = 8; switch_segment_ = 5; extra_time = 7.0; updates_psi_ = true;
    } else {
        std::cerr << "[TrajectorySequencer] Unknown trajectory method '" << method
                  << "' in " << current_.name << ", skipping." << std::endl;
        return false;
    }

    // Start from the current reference when this is not the first trajectory
    std::vector<double> points_x_mod = current_.points_x;
    std::vector<double> points_y_mod = current_.points_y;
    std::vector<double> points_z_mod = current_.points_z;
    std::vector<double> times_mod = current_.times;
    if (!m_initial_trajectory) {
        points_x_mod.insert(points_x_mod.begin(), m_position.x);
        points_y_mod.insert(points_y_mod.begin(), m_position.y);
        points_z_mod.insert(points_z_mod.begin(), m_position.z);
        double last_time = times_mod.back() + extra_time;
        times_mod.push_back(last_time);
    }

    if (method == "natural_cubic_spline") {
        buildNaturalCubicSpline(points_x_mod, points_y_mod, points_z_mod, times_mod);
    } else if (method == "cubic_spline") {
        buildCubicSpline(points_x_mod, points_y_mod, points_z_mod, times_mod);
    } else if (method == "minimum_jerk") {
        buildMinimumJerk(points_x_mod, points_y_mod, points_z_mod, times_mod);
    } else {
        buildMinimumSnap(points_x_mod, points_y_mod, points_z_mod, times_mod);
    }

    n_ = static_cast<int>(points_x_mod.size()) - 1;
    times_mod_ = times_mod;
    t_ = times_mod_[0];
    return true;
}

// Evaluates the piecewise polynomial at local time t, updates m_position/velocity/acceleration/jerk
// and returns the segment index.
int TrajectorySequencer::evaluate(double t) {
    int i = 0;
    while (i < n_ && t > times_mod_[i + 1]) {
        i++;
    }
    const double dt = t - times_mod_[i];
    const int base = coeffs_per_seg_ * i;

    auto derivs = [&](const Eigen::VectorXd& c, double out[4]) {
        out[0] = out[1] = out[2] = out[3] = 0.0;
        for (int k = 0; k < coeffs_per_seg_; ++k) {
            const double a = c(base + k);
            out[0] += a * std::pow(dt, k);
            if (k >= 1) out[1] += k * a * std::pow(dt, k - 1);
            if (k >= 2) out[2] += k * (k - 1) * a * std::pow(dt, k - 2);
            if (k >= 3) out[3] += k * (k - 1) * (k - 2) * a * std::pow(dt, k - 3);
        }
    };
    double dx[4], dy[4], dz[4];
    derivs(cx_, dx);
    derivs(cy_, dy);
    derivs(cz_, dz);

    m_position = {dx[0], dy[0], dz[0]};
    m_velocity = {dx[1], dy[1], dz[1]};
    m_acceleration = {dx[2], dy[2], dz[2]};
    m_jerk = {dx[3], dy[3], dz[3]};
    return i;
}

void TrajectorySequencer::tick() {
    // The loop handles trajectory switches that happen within the same tick
    // (in the original node the new trajectory is published without sleeping).
    for (int guard = 0; guard < 1000 && !finished_; ++guard) {
        if (!active_) {
            if (!have_current_) {
                finished_ = true;
                return;
            }
            if (!plan()) {
                // Unknown method: generateTrajectory() simply requests the next one
                have_current_ = !m_no_more_trajectory && fetchNext();
                continue;
            }
            active_ = true;
        }

        // for (t = times_mod[0]; t <= times_mod[n]; t += 0.05)
        if (!(t_ <= times_mod_[n_])) {
            active_ = false;
            have_current_ = !m_no_more_trajectory && fetchNext();
            if (!have_current_) {
                finished_ = true;  // references keep their last values
                return;
            }
            continue;
        }

        const int i = evaluate(t_);

        // Request the next waypoint set when the switch segment is reached
        if (i == switch_segment_ && fetchNext()) {
            active_ = false;
            have_current_ = true;
            continue;  // re-plan from the current reference and publish its first sample now
        }

        if (updates_psi_) generatePsi();  // psi is held during natural cubic splines (as in the original)
        t_ += kSampleStep;
        return;
    }
}

// Same as TrajectoryGenerator::generatePsi (m_last_psi is initialised to 0 here)
void TrajectorySequencer::generatePsi() {
    const double alpha = 0.05;

    if (m_is_psi_active) {
        if (std::isfinite(m_velocity.x) && std::isfinite(m_velocity.y)) {
            if (!(m_velocity.x == 0.0 && m_velocity.y == 0.0)) {
                double new_psi = std::atan2(m_velocity.y, m_velocity.x);
                m_des_psi = alpha * new_psi + (1.0 - alpha) * m_last_psi;
                m_last_psi = m_des_psi;
            } else {
                m_des_psi = m_last_psi;
            }
        } else {
            m_des_psi = m_last_psi;
        }
    } else {
        m_des_psi = 0.0;
    }
}

// Matrix set-up copied verbatim from TrajectoryGenerator::solveNaturalCubicSpline()
void TrajectorySequencer::buildNaturalCubicSpline(const std::vector<double>& points_x_mod, const std::vector<double>& points_y_mod,
                                 const std::vector<double>& points_z_mod, const std::vector<double>& times_mod) {
    int n = points_x_mod.size() - 1; // Segment sayısı, nokta sayısından bir eksik
    int matrix_size = 4 * n;     // 4n boyutunda matris (a_i, b_i, c_i, d_i)
    
    // A matrisi (4n x 4n) ve b vektörü (4n) her eksen için
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(matrix_size, matrix_size);
    Eigen::VectorXd b_x = Eigen::VectorXd::Zero(matrix_size);
    Eigen::VectorXd b_y = Eigen::VectorXd::Zero(matrix_size);
    Eigen::VectorXd b_z = Eigen::VectorXd::Zero(matrix_size);

    // 1. Pozisyon denklemleri (Position Continuity)
    for (int i = 0; i < n; ++i) {

        // S_i(t_i) = P_i (her eksen için)
        A(2 * i, 4 * i + 0) = 1;
        A(2 * i, 4 * i + 1) = 0;
        A(2 * i, 4 * i + 2) = 0;
        A(2 * i, 4 * i + 3) = 0;
        b_x(2 * i) = points_x_mod[i];
        b_y(2 * i) = points_y_mod[i];
        b_z(2 * i) = points_z_mod[i];

        // S_i(t_{i+1}) = P_{i+1}
        double dt = times_mod[i + 1] - times_mod[i];
        A(2 * i + 1, 4 * i + 0) = 1;
        A(2 * i + 1, 4 * i + 1) = dt;
        A(2 * i + 1, 4 * i + 2) = std::pow(dt, 2);
        A(2 * i + 1, 4 * i + 3) = std::pow(dt, 3);
        b_x(2 * i + 1) = points_x_mod[i + 1];
        b_y(2 * i + 1) = points_y_mod[i + 1];
        b_z(2 * i + 1) = points_z_mod[i + 1];
    }
    // 2. Hız sürekliliği (Velocity Continuity)
    for (int i = 1; i < n; ++i) {
        double dt_prev = times_mod[i] - times_mod[i - 1];
        A(2 * n + (i - 1), 4 * (i - 1) + 1) = 1;
        A(2 * n + (i - 1), 4 * (i - 1) + 2) = 2 * dt_prev;
        A(2 * n + (i - 1), 4 * (i - 1) + 3) = 3 * std::pow(dt_prev, 2);
        A(2 * n + (i - 1), 4 * i + 1) = -1;
        b_x(2 * n + (i - 1)) = 0;
        b_y(2 * n + (i - 1)) = 0;
        b_z(2 * n + (i - 1)) = 0;
    }
    // 3. İvme sürekliliği (Acceleration Continuity)
    for (int i = 1; i < n; ++i) {
        double dt_prev = times_mod[i] - times_mod[i - 1];
        A(3 * n + (i - 2), 4 * (i - 1) + 2) = 2;
        A(3 * n - 1 + (i - 1), 4 * (i - 1) + 3) = 6 * dt_prev;
        A(3 * n - 1 + (i - 1), 4 * i + 2) = -2;
        b_x(3 * n + (i - 2)) = 0;
        b_y(3 * n + (i - 2)) = 0;
        b_z(3 * n + (i - 2)) = 0;
    }
    
    // 4. initial acceleration and final acceleration conditions
    A(4 * n - 2, 2) = 2;

    if (m_initial_trajectory)
    {
        b_x(4 * n - 2) = 0;
        b_y(4 * n - 2) = 0;
        b_z(4 * n - 2) = 0;

        m_initial_trajectory = false;
    } else
    {
        b_x(4 * n - 2) = m_acceleration.x;
        b_y(4 * n - 2) = m_acceleration.y;
        b_z(4 * n - 2) = m_acceleration.z;
    }
    
    double dt_last = times_mod[n] - times_mod[n - 1];
    A(4 * n - 1, 4 * (n - 1) + 2) = 2;
    A(4 * n - 1, 4 * (n - 1) + 3) = 6 * dt_last;
    b_x(4 * n - 1) = 0;
    b_y(4 * n - 1) = 0;
    b_z(4 * n - 1) = 0;

    // Denklemi çöz (Ax = b) her eksen için
    Eigen::VectorXd coeffs_x = A.colPivHouseholderQr().solve(b_x);
    Eigen::VectorXd coeffs_y = A.colPivHouseholderQr().solve(b_y);
    Eigen::VectorXd coeffs_z = A.colPivHouseholderQr().solve(b_z);

    cx_ = coeffs_x;
    cy_ = coeffs_y;
    cz_ = coeffs_z;
}

// Matrix set-up copied verbatim from TrajectoryGenerator::solveCubicSpline()
void TrajectorySequencer::buildCubicSpline(const std::vector<double>& points_x_mod, const std::vector<double>& points_y_mod,
                                 const std::vector<double>& points_z_mod, const std::vector<double>& times_mod) {
    int n = points_x_mod.size() - 1; // Segment sayısı, nokta sayısından bir eksik
    int matrix_size = 4 * n;     // 4n boyutunda matris (a_i, b_i, c_i, d_i)
    
    // A matrisi (4n x 4n) ve b vektörü (4n) her eksen için
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(matrix_size, matrix_size);
    Eigen::VectorXd b_x = Eigen::VectorXd::Zero(matrix_size);
    Eigen::VectorXd b_y = Eigen::VectorXd::Zero(matrix_size);
    Eigen::VectorXd b_z = Eigen::VectorXd::Zero(matrix_size);

    // 1. Pozisyon denklemleri (Position Continuity)
    for (int i = 0; i < n; ++i) {

        // S_i(t_i) = P_i (her eksen için)
        A(2 * i, 4 * i + 0) = 1;
        b_x(2 * i) = points_x_mod[i];
        b_y(2 * i) = points_y_mod[i];
        b_z(2 * i) = points_z_mod[i];

        // S_i(t_{i+1}) = P_{i+1}
        double dt = times_mod[i + 1] - times_mod[i];
        A(2 * i + 1, 4 * i + 0) = 1;
        A(2 * i + 1, 4 * i + 1) = dt;
        A(2 * i + 1, 4 * i + 2) = std::pow(dt, 2);
        A(2 * i + 1, 4 * i + 3) = std::pow(dt, 3);
        b_x(2 * i + 1) = points_x_mod[i + 1];
        b_y(2 * i + 1) = points_y_mod[i + 1];
        b_z(2 * i + 1) = points_z_mod[i + 1];
    }
    // 2. Hız sürekliliği (Velocity Continuity)
    for (int i = 1; i < n; ++i) {
        double dt_prev = times_mod[i] - times_mod[i - 1];
        A(2 * n + (i - 1), 4 * (i - 1) + 1) = 1;
        A(2 * n + (i - 1), 4 * (i - 1) + 2) = 2 * dt_prev;
        A(2 * n + (i - 1), 4 * (i - 1) + 3) = 3 * std::pow(dt_prev, 2);
        A(2 * n + (i - 1), 4 * i + 1) = -1;
    }
    // 3. İvme sürekliliği (Acceleration Continuity)
    for (int i = 1; i < n; ++i) {
        double dt_prev = times_mod[i] - times_mod[i - 1];
        A(3 * n + (i - 2), 4 * (i - 1) + 2) = 2;
        A(3 * n - 1 + (i - 1), 4 * (i - 1) + 3) = 6 * dt_prev;
        A(3 * n - 1 + (i - 1), 4 * i + 2) = -2;
    }
    
    // 4. initial velocity and final velocity conditions
    A(4 * n - 2, 1) = 1;

    if (m_initial_trajectory)
    {
        b_x(4 * n - 2) = 0;
        b_y(4 * n - 2) = 0;
        b_z(4 * n - 2) = 0;
        m_initial_trajectory = false;
    } else
    {
        b_x(4 * n - 2) = m_velocity.x;
        b_y(4 * n - 2) = m_velocity.y;
        b_z(4 * n - 2) = m_velocity.z;
    }
    
    double dt_last = times_mod[n] - times_mod[n - 1];
    A(4 * n - 1, 4 * (n - 1) + 1) = 1;
    A(4 * n - 1, 4 * (n - 1) + 2) = 2 * dt_last;
    A(4 * n - 1, 4 * (n - 1) + 3) = 3 * std::pow(dt_last, 2);

    // Denklemi çöz (Ax = b) her eksen için
    Eigen::VectorXd coeffs_x = A.colPivHouseholderQr().solve(b_x);
    Eigen::VectorXd coeffs_y = A.colPivHouseholderQr().solve(b_y);
    Eigen::VectorXd coeffs_z = A.colPivHouseholderQr().solve(b_z);

    cx_ = coeffs_x;
    cy_ = coeffs_y;
    cz_ = coeffs_z;
}

// Matrix set-up copied verbatim from TrajectoryGenerator::solveMinimumJerk()
void TrajectorySequencer::buildMinimumJerk(const std::vector<double>& points_x_mod, const std::vector<double>& points_y_mod,
                                 const std::vector<double>& points_z_mod, const std::vector<double>& times_mod) {
    int n = points_x_mod.size() - 1; // Segment sayısı
    int matrix_size = 6 * n;     // Minimum jerk polinom çözümü için 6n boyutunda matris

    // A matrisi (6n x 6n) ve b vektörleri (6n) her eksen için
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(matrix_size, matrix_size);
    Eigen::VectorXd b_x = Eigen::VectorXd::Zero(matrix_size);
    Eigen::VectorXd b_y = Eigen::VectorXd::Zero(matrix_size);
    Eigen::VectorXd b_z = Eigen::VectorXd::Zero(matrix_size);

    // 1. Pozisyon denklemleri (Position Continuity)
    for (int i = 0; i < n; ++i) {

        // S_i(t_i) = P_i (her eksen için)
        A(2 * i, 6 * i) = 1;
        b_x(2 * i) = points_x_mod[i];
        b_y(2 * i) = points_y_mod[i];
        b_z(2 * i) = points_z_mod[i];

        // S_i(t_{i+1}) = P_{i+1}
        double dt = times_mod[i + 1] - times_mod[i];
        A(2 * i + 1, 6 * i + 0) = 1;
        A(2 * i + 1, 6 * i + 1) = dt;
        A(2 * i + 1, 6 * i + 2) = std::pow(dt, 2);
        A(2 * i + 1, 6 * i + 3) = std::pow(dt, 3);
        A(2 * i + 1, 6 * i + 4) = std::pow(dt, 4);
        A(2 * i + 1, 6 * i + 5) = std::pow(dt, 5);
        
        b_x(2 * i + 1) = points_x_mod[i + 1];
        b_y(2 * i + 1) = points_y_mod[i + 1];
        b_z(2 * i + 1) = points_z_mod[i + 1];
    }

    // Başlangıç hız koşulu (initial velocity)
    A(2 * n, 1) = 1;      // İlk segmentin başındaki hız için katsayılar
    if (m_initial_trajectory)
    {
        b_x(2 * n) = 0;  // X ekseni için başlangıç hızı
        b_y(2 * n) = 0;  // Y ekseni için başlangıç hızı
        b_z(2 * n) = 0;  // Z ekseni için başlangıç hızı
    } else
    {
        b_x(2 * n) = m_velocity.x;  // X ekseni için başlangıç hızı
        b_y(2 * n) = m_velocity.y;  // Y ekseni için başlangıç hızı
        b_z(2 * n) = m_velocity.z;  // Z ekseni için başlangıç hızı
    }
        
    // 2. Hız sürekliliği (Velocity Continuity)
    for (int i = 1; i < n; ++i) {
        double dt_prev = times_mod[i] - times_mod[i - 1];
        A(2 * n + i, 6 * (i - 1) + 1) = 1;
        A(2 * n + i, 6 * (i - 1) + 2) = 2 * dt_prev;
        A(2 * n + i, 6 * (i - 1) + 3) = 3 * std::pow(dt_prev, 2);
        A(2 * n + i, 6 * (i - 1) + 4) = 4 * std::pow(dt_prev, 3);
        A(2 * n + i, 6 * (i - 1) + 5) = 5 * std::pow(dt_prev, 4);
        A(2 * n + i, 6 * i + 1) = -1;
        // b vektöründe hız sürekliliği denklemleri için sıfır ekleniyor
        b_x(2 * n + i) = 0;  // X ekseni için hız sürekliliği
        b_y(2 * n + i) = 0;  // Y ekseni için hız sürekliliği
        b_z(2 * n + i) = 0;  // Z ekseni için hız sürekliliği
    }

    // Son hız değeri (last velocity condition) -> dx_n(t_n) = v_n
    double dt_last = times_mod[n] - times_mod[n - 1];
    A(3 * n, 6 * (n - 1) + 1) = 1;
    A(3 * n, 6 * (n - 1) + 2) = 2 * dt_last;
    A(3 * n, 6 * (n - 1) + 3) = 3 * std::pow(dt_last, 2);
    A(3 * n, 6 * (n - 1) + 4) = 4 * std::pow(dt_last, 3);
    A(3 * n, 6 * (n - 1) + 5) = 5 * std::pow(dt_last, 4);

    // Son hız değeri sağlanacak: b_x, b_y, b_z vektörlerine son hız koşulu eklenmeli
    b_x(3 * n) = 0;  // Son hız değeri
    b_y(3 * n) = 0;  // Son hız değeri
    b_z(3 * n) = 0;  // Son hız değeri

    // Başlangıç ivmesi (initial acceleration)
    A(3 * n + 1, 2) = 2;
    
    if (m_initial_trajectory)
    {
        b_x(3 * n + 1) = 0;  // X ekseni için başlangıç ivmesi
        b_y(3 * n + 1) = 0;  // Y ekseni için başlangıç ivmesi
        b_z(3 * n + 1) = 0;  // Z ekseni için başlangıç ivmesi
        m_initial_trajectory = false;
    } else
    {
        b_x(3 * n + 1) = m_acceleration.x;  // X ekseni için başlangıç ivmesi
        b_y(3 * n + 1) = m_acceleration.y;  // Y ekseni için başlangıç ivmesi
        b_z(3 * n + 1) = m_acceleration.z;  // Z ekseni için başlangıç ivmesi
    }
       
    // 3. İvme sürekliliği (Acceleration Continuity)
    for (int i = 1; i < n; ++i) {
        double dt_prev = times_mod[i] - times_mod[i - 1];
        // İvme sürekliliği denklemleri
        A(3 * n + 1 + i, 6 * (i - 1) + 2) = 2;
        A(3 * n + 1 + i, 6 * (i - 1) + 3) = 6 * dt_prev;
        A(3 * n + 1 + i, 6 * (i - 1) + 4) = 12 * std::pow(dt_prev, 2);
        A(3 * n + 1 + i, 6 * (i - 1) + 5) = 20 * std::pow(dt_prev, 3);
        A(3 * n + 1 + i, 6 * i + 2) = -2;
        // b vektörlerine ivme sürekliliği için sıfır ekleniyor
        b_x(3 * n + 1 + i) = 0;
        b_y(3 * n + 1 + i) = 0;
        b_z(3 * n + 1 + i) = 0;
    }
    
    // Bitiş ivmesi (final acceleration)
    A(4 * n + 1, 6 * (n - 1) + 2) = 2;
    A(4 * n + 1, 6 * (n - 1) + 3) = 6 * dt_last;
    A(4 * n + 1, 6 * (n - 1) + 4) = 12 * std::pow(dt_last, 2);
    A(4 * n + 1, 6 * (n - 1) + 5) = 20 * std::pow(dt_last, 3);
    b_x(4 * n + 1) = 0;  // X ekseni için bitiş ivmesi
    b_y(4 * n + 1) = 0;  // Y ekseni için bitiş ivmesi
    b_z(4 * n + 1) = 0;  // Z ekseni için bitiş ivmesi
    
    // 4. Jerk sürekliliği (Jerk Continuity)
    for (int i = 1; i < n; ++i) {
        double dt_prev = times_mod[i] - times_mod[i - 1];
        A(4 * n - 1 + (i + 2), 6 * (i - 1) + 3) = 6;
        A(4 * n - 1 + (i + 2), 6 * (i - 1) + 4) = 24 * dt_prev;
        A(4 * n - 1 + (i + 2), 6 * (i - 1) + 5) = 60 * std::pow(dt_prev, 2);
        A(4 * n - 1 + (i + 2), 6 * i + 3) = -6;
        b_x(4 * n - 1 + (i + 2)) = 0;  
        b_y(4 * n - 1 + (i + 2)) = 0;  
        b_z(4 * n - 1 + (i + 2)) = 0;  
    }
    // 5. Snap sürekliliği (Snap Continuity)
    for (int i = 1; i < n; ++i) {
        double dt_prev = times_mod[i] - times_mod[i - 1];
        A(5 * n + i, 6 * (i - 1) + 4) = 24;
        A(5 * n + i, 6 * (i - 1) + 5) = 120 * dt_prev;
        A(5 * n + i, 6 * i + 4) = -24;
        b_x(5 * n + i) = 0;  
        b_y(5 * n + i) = 0;  
        b_z(5 * n + i) = 0;  
    }
    // Denklemi çöz (Ax = b) her eksen için
    Eigen::VectorXd coeffs_x = A.colPivHouseholderQr().solve(b_x);
    Eigen::VectorXd coeffs_y = A.colPivHouseholderQr().solve(b_y);
    Eigen::VectorXd coeffs_z = A.colPivHouseholderQr().solve(b_z);

    cx_ = coeffs_x;
    cy_ = coeffs_y;
    cz_ = coeffs_z;
}

// Matrix set-up copied verbatim from TrajectoryGenerator::solveMinimumSnap()
void TrajectorySequencer::buildMinimumSnap(const std::vector<double>& points_x_mod, const std::vector<double>& points_y_mod,
                                 const std::vector<double>& points_z_mod, const std::vector<double>& times_mod) {
    int n = points_x_mod.size() - 1; // Segment sayısı
    int matrix_size = 8 * n;     // Minimum jerk polinom çözümü için 8n boyutunda matris
    
    // A matrisi (8n x 8n) ve b vektörleri (8n) her eksen için
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(matrix_size, matrix_size);
    Eigen::VectorXd b_x = Eigen::VectorXd::Zero(matrix_size);
    Eigen::VectorXd b_y = Eigen::VectorXd::Zero(matrix_size);
    Eigen::VectorXd b_z = Eigen::VectorXd::Zero(matrix_size);
    
    // 1. Pozisyon denklemleri (Position Continuity)
    for (int i = 0; i < n; ++i) {
        
        // S_i(t_i) = P_i (her eksen için)
        A(2 * i, 8 * i) = 1;
        b_x(2 * i) = points_x_mod[i];
        b_y(2 * i) = points_y_mod[i];
        b_z(2 * i) = points_z_mod[i];
        
        // S_i(t_{i+1}) = P_{i+1}
        double dt = times_mod[i + 1] - times_mod[i];
        A(2 * i + 1, 8 * i + 0) = 1;
        A(2 * i + 1, 8 * i + 1) = dt;
        A(2 * i + 1, 8 * i + 2) = std::pow(dt, 2);
        A(2 * i + 1, 8 * i + 3) = std::pow(dt, 3);
        A(2 * i + 1, 8 * i + 4) = std::pow(dt, 4);
        A(2 * i + 1, 8 * i + 5) = std::pow(dt, 5);
        A(2 * i + 1, 8 * i + 6) = std::pow(dt, 6);
        A(2 * i + 1, 8 * i + 7) = std::pow(dt, 7);
        
        b_x(2 * i + 1) = points_x_mod[i + 1];
        b_y(2 * i + 1) = points_y_mod[i + 1];
        b_z(2 * i + 1) = points_z_mod[i + 1];
    }
    
    // Başlangıç hız koşulu (initial velocity)
    A(2 * n, 1) = 1;      // İlk segmentin başındaki hız için katsayılar
    
    if (m_initial_trajectory)
    {
        b_x(2 * n) = 0;  // X ekseni için başlangıç hızı
        b_y(2 * n) = 0;  // Y ekseni için başlangıç hızı
        b_z(2 * n) = 0;  // Z ekseni için başlangıç hızı
    }
    else
    {
        b_x(2 * n) = m_velocity.x;  // X ekseni için başlangıç hızı
        b_y(2 * n) = m_velocity.y;  // Y ekseni için başlangıç hızı
        b_z(2 * n) = m_velocity.z;  // Z ekseni için başlangıç hızı    /* code */
    }
    
    // 2. Hız sürekliliği (Velocity Continuity)
    for (int i = 1; i < n; ++i) {
        double dt_prev = times_mod[i] - times_mod[i - 1];
        A(2 * n + i, 8 * (i - 1) + 1) = 1;
        A(2 * n + i, 8 * (i - 1) + 2) = 2 * dt_prev;
        A(2 * n + i, 8 * (i - 1) + 3) = 3 * std::pow(dt_prev, 2);
        A(2 * n + i, 8 * (i - 1) + 4) = 4 * std::pow(dt_prev, 3);
        A(2 * n + i, 8 * (i - 1) + 5) = 5 * std::pow(dt_prev, 4);
        A(2 * n + i, 8 * (i - 1) + 6) = 6 * std::pow(dt_prev, 5);
        A(2 * n + i, 8 * (i - 1) + 7) = 7 * std::pow(dt_prev, 6);
        A(2 * n + i, 8 * i + 1) = -1;
        // b vektöründe hız sürekliliği denklemleri için sıfır ekleniyor
        b_x(2 * n + i) = 0;  // X ekseni için hız sürekliliği
        b_y(2 * n + i) = 0;  // Y ekseni için hız sürekliliği
        b_z(2 * n + i) = 0;  // Z ekseni için hız sürekliliği
    }

    // Son hız değeri (last velocity condition) -> dx_n(t_n) = v_n
    double dt_last = times_mod[n] - times_mod[n - 1];
    A(3 * n, 8 * (n - 1) + 1) = 1;
    A(3 * n, 8 * (n - 1) + 2) = 2 * dt_last;
    A(3 * n, 8 * (n - 1) + 3) = 3 * std::pow(dt_last, 2);
    A(3 * n, 8 * (n - 1) + 4) = 4 * std::pow(dt_last, 3);
    A(3 * n, 8 * (n - 1) + 5) = 5 * std::pow(dt_last, 4);
    A(3 * n, 8 * (n - 1) + 6) = 6 * std::pow(dt_last, 5);
    A(3 * n, 8 * (n - 1) + 7) = 7 * std::pow(dt_last, 6);

    // Son hız değeri sağlanacak: b_x, b_y, b_z vektörlerine son hız koşulu eklenmeli
    b_x(3 * n) = 0;  // Son hız değeri
    b_y(3 * n) = 0;  // Son hız değeri
    b_z(3 * n) = 0;  // Son hız değeri

    // Başlangıç ivmesi (initial acceleration)
    A(3 * n + 1, 2) = 2;
    
    if (m_initial_trajectory)
    {
        b_x(3 * n + 1) = 0;  // X ekseni için başlangıç ivmesi
        b_y(3 * n + 1) = 0;  // Y ekseni için başlangıç ivmesi
        b_z(3 * n + 1) = 0;  // Z ekseni için başlangıç ivmesi
    }
    else
    {
        b_x(3 * n + 1) = m_acceleration.x;  // X ekseni için başlangıç ivmesi
        b_y(3 * n + 1) = m_acceleration.y;  // Y ekseni için başlangıç ivmesi
        b_z(3 * n + 1) = m_acceleration.z;  // Z ekseni için başlangıç ivmesi
    }
    
    // 3. İvme sürekliliği (Acceleration Continuity)
    for (int i = 1; i < n; ++i) {
        double dt_prev = times_mod[i] - times_mod[i - 1];
        // İvme sürekliliği denklemleri
        A(3 * n + 1 + i, 8 * (i - 1) + 2) = 2;
        A(3 * n + 1 + i, 8 * (i - 1) + 3) = 6 * dt_prev;
        A(3 * n + 1 + i, 8 * (i - 1) + 4) = 12 * std::pow(dt_prev, 2);
        A(3 * n + 1 + i, 8 * (i - 1) + 5) = 20 * std::pow(dt_prev, 3);
        A(3 * n + 1 + i, 8 * (i - 1) + 6) = 30 * std::pow(dt_prev, 4);
        A(3 * n + 1 + i, 8 * (i - 1) + 7) = 42 * std::pow(dt_prev, 5);
        A(3 * n + 1 + i, 8 * i + 2) = -2;
        // b vektörlerine ivme sürekliliği için sıfır ekleniyor
        b_x(3 * n + 1 + i) = 0;
        b_y(3 * n + 1 + i) = 0;
        b_z(3 * n + 1 + i) = 0;
    }
    
    // Bitiş ivmesi (final acceleration)
    A(4 * n + 1, 8 * (n - 1) + 2) = 2;
    A(4 * n + 1, 8 * (n - 1) + 3) = 6 * dt_last;
    A(4 * n + 1, 8 * (n - 1) + 4) = 12 * std::pow(dt_last, 2);
    A(4 * n + 1, 8 * (n - 1) + 5) = 20 * std::pow(dt_last, 3);
    A(4 * n + 1, 8 * (n - 1) + 6) = 30 * std::pow(dt_last, 4);
    A(4 * n + 1, 8 * (n - 1) + 7) = 42 * std::pow(dt_last, 5);
    b_x(4 * n + 1) = 0;  // X ekseni için bitiş ivmesi
    b_y(4 * n + 1) = 0;  // Y ekseni için bitiş ivmesi
    b_z(4 * n + 1) = 0;  // Z ekseni için bitiş ivmesi

    // Başlangıç jerk (initial jerk)
    A(4 * n + 2, 3) = 6;
    
    if (m_initial_trajectory)
    {
        b_x(4 * n + 2) = 0;  // X ekseni için başlangıç jerk
        b_y(4 * n + 2) = 0;  // Y ekseni için başlangıç jerk
        b_z(4 * n + 2) = 0;  // Z ekseni için başlangıç jerk
        m_initial_trajectory = false;
    }
    else
    {
        b_x(4 * n + 2) = m_jerk.x;  // X ekseni için başlangıç jerk
        b_y(4 * n + 2) = m_jerk.y;  // Y ekseni için başlangıç jerk
        b_z(4 * n + 2) = m_jerk.z;  // Z ekseni için başlangıç jerk
    }
    
    // 4. Jerk sürekliliği (Jerk Continuity)
    for (int i = 1; i < n; ++i) {
        double dt_prev = times_mod[i] - times_mod[i - 1];
        A(4 * n + 2 + i, 8 * (i - 1) + 3) = 6;
        A(4 * n + 2 + i, 8 * (i - 1) + 4) = 24 * dt_prev;
        A(4 * n + 2 + i, 8 * (i - 1) + 5) = 60 * std::pow(dt_prev, 2);
        A(4 * n + 2 + i, 8 * (i - 1) + 6) = 120 * std::pow(dt_prev, 3);
        A(4 * n + 2 + i, 8 * (i - 1) + 7) = 210 * std::pow(dt_prev, 4);
        A(4 * n + 2 + i, 8 * i + 3) = -6;
        b_x(4 * n + 2 + i) = 0;  
        b_y(4 * n + 2 + i) = 0;  
        b_z(4 * n + 2 + i) = 0;  
    }
    
    // Bitiş jerk (final jerk)
    A(5 * n + 2, 8 * (n - 1) + 3) = 6;
    A(5 * n + 2, 8 * (n - 1) + 4) = 24 * dt_last;
    A(5 * n + 2, 8 * (n - 1) + 5) = 60 * std::pow(dt_last, 2);
    A(5 * n + 2, 8 * (n - 1) + 6) = 120 * std::pow(dt_last, 3);
    A(5 * n + 2, 8 * (n - 1) + 7) = 210 * std::pow(dt_last, 4);
    b_x(5 * n + 2) = 0;  // X ekseni için bitiş ivmesi
    b_y(5 * n + 2) = 0;  // Y ekseni için bitiş ivmesi
    b_z(5 * n + 2) = 0;  // Z ekseni için bitiş ivmesi
    
    // 5. Snap sürekliliği (Snap Continuity)
    for (int i = 1; i < n; ++i) {
        double dt_prev = times_mod[i] - times_mod[i - 1];
        A(5 * n + i + 2, 8 * (i - 1) + 4) = 24;
        A(5 * n + i + 2, 8 * (i - 1) + 5) = 120 * dt_prev;
        A(5 * n + i + 2, 8 * (i - 1) + 6) = 360 * std::pow(dt_prev, 2);
        A(5 * n + i + 2, 8 * (i - 1) + 7) = 840 * std::pow(dt_prev, 3);
        A(5 * n + i + 2, 8 * i + 4) = -24;
        b_x(5 * n + i + 2) = 0;  
        b_y(5 * n + i + 2) = 0;  
        b_z(5 * n + i + 2) = 0;  
    }
    
    // 6. Crackle sürekliliği (Crackle Continuity)
    for (int i = 1; i < n; ++i) {
        double dt_prev = times_mod[i] - times_mod[i - 1];
        A(6 * n + i + 1, 8 * (i - 1) + 5) = 120;
        A(6 * n + i + 1, 8 * (i - 1) + 6) = 720 * dt_prev;
        A(6 * n + i + 1, 8 * (i - 1) + 7) = 2520 * std::pow(dt_prev, 2);
        A(6 * n + i + 1, 8 * i + 5) = -120;
        b_x(6 * n + i + 1) = 0;  
        b_y(6 * n + i + 1) = 0;  
        b_z(6 * n + i + 1) = 0;  
    }
    
    // 7. Pop sürekliliği (Pop Continuity)
    for (int i = 1; i < n; ++i) {
        double dt_prev = times_mod[i] - times_mod[i - 1];
        A(7 * n + i, 8 * (i - 1) + 6) = 720;
        A(7 * n + i, 8 * (i - 1) + 7) = 5040 * dt_prev;
        A(7 * n + i, 8 * i + 6) = -720;
        b_x(7 * n + i) = 0;  
        b_y(7 * n + i) = 0;  
        b_z(7 * n + i) = 0;  
    }
    
    // Denklemi çöz (Ax = b) her eksen için
    Eigen::VectorXd coeffs_x = A.colPivHouseholderQr().solve(b_x);
    Eigen::VectorXd coeffs_y = A.colPivHouseholderQr().solve(b_y);
    Eigen::VectorXd coeffs_z = A.colPivHouseholderQr().solve(b_z);

    cx_ = coeffs_x;
    cy_ = coeffs_y;
    cz_ = coeffs_z;
}

}  // namespace mqc_sim
