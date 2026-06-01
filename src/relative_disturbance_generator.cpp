#include <ros/ros.h>
#include <ros/master.h>
#include <geometry_msgs/Vector3.h>
#include <map>
#include <unordered_map>
#include <cmath>
#include <vector>
#include <string>
#include <Eigen/Dense>
#include <random>
#include <array>
#include <algorithm>
#include <boost/bind.hpp>

// ======================================================================================
// 1. PINK NOISE GENERATOR (Voss-McCartney Algorithm)
// ======================================================================================
// Manuscript Eq. (11): 1/f karakterli colored-noise davranışının gerçek zamanlı karşılığı.
class PinkNoiseGenerator {
public:
    PinkNoiseGenerator() : counter_(0) {
        std::random_device rd;
        gen_ = std::mt19937(rd());
        dist_ = std::uniform_real_distribution<double>(-1.0, 1.0);
        for (int i = 0; i < NUM_ROWS; ++i) {
            rows_[i] = dist_(gen_);
        }
    }

    double next() {
        const int last_counter = counter_;
        counter_++;

        int number_of_changes = counter_ ^ last_counter;
        int row_to_change = 0;
        while (number_of_changes >>= 1) {
            row_to_change++;
        }

        if (row_to_change < NUM_ROWS) {
            rows_[row_to_change] = dist_(gen_);
        }

        double sum = 0.0;
        for (int i = 0; i < NUM_ROWS; ++i) {
            sum += rows_[i];
        }

        // Yaklaşık [-1, 1] bandına ölçekleme.
        return sum / 5.0;
    }

private:
    static const int NUM_ROWS = 16;
    std::array<double, NUM_ROWS> rows_;
    int counter_;
    std::mt19937 gen_;
    std::uniform_real_distribution<double> dist_;
};

struct PinkNoise3D {
    PinkNoiseGenerator px, py, pz;

    Eigen::Vector3d getVector() {
        return Eigen::Vector3d(px.next(), py.next(), pz.next());
    }
};

// ======================================================================================
// 2. GRID VE HASH YAPILARI (Spatial Hashing)
// ======================================================================================
struct GridKey {
    int x, y, z;

    bool operator==(const GridKey& other) const {
        return x == other.x && y == other.y && z == other.z;
    }
};

struct GridKeyHash {
    std::size_t operator()(const GridKey& k) const {
        return std::hash<int>()(k.x) ^
               (std::hash<int>()(k.y) << 1) ^
               (std::hash<int>()(k.z) << 2);
    }
};

// ======================================================================================
// 3. ANA SINIF: DroneDisturbanceManager
// ======================================================================================
class DroneDisturbanceManager {
public:
    DroneDisturbanceManager()
        : rate_(100.0),
          topic_check_interval_(1.0),
          drone_count_logged_(false) {

        loadGlobalModelParameters();
        updateGridParameters();

        // Başlangıç offset ve orientation yer tutucuları.
        // Drone sayısı dinamik olarak topic üzerinden bulunuyor; burası sadece olası namespace'ler için offset okur.
        for (int i = 1; i <= 50; ++i) {
            const std::string drone_name = "drone" + std::to_string(i);
            const std::string ns = "/" + drone_name + "/";

            geometry_msgs::Vector3 offset;
            if (nh_.getParam(ns + "initial_x", offset.x)) {
                nh_.getParam(ns + "initial_y", offset.y);
                nh_.getParam(ns + "initial_z", offset.z);
            } else {
                offset.x = 0.0;
                offset.y = 0.0;
                offset.z = 0.0;
            }

            drone_initial_offsets_[drone_name] = offset;
            drone_orientations_[drone_name] = Eigen::Quaterniond::Identity();
        }

        grid_map_cache_.reserve(1000);
        last_topic_check_ = ros::Time::now();
    }

    void run() {
        while (ros::ok()) {
            checkNewTopics();
            logDroneCount();
            computeAndPublishDisturbancesOptimized();
            ros::spinOnce();
            rate_.sleep();
        }
    }

private:
    ros::NodeHandle nh_;
    ros::Rate rate_;
    ros::Time last_topic_check_;
    ros::Duration topic_check_interval_;
    bool drone_count_logged_;

    // -----------------------------
    // Manuscript model parametreleri
    // -----------------------------
    double g_;
    double aerodynamic_coupling_;      // C_coup, default: 0.065
    double lateral_force_ratio_;       // kappa_lat, default: 0.05
    double singularity_eps_;           // z0, default: 0.1
    double spread_coefficient_;        // S, default: 0.15
    double virtual_origin_;            // s_v, default: 0.0
    double sigma_min_;                 // sayısal güvenlik için minimum sigma
    double turb_intensity_;            // lambda, default: 0.3
    double sigma_cutoff_multiplier_;   // Algorithm I: |p_xy| < 2 sigma
    double max_vertical_effect_;       // grid arama sınırı
    bool body_z_positive_down_;        // false: ROS/Eigen z-up kabulü

    double cell_size_;
    int neighbor_range_;

    // Drone verileri
    std::map<std::string, geometry_msgs::Vector3> drone_positions_;
    std::map<std::string, Eigen::Quaterniond> drone_orientations_;
    std::map<std::string, double> drone_masses_;

    // ROS iletişimi
    std::map<std::string, ros::Subscriber> sub_pos_;
    std::map<std::string, ros::Subscriber> sub_orient_;
    std::map<std::string, geometry_msgs::Vector3> drone_initial_offsets_;
    std::map<std::string, ros::Publisher> pub_total_;
    std::map<std::string, ros::Publisher> pub_static_;

    // Optimizasyon yapıları
    std::unordered_map<GridKey, std::vector<std::string>, GridKeyHash> grid_map_cache_;
    std::map<std::string, PinkNoise3D> noise_generators_;

    // ==================================================================================
    // PARAMETRELER
    // ==================================================================================
    void loadGlobalModelParameters() {
        nh_.param("/disturbance_model/gravity", g_, 9.81);
        nh_.param("/disturbance_model/aerodynamic_coupling", aerodynamic_coupling_, 0.25);
        nh_.param("/disturbance_model/lateral_force_ratio", lateral_force_ratio_, 0.05);
        nh_.param("/disturbance_model/singularity_prevention", singularity_eps_, 0.1);
        nh_.param("/disturbance_model/spread_coefficient", spread_coefficient_, 0.3);
        nh_.param("/disturbance_model/virtual_origin", virtual_origin_, 0.0);
        nh_.param("/disturbance_model/sigma_min", sigma_min_, 0.03);
        nh_.param("/disturbance_model/turbulence_intensity", turb_intensity_, 0.6);
        nh_.param("/disturbance_model/sigma_cutoff_multiplier", sigma_cutoff_multiplier_, 2.0);
        nh_.param("/disturbance_model/max_vertical_effect", max_vertical_effect_, 3.0);
        nh_.param("/disturbance_model/body_z_positive_down", body_z_positive_down_, false);

        ROS_INFO("Disturbance model parameters loaded: C_coup=%.4f, kappa_lat=%.4f, z0=%.4f, S=%.4f, lambda=%.4f",
                 aerodynamic_coupling_, lateral_force_ratio_, singularity_eps_, spread_coefficient_, turb_intensity_);
    }

    void updateGridParameters() {
        const double sigma_at_max_height = localSigma(max_vertical_effect_);
        const double lateral_radius = sigma_cutoff_multiplier_ * sigma_at_max_height;

        cell_size_ = std::max(0.25, lateral_radius);
        neighbor_range_ = std::max(1, static_cast<int>(std::ceil(max_vertical_effect_ / cell_size_)) + 1);

        ROS_INFO("Disturbance grid: cell_size=%.3f m, neighbor_range=%d", cell_size_, neighbor_range_);
    }

    bool getParamAny(const std::vector<std::string>& names, double& value) {
        for (const auto& name : names) {
            if (nh_.getParam(name, value)) {
                return true;
            }
        }
        return false;
    }

    void ensureDroneParameters(const std::string& drone_name) {
        if (drone_masses_.find(drone_name) != drone_masses_.end()) {
            return;
        }

        double mass = 0.382;
        const std::string ns = "/" + drone_name;

        // Heterogeneous swarm için kaynak UAV'nin kendi kütlesi öncelikli okunur.
        // Bulunamazsa eski global parametreye, o da yoksa default değere döner.
        getParamAny({
            ns + "/state_derivative_solver_node/mass",
            ns + "/mass",
            "/state_derivative_solver_node/mass"
        }, mass);

        drone_masses_[drone_name] = mass;
        ROS_INFO("Disturbance Manager: %s mass used for W_i matrix: %.4f kg", drone_name.c_str(), mass);
    }

    Eigen::Matrix3d influenceMatrixForSource(const std::string& source_name) {
        ensureDroneParameters(source_name);

        // Manuscript Eq. (9): W_i,z = f_thrust * C_coup; W_i,x = W_i,y = W_i,z * kappa_lat.
        // Burada f_thrust, source UAV için hover thrust olarak alınır: m_i * g.
        // Gerçek thrust topic'i bağlanacaksa bu satır source thrust ölçümü/komutu ile değiştirilebilir.
        const double source_hover_thrust = drone_masses_[source_name] * g_;
        const double w_z = source_hover_thrust * aerodynamic_coupling_;
        const double w_x = w_z * lateral_force_ratio_;
        const double w_y = w_z * lateral_force_ratio_;

        Eigen::Matrix3d W = Eigen::Matrix3d::Zero();
        W(0, 0) = w_x;
        W(1, 1) = w_y;
        W(2, 2) = w_z;
        return W;
    }

    // Manuscript: sigma(s) = r_1/2(s) / sqrt(2 ln 2), r_1/2(s) = S (s - s_v).
    double localSigma(double vertical_distance) const {
        const double r_half = spread_coefficient_ * std::max(0.0, vertical_distance - virtual_origin_);
        const double sigma = r_half / std::sqrt(2.0 * std::log(2.0));
        return std::max(sigma, sigma_min_);
    }

    // ==================================================================================
    // YARDIMCI FONKSİYONLAR
    // ==================================================================================
    GridKey getGridKey(const geometry_msgs::Vector3& pos) const {
        return {
            static_cast<int>(std::floor(pos.x / cell_size_)),
            static_cast<int>(std::floor(pos.y / cell_size_)),
            static_cast<int>(std::floor(pos.z / cell_size_))
        };
    }

    std::vector<std::string> getTopicsByType(const std::string& key_phrase, const std::string& type) {
        ros::master::V_TopicInfo master_topics;
        ros::master::getTopics(master_topics);

        std::vector<std::string> found_topics;
        for (const auto& topic : master_topics) {
            if (topic.name.find(key_phrase) != std::string::npos && topic.datatype == type) {
                found_topics.push_back(topic.name);
            }
        }
        return found_topics;
    }

    std::string extractDroneName(const std::string& topic_name) const {
        const size_t first_slash = topic_name.find("/");
        const size_t second_slash = topic_name.find("/", first_slash + 1);

        if (first_slash != std::string::npos && second_slash != std::string::npos) {
            return topic_name.substr(first_slash + 1, second_slash - first_slash - 1);
        }
        return topic_name;
    }

    bool isTargetInsideDownwashColumn(const Eigen::Vector3d& p_rel_body) const {
        // Manuscript formülü p_z > 0 tarafını etkin kabul eder.
        // Bu kodda default ROS/Eigen z-up kullanıldığı için source UAV'nin altı p_z < 0 kabul edilmiştir.
        if (body_z_positive_down_) {
            return p_rel_body.z() > 0.0;
        }
        return p_rel_body.z() < 0.0;
    }

    Eigen::Vector3d signPreservingRSS(const Eigen::Vector3d& linear_sum,
                                      const Eigen::Vector3d& squared_sum) const {
        Eigen::Vector3d result = Eigen::Vector3d::Zero();

        for (int axis = 0; axis < 3; ++axis) {
            if (squared_sum(axis) <= 0.0 || std::abs(linear_sum(axis)) < 1e-12) {
                result(axis) = 0.0;
            } else {
                result(axis) = std::copysign(std::sqrt(squared_sum(axis)), linear_sum(axis));
            }
        }
        return result;
    }

    void accumulateRSS(std::map<std::string, Eigen::Vector3d>& linear_sum,
                       std::map<std::string, Eigen::Vector3d>& squared_sum,
                       const std::string& target_name,
                       const Eigen::Vector3d& contribution) {
        linear_sum[target_name] += contribution;
        squared_sum[target_name] += contribution.cwiseProduct(contribution);
    }

    // ==================================================================================
    // CALLBACK FONKSİYONLARI
    // ==================================================================================
    void positionCallback(const geometry_msgs::Vector3::ConstPtr& msg, const std::string& topic_name) {
        const std::string drone_name = extractDroneName(topic_name);
        const geometry_msgs::Vector3 offset = drone_initial_offsets_[drone_name];

        geometry_msgs::Vector3 adjusted_position;
        adjusted_position.x = msg->x + offset.x;
        adjusted_position.y = msg->y + offset.y;
        adjusted_position.z = msg->z + offset.z;

        drone_positions_[drone_name] = adjusted_position;
    }

    void eulerCallback(const geometry_msgs::Vector3::ConstPtr& msg, const std::string& topic_name) {
        const std::string drone_name = extractDroneName(topic_name);

        const double roll = msg->x;
        const double pitch = msg->y;
        const double yaw = msg->z;

        // Euler -> Quaternion, ZYX sırası: yaw, pitch, roll.
        Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());

        const Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;
        drone_orientations_[drone_name] = q.normalized();
    }

    // ==================================================================================
    // TOPIC KONTROLÜ
    // ==================================================================================
    void checkNewTopics() {
        if ((ros::Time::now() - last_topic_check_) <= topic_check_interval_) {
            return;
        }

        // 1. Pozisyon topic'lerini tara.
        const std::vector<std::string> pos_topics = getTopicsByType("/actual_position", "geometry_msgs/Vector3");
        for (const auto& topic : pos_topics) {
            if (sub_pos_.find(topic) != sub_pos_.end()) {
                continue;
            }

            sub_pos_[topic] = nh_.subscribe<geometry_msgs::Vector3>(
                topic, 10, boost::bind(&DroneDisturbanceManager::positionCallback, this, _1, topic));

            const std::string drone_name = extractDroneName(topic);
            ensureDroneParameters(drone_name);

            pub_total_[drone_name] = nh_.advertise<geometry_msgs::Vector3>("/" + drone_name + "/disturbance_force", 10);
            pub_static_[drone_name] = nh_.advertise<geometry_msgs::Vector3>("/" + drone_name + "/disturbance_static", 10);

            if (noise_generators_.find(drone_name) == noise_generators_.end()) {
                noise_generators_[drone_name] = PinkNoise3D();
            }
        }

        // 2. Oryantasyon topic'lerini tara.
        const std::vector<std::string> orient_topics = getTopicsByType("actual_euler_angles", "geometry_msgs/Vector3");
        for (const auto& topic : orient_topics) {
            if (sub_orient_.find(topic) != sub_orient_.end()) {
                continue;
            }

            sub_orient_[topic] = nh_.subscribe<geometry_msgs::Vector3>(
                topic, 10, boost::bind(&DroneDisturbanceManager::eulerCallback, this, _1, topic));
        }

        last_topic_check_ = ros::Time::now();
    }

    void logDroneCount() {
        if (!drone_count_logged_ && !sub_pos_.empty()) {
            ROS_INFO("Disturbance Manager: Active drones connected: %zu", sub_pos_.size());
            drone_count_logged_ = true;
        }
    }

    // ==================================================================================
    // 4. HESAPLAMA DÖNGÜSÜ: MANUSCRIPT MODEL + OPTIMIZED GRID
    // ==================================================================================
    void computeAndPublishDisturbancesOptimized() {
        std::map<std::string, Eigen::Vector3d> total_linear_world;
        std::map<std::string, Eigen::Vector3d> total_squared_world;
        std::map<std::string, Eigen::Vector3d> static_linear_world;
        std::map<std::string, Eigen::Vector3d> static_squared_world;

        // --- 4.1. Spatial hashing ---
        grid_map_cache_.clear();
        for (const auto& pair : drone_positions_) {
            const std::string& drone_name = pair.first;

            total_linear_world[drone_name] = Eigen::Vector3d::Zero();
            total_squared_world[drone_name] = Eigen::Vector3d::Zero();
            static_linear_world[drone_name] = Eigen::Vector3d::Zero();
            static_squared_world[drone_name] = Eigen::Vector3d::Zero();

            const GridKey key = getGridKey(pair.second);
            grid_map_cache_[key].push_back(drone_name);
        }

        // --- 4.2. Pairwise aerodynamic interaction ---
        for (const auto& source_pair : drone_positions_) {
            const std::string source_name = source_pair.first;
            const geometry_msgs::Vector3 source_pos_msg = source_pair.second;

            const Eigen::Vector3d p_source_world(source_pos_msg.x, source_pos_msg.y, source_pos_msg.z);
            const Eigen::Quaterniond q_source = drone_orientations_[source_name].normalized();

            const Eigen::Matrix3d R_source_to_world = q_source.toRotationMatrix();
            const Eigen::Matrix3d R_world_to_source = R_source_to_world.transpose();
            const Eigen::Matrix3d W_source = influenceMatrixForSource(source_name);

            const GridKey source_key = getGridKey(source_pos_msg);

            for (int dx = -neighbor_range_; dx <= neighbor_range_; ++dx) {
                for (int dy = -neighbor_range_; dy <= neighbor_range_; ++dy) {
                    for (int dz = -neighbor_range_; dz <= neighbor_range_; ++dz) {
                        const GridKey neighbor_key = {
                            source_key.x + dx,
                            source_key.y + dy,
                            source_key.z + dz
                        };

                        const auto grid_it = grid_map_cache_.find(neighbor_key);
                        if (grid_it == grid_map_cache_.end()) {
                            continue;
                        }

                        const auto& candidate_targets = grid_it->second;
                        for (const auto& target_name : candidate_targets) {
                            if (source_name == target_name) {
                                continue;
                            }

                            const geometry_msgs::Vector3 target_pos_msg = drone_positions_[target_name];
                            const Eigen::Vector3d p_target_world(target_pos_msg.x, target_pos_msg.y, target_pos_msg.z);

                            // Manuscript Eq. (8): p_j expressed in source UAV body frame B_i.
                            const Eigen::Vector3d p_rel_world = p_target_world - p_source_world;
                            const Eigen::Vector3d p_rel_body = R_world_to_source * p_rel_world;

                            if (!isTargetInsideDownwashColumn(p_rel_body)) {
                                continue;
                            }

                            const double vertical_distance = std::abs(p_rel_body.z());
                            if (vertical_distance < 1e-6 || vertical_distance > max_vertical_effect_) {
                                continue;
                            }

                            const double sigma_s = localSigma(vertical_distance);
                            const double lateral_sq = p_rel_body.x() * p_rel_body.x() +
                                                      p_rel_body.y() * p_rel_body.y();
                            const double lateral_distance = std::sqrt(lateral_sq);

                            // Algorithm I: expensive exponential öncesi erken eleme.
                            if (lateral_distance > sigma_cutoff_multiplier_ * sigma_s) {
                                continue;
                            }

                            const double gaussian = std::exp(-lateral_sq / (2.0 * sigma_s * sigma_s));
                            const double axial_decay = 1.0 / (vertical_distance * vertical_distance + singularity_eps_);

                            // Manuscript Eq. (8): -W_i p / (p_z^2 + z0) * Gaussian.
                            // Default ROS/Eigen z-up kullanımında source altı p_z < 0 olduğu için eşdeğer işaret +W_i p olur.
                            const double model_sign = body_z_positive_down_ ? -1.0 : 1.0;
                            const Eigen::Vector3d f_dw_body = model_sign * (W_source * p_rel_body) * axial_decay * gaussian;

                            // Manuscript Eq. (10): f_turb = lambda * f_dw * zeta.
                            if (noise_generators_.find(target_name) == noise_generators_.end()) {
                                noise_generators_[target_name] = PinkNoise3D();
                            }

                            const Eigen::Vector3d zeta = noise_generators_[target_name].getVector();
                            const Eigen::Vector3d f_turb_body = turb_intensity_ * f_dw_body.cwiseProduct(zeta);

                            // Manuscript Eq. (12): f_dist = f_dw + f_turb.
                            const Eigen::Vector3d f_dist_body = f_dw_body + f_turb_body;

                            // Source body frame -> world frame.
                            const Eigen::Vector3d f_dw_world = R_source_to_world * f_dw_body;
                            const Eigen::Vector3d f_dist_world = R_source_to_world * f_dist_body;

                            // Manuscript Eq. (13): linear summation yerine sign-preserving RSS.
                            accumulateRSS(static_linear_world, static_squared_world, target_name, f_dw_world);
                            accumulateRSS(total_linear_world, total_squared_world, target_name, f_dist_world);
                        }
                    }
                }
            }
        }

        // --- 4.3. Publish ---
        for (const auto& pair : total_linear_world) {
            const std::string& drone_name = pair.first;

            const Eigen::Vector3d f_total_world = signPreservingRSS(
                total_linear_world[drone_name], total_squared_world[drone_name]);

            const Eigen::Vector3d f_static_world = signPreservingRSS(
                static_linear_world[drone_name], static_squared_world[drone_name]);

            geometry_msgs::Vector3 msg_total;
            msg_total.x = f_total_world.x();
            msg_total.y = f_total_world.y();
            msg_total.z = f_total_world.z();
            pub_total_[drone_name].publish(msg_total);

            geometry_msgs::Vector3 msg_static;
            msg_static.x = f_static_world.x();
            msg_static.y = f_static_world.y();
            msg_static.z = f_static_world.z();
            pub_static_[drone_name].publish(msg_static);
        }
    }
};

// ======================================================================================
// 5. MAIN
// ======================================================================================
int main(int argc, char** argv) {
    ros::init(argc, argv, "relative_disturbance_generator");
    DroneDisturbanceManager manager;
    manager.run();
    return 0;
}
