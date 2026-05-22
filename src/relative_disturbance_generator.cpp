#include <ros/ros.h>
#include <ros/master.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <map>
#include <unordered_map> 
#include <cmath>
#include <vector>
#include <string>
#include <Eigen/Dense>
#include <random>
#include <array>

// ======================================================================================
// 1. PINK NOISE GENERATOR (Voss-McCartney Algorithm)
// ======================================================================================
// MATLAB'daki spektral yöntemin Real-Time çalışan karşılığıdır.
class PinkNoiseGenerator {
public:
    PinkNoiseGenerator() : counter_(0) {
        std::random_device rd;
        gen_ = std::mt19937(rd());
        dist_ = std::uniform_real_distribution<double>(-1.0, 1.0);
        // Başlangıç durumunu rastgele doldur
        for (int i = 0; i < NUM_ROWS; ++i) rows_[i] = dist_(gen_);
    }

    double next() {
        int last_counter = counter_;
        counter_++;

        // Hangi oktavın güncelleneceğini belirle
        int number_of_changes = counter_ ^ last_counter;
        int row_to_change = 0;
        while (number_of_changes >>= 1) row_to_change++;
        
        if (row_to_change < NUM_ROWS) {
            rows_[row_to_change] = dist_(gen_);
        }

        double sum = 0.0;
        for (int i = 0; i < NUM_ROWS; ++i) {
            sum += rows_[i];
        }

        // Normalizasyon (Ortalama +/- 1.0 aralığına getir)
        return (sum / 5.0); 
    }

private:
    static const int NUM_ROWS = 16; 
    std::array<double, NUM_ROWS> rows_;
    int counter_;
    std::mt19937 gen_;
    std::uniform_real_distribution<double> dist_;
};

// 3 Eksen (X, Y, Z) için bağımsız gürültü yapısı
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
        // Bit kaydırma ile basit ve hızlı bir hash fonksiyonu
        return std::hash<int>()(k.x) ^ (std::hash<int>()(k.y) << 1) ^ (std::hash<int>()(k.z) << 2);
    }
};

// ======================================================================================
// 3. ANA SINIF: DroneDisturbanceManager
// ======================================================================================
class DroneDisturbanceManager {
public:
    DroneDisturbanceManager() : rate_(100.0), topic_check_interval_(1.0), drone_count_logged_(false) {
        
        // --- 3.1. Model Parametreleri ---
        double m_drone;
        if (nh_.getParam("/state_derivative_solver_node/mass", m_drone)) {
            ROS_INFO("Disturbance Manager: Mass loaded: %.4f kg", m_drone);
        } else {
            m_drone = 0.382; // Varsayılan kütle
            ROS_WARN("Disturbance Manager: Mass parameter not found, using default.");
        }

        double g = 9.81;
        double c_coup = 0.09; // Eşleşme katsayısı
        double r_lat = 0.05;   // Yanal etki katsayısı
        
        double thrust_total = m_drone * g;
        double thrust_per_rotor = thrust_total / 4.0;
        
        // Kuvvet katsayıları
        double k_z = thrust_per_rotor * c_coup; 
        double k_x = k_z * r_lat;            
        double k_y = k_z * r_lat;

        // Katsayı matrisi (Diagonal)
        k_matrix_ << k_x, 0.0, 0.0,
                     0.0, k_y, 0.0,
                     0.0, 0.0, k_z;

        // Downwash Model Parametreleri
        z0_ = 0.05;            
        sigma_ = 0.15;         
        turb_intensity_ = 0.70; 
        
        // --- 3.2. Dinamik Grid Boyutu (Sigma Kuralı) ---
        // 2*Sigma mesafesi, etkinin %99.99'unu kapsar. 
        max_effect_radius_ = (2.0 * sigma_) + 0.25; 
        cell_size_ = max_effect_radius_; 

        // --- 3.3. Başlangıç Ayarları ---
        // 20 Drone için offset ve orientation yer tutucuları
        for (int i = 1; i <= 20; ++i) { 
            std::string drone_name = "drone" + std::to_string(i);
            std::string ns = "/" + drone_name + "/";
            geometry_msgs::Vector3 offset;
            if (nh_.getParam(ns + "initial_x", offset.x)) {
                nh_.getParam(ns + "initial_y", offset.y);
                nh_.getParam(ns + "initial_z", offset.z);
            } else {
                offset.x = 0; offset.y = 0; offset.z = 0;
            }
            drone_initial_offsets_[drone_name] = offset;
            drone_orientations_[drone_name] = Eigen::Quaterniond::Identity(); // Varsayılan: Düz
        }

        grid_map_cache_.reserve(1000);
        last_topic_check_ = ros::Time::now();
    }

    void run() {
        while (ros::ok()) {
            checkNewTopics();   // Yeni drone'ları keşfet
            logDroneCount();    // Bilgi ver
            computeAndPublishDisturbancesOptimized(); // Ana hesaplama
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
    
    // Model Değişkenleri
    Eigen::Matrix3d k_matrix_;
    double z0_, sigma_, turb_intensity_;
    double max_effect_radius_, cell_size_;

    // Drone Verileri
    std::map<std::string, geometry_msgs::Vector3> drone_positions_;
    std::map<std::string, Eigen::Quaterniond> drone_orientations_;

    // ROS İletişimi
    std::map<std::string, ros::Subscriber> sub_pos_;
    std::map<std::string, ros::Subscriber> sub_orient_; 
    std::map<std::string, geometry_msgs::Vector3> drone_initial_offsets_;
    std::map<std::string, ros::Publisher> pub_total_;  
    std::map<std::string, ros::Publisher> pub_static_; 

    // Optimizasyon Yapıları
    std::unordered_map<GridKey, std::vector<std::string>, GridKeyHash> grid_map_cache_;
    std::map<std::string, PinkNoise3D> noise_generators_;

    // --- YARDIMCI FONKSİYONLAR ---
    
    // Grid Anahtarı Hesaplama
    GridKey getGridKey(const geometry_msgs::Vector3& pos) {
        return {
            static_cast<int>(std::floor(pos.x / cell_size_)),
            static_cast<int>(std::floor(pos.y / cell_size_)),
            static_cast<int>(std::floor(pos.z / cell_size_))
        };
    }

    // İlgili Topic'leri Bulma
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

    // Topic isminden drone ismini ayıklama (/drone1/actual_position -> drone1)
    std::string extractDroneName(const std::string& topic_name) {
        size_t first_slash = topic_name.find("/");
        size_t second_slash = topic_name.find("/", first_slash + 1);
        if (first_slash != std::string::npos && second_slash != std::string::npos) {
            return topic_name.substr(first_slash + 1, second_slash - first_slash - 1);
        }
        return topic_name; // Fallback
    }

    // --- CALLBACK FONKSİYONLARI ---

    void positionCallback(const geometry_msgs::Vector3::ConstPtr& msg, const std::string& topic_name) {
        std::string drone_name = extractDroneName(topic_name);
        geometry_msgs::Vector3 offset = drone_initial_offsets_[drone_name]; 
        
        geometry_msgs::Vector3 adjusted_position;
        adjusted_position.x = msg->x + offset.x;
        adjusted_position.y = msg->y + offset.y;
        adjusted_position.z = msg->z + offset.z;
        
        drone_positions_[drone_name] = adjusted_position;
    }

    void eulerCallback(const geometry_msgs::Vector3::ConstPtr& msg, const std::string& topic_name) {
        std::string drone_name = extractDroneName(topic_name);
        
        double roll = msg->x;
        double pitch = msg->y;
        double yaw = msg->z;

        // Euler -> Quaternion (ZYX Sırası: Yaw -> Pitch -> Roll)
        Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());

        Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;
        drone_orientations_[drone_name] = q;
    }

    // --- TOPIC KONTROLÜ ---
    void checkNewTopics() {
        if ((ros::Time::now() - last_topic_check_) > topic_check_interval_) {
            
            // 1. Pozisyon Topic'lerini Tara
            std::vector<std::string> pos_topics = getTopicsByType("/actual_position", "geometry_msgs/Vector3");
            for (const auto& topic : pos_topics) {
                if (sub_pos_.find(topic) == sub_pos_.end()) {
                    sub_pos_[topic] = nh_.subscribe<geometry_msgs::Vector3>(
                        topic, 10, boost::bind(&DroneDisturbanceManager::positionCallback, this, _1, topic));
                    
                    std::string drone_name = extractDroneName(topic);
                    
                    // Publisher'ları oluştur
                    pub_total_[drone_name] = nh_.advertise<geometry_msgs::Vector3>("/" + drone_name + "/disturbance_force", 10);
                    pub_static_[drone_name] = nh_.advertise<geometry_msgs::Vector3>("/" + drone_name + "/disturbance_static", 10);
                    
                    // Gürültü üretecini başlat
                    if (noise_generators_.find(drone_name) == noise_generators_.end()) {
                        noise_generators_[drone_name] = PinkNoise3D();
                    }
                }
            }

            // 2. Oryantasyon Topic'lerini Tara (reference_euler_angles)
            std::vector<std::string> orient_topics = getTopicsByType("actual_euler_angles", "geometry_msgs/Vector3");
            for (const auto& topic : orient_topics) {
                if (sub_orient_.find(topic) == sub_orient_.end()) {
                    sub_orient_[topic] = nh_.subscribe<geometry_msgs::Vector3>(
                        topic, 10, boost::bind(&DroneDisturbanceManager::eulerCallback, this, _1, topic));
                }
            }

            last_topic_check_ = ros::Time::now();
        }
    }

    void logDroneCount() {
        if (!drone_count_logged_ && !sub_pos_.empty()) {
            ROS_INFO("Disturbance Manager: Active drones connected: %zu", sub_pos_.size());
            drone_count_logged_ = true;
        }
    }

    // ======================================================================================
    // 4. HESAPLAMA DÖNGÜSÜ (OPTIMIZED)
    // ======================================================================================
    void computeAndPublishDisturbancesOptimized() {
        std::map<std::string, Eigen::Vector3d> acc_total_world;
        std::map<std::string, Eigen::Vector3d> acc_static_world;
        
        // --- 4.1. Spatial Hashing (Grid Doldurma) ---
        grid_map_cache_.clear(); 
        for (const auto& pair : drone_positions_) {
            // Başlangıçta kuvvetleri sıfırla
            acc_total_world[pair.first] = Eigen::Vector3d::Zero();
            acc_static_world[pair.first] = Eigen::Vector3d::Zero();
            
            // Grid'e yerleştir
            GridKey key = getGridKey(pair.second);
            grid_map_cache_[key].push_back(pair.first);
        }

        // Ön hesaplama: 2*sigma^2
        double two_sigma_sq = 2.0 * std::pow(sigma_, 2);

        // --- 4.2. Etkileşim Hesaplama ---
        for (const auto& pair : drone_positions_) {
            std::string top_name = pair.first; // Kaynak Drone (Rüzgarı Üreten)
            geometry_msgs::Vector3 top_pos_msg = pair.second;
            
            // Dönüşüm Matrislerini Hazırla
            Eigen::Vector3d P_top_world(top_pos_msg.x, top_pos_msg.y, top_pos_msg.z);
            Eigen::Quaterniond Q_top = drone_orientations_[top_name];
            
            Eigen::Matrix3d R_top_to_world = Q_top.toRotationMatrix(); // Body -> World
            Eigen::Matrix3d R_world_to_top = R_top_to_world.transpose(); // World -> Body

            GridKey current_key = getGridKey(top_pos_msg);

            // Sadece komşu grid hücrelerini tara (27 Hücre)
            for (int dx = -1; dx <= 1; ++dx) {
                for (int dy = -1; dy <= 1; ++dy) {
                    for (int dz = -1; dz <= 1; ++dz) {
                        GridKey neighbor_key = {current_key.x + dx, current_key.y + dy, current_key.z + dz};

                        auto it = grid_map_cache_.find(neighbor_key);
                        if (it != grid_map_cache_.end()) {
                            const auto& neighbors = it->second;
                            for (const auto& bot_name : neighbors) {
                                if (top_name == bot_name) continue; // Kendisiyle etkileşmez
                                
                                geometry_msgs::Vector3 bot_pos_msg = drone_positions_[bot_name]; // Hedef Drone (Etkilenen)
                                Eigen::Vector3d P_bot_world(bot_pos_msg.x, bot_pos_msg.y, bot_pos_msg.z);

                                // 1. Bağıl Vektör (World Frame)
                                Eigen::Vector3d P_rel_world = P_bot_world - P_top_world;

                                // 2. Bağıl Vektör (Body Frame - Üstteki drone'un bakış açısı)
                                Eigen::Vector3d P_rel_body = R_world_to_top * P_rel_world;

                                // 3. Kontrol: Hedef drone, Kaynak drone'un "altında" mı?
                                // Body Frame'de Z ekseni yukarı bakar. Alt taraf -Z yönüdür.
                                if (P_rel_body.z() < 0) { 
                                    
                                    // Yatay uzaklık (Body Frame XY düzlemi)
                                    double dist_xy_sq = (P_rel_body.x() * P_rel_body.x()) + (P_rel_body.y() * P_rel_body.y());
                                    
                                    // Exponential Cutoff Kontrolü (Ağır işlemlerden kaçınmak için)
                                    double exp_power_term = dist_xy_sq / two_sigma_sq;

                                    if (exp_power_term < 5.0) { // Eğer etki çok zayıf değilse
                                        
                                        double d_z = std::abs(P_rel_body.z()); 
                                        
                                        // Birim Yön Vektörü (Body Frame)
                                        // Bu vektör aşağı doğru (-Z) bakacaktır.
                                        Eigen::Vector3d n_vec_body = P_rel_body.normalized();

                                        double decay_factor = (1.0 / (std::pow(d_z, 2) + z0_)) * std::exp(-exp_power_term);
                                        
                                        // 4. Statik Kuvvet Hesabı (Body Frame)
                                        // DÜZELTME: (-) işareti kaldırıldı. n_vec zaten aşağı bakıyor.
                                        // Kuvvet rüzgar yönündedir (aşağı).
                                        Eigen::Vector3d f_static_body = k_matrix_ * n_vec_body * decay_factor;

                                        // 5. Pink Noise (Türbülans) Hesabı
                                        // Türbülans akışla birlikte oluşur, bu yüzden Body Frame'de eklenir.
                                        if (noise_generators_.find(bot_name) == noise_generators_.end()) {
                                            noise_generators_[bot_name] = PinkNoise3D();
                                        }

                                        Eigen::Vector3d noise_vector = noise_generators_[bot_name].getVector();
                                        Eigen::Vector3d f_static_abs = f_static_body.cwiseAbs();
                                        Eigen::Vector3d f_turb_body = f_static_abs.cwiseProduct(noise_vector * turb_intensity_);
                                        
                                        // Toplam Kuvvet (4 pervane etkisi)
                                        Eigen::Vector3d f_total_body = (f_static_body + f_turb_body) * 4.0;
                                        Eigen::Vector3d f_static_only_body = f_static_body * 4.0;

                                        // 6. World Frame'e Dönüş
                                        // Fizik motoru (Gazebo/ROS) için kuvveti dünya eksenine çeviriyoruz.
                                        Eigen::Vector3d f_total_world = R_top_to_world * f_total_body;
                                        Eigen::Vector3d f_static_world_vec = R_top_to_world * f_static_only_body;

                                        // Toplama Ekle
                                        acc_total_world[bot_name] += f_total_world;
                                        acc_static_world[bot_name] += f_static_world_vec;
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }

        // --- 4.3. Yayınlama (Publish) ---
        for (const auto& pair : acc_total_world) {
            std::string d_name = pair.first;
            
            // Toplam Kuvvet (Statik + Türbülans) -> World Frame
            geometry_msgs::Vector3 msg_total;
            msg_total.x = pair.second.x();
            msg_total.y = pair.second.y();
            msg_total.z = pair.second.z();
            pub_total_[d_name].publish(msg_total);

            // Sadece Statik (Görselleştirme için) -> World Frame
            geometry_msgs::Vector3 msg_static;
            msg_static.x = acc_static_world[d_name].x();
            msg_static.y = acc_static_world[d_name].y();
            msg_static.z = acc_static_world[d_name].z();
            pub_static_[d_name].publish(msg_static);
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