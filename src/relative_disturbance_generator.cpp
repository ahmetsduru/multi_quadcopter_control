#include <ros/ros.h>
#include <ros/master.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <map>
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
    // seed < 0  -> her çalıştırmada farklı (std::random_device)
    // seed >= 0 -> tekrarlanabilir (deterministik) gürültü dizisi
    explicit PinkNoiseGenerator(long long seed = -1) : counter_(0) {
        if (seed < 0) {
            std::random_device rd;
            gen_ = std::mt19937(rd());
        } else {
            gen_ = std::mt19937(static_cast<std::mt19937::result_type>(seed));
        }
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

        // Ölçekleme: 16 adet U(-1,1) toplamı / 5 -> std ~0.46, uç değerler +/-3.2
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
    PinkNoise3D() = default;
    // Her eksen için farklı ama base_seed'den türetilmiş tohum
    explicit PinkNoise3D(long long base_seed)
        : px(base_seed < 0 ? -1 : base_seed * 3 + 0),
          py(base_seed < 0 ? -1 : base_seed * 3 + 1),
          pz(base_seed < 0 ? -1 : base_seed * 3 + 2) {}
    Eigen::Vector3d getVector() {
        return Eigen::Vector3d(px.next(), py.next(), pz.next());
    }
};

// ======================================================================================
// 3. ANA SINIF: DroneDisturbanceManager
// ======================================================================================
class DroneDisturbanceManager {
public:
    DroneDisturbanceManager() : topic_check_interval_(1.0), drone_count_logged_(false) {
        ros::NodeHandle pnh("~");

        // --- 3.1. Model Parametreleri (hepsi ~özel parametre olarak değiştirilebilir) ---
        // Kuvvet katsayısı matrisi (k_matrix) her drone için KENDİ kütlesinden
        // hesaplanır (bkz. getKMatrix). Kütle /<drone>/state_derivative_solver_node/mass
        // parametresinden okunur; bulunamazsa default_mass kullanılır.
        pnh.param("default_mass", default_mass_, 0.382);
        pnh.param("c_coup", c_coup_, 0.065);  // Eşleşme katsayısı (Tablo S2)
        pnh.param("r_lat", r_lat_, 0.05);    // Yanal etki katsayısı

        // Downwash Model Parametreleri
        // Azalma terimi: 1 / (dz^2 + z0^2). z0 = sqrt(0.05) ~ 0.2236 m seçilerek
        // z0^2 = 0.05 m^2 olur; yani kuvvet büyüklükleri eski modelle (1 / (dz^2 + 0.05)) aynıdır.
        pnh.param("z0", z0_, std::sqrt(0.05));
        pnh.param("sigma", sigma_, 0.15);
        pnh.param("turb_intensity", turb_intensity_, 0.30);

        // Döngü frekansı (Hz). Varsayılan 500 Hz = eom düğümünün frekansı (dt = 0.002),
        // böylece bozucu kuvvet eom'un her adımında güncellenir.
        pnh.param("rate", rate_hz_, 500.0);

        // Bu süreden (s) uzun süredir konum mesajı gelmeyen drone hesaba katılmaz.
        pnh.param("position_timeout", position_timeout_, 0.5);

        ROS_INFO("Disturbance Manager: c_coup=%.4f r_lat=%.4f z0=%.4f (z0^2=%.4f) sigma=%.4f "
                 "turb_intensity=%.3f rate=%.1f Hz position_timeout=%.2f s",
                 c_coup_, r_lat_, z0_, z0_ * z0_, sigma_, turb_intensity_, rate_hz_, position_timeout_);

        // --- 3.2. Etki Menzili ---
        // Yatay: exp(-r^2 / 2sigma^2) < exp(-5) olduğunda etki ihmal edilir (r > ~3.16 sigma).
        // Dikey: varsayılan olarak SINIR YOK (1/(dz^2+z0^2) yumuşak şekilde azalır, kesme
        // kaynaklı kuvvet sıçraması olmaz). ~max_vertical_range > 0 verilirse
        // üstteki drone'un gövde ekseninde |dz| > max_vertical_range olan etkileşimler ihmal edilir.
        pnh.param("max_vertical_range", max_vertical_range_, -1.0);
        if (max_vertical_range_ > 0.0) {
            ROS_INFO("Disturbance Manager: max_vertical_range = %.3f m", max_vertical_range_);
        } else {
            ROS_INFO("Disturbance Manager: max_vertical_range = unlimited");
        }

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

        // --- 3.4. Gürültü Tohumu (Tekrarlanabilirlik) ---
        // ~noise_seed >= 0 ise türbülans her çalıştırmada aynı dizi ile üretilir.
        // -1 verilirse eski davranış (rastgele tohum) kullanılır.
        int seed_param = 42;
        pnh.param("noise_seed", seed_param, 42);
        noise_seed_ = seed_param;
        ROS_INFO("Disturbance Manager: noise_seed = %lld%s", noise_seed_,
                 noise_seed_ < 0 ? " (random)" : " (deterministic)");

        // İlk döngüde topic taraması hemen yapılsın (1 s beklemeden)
        last_topic_check_ = ros::Time::now();
        first_topic_check_ = true;
    }

    void run() {
        ros::Rate rate(rate_hz_);
        while (ros::ok()) {
            checkNewTopics();   // Yeni drone'ları keşfet
            logDroneCount();    // Bilgi ver
            computeAndPublishDisturbancesOptimized(); // Ana hesaplama
            ros::spinOnce();
            rate.sleep();
        }
    }

private:
    ros::NodeHandle nh_;
    double rate_hz_;
    double position_timeout_;
    ros::Time last_topic_check_;
    ros::Duration topic_check_interval_;
    bool first_topic_check_ = true;
    bool drone_count_logged_;
    
    // Model Değişkenleri
    double default_mass_, c_coup_, r_lat_;
    double z0_, sigma_, turb_intensity_;
    double max_vertical_range_;
    std::map<std::string, Eigen::Matrix3d> k_matrices_; // Kaynak drone -> katsayı matrisi

    // Drone Verileri
    std::map<std::string, geometry_msgs::Vector3> drone_positions_;
    std::map<std::string, ros::Time> drone_last_update_; // Son konum mesajı zamanı
    std::map<std::string, Eigen::Quaterniond> drone_orientations_;

    // ROS İletişimi
    std::map<std::string, ros::Subscriber> sub_pos_;
    std::map<std::string, ros::Subscriber> sub_orient_; 
    std::map<std::string, geometry_msgs::Vector3> drone_initial_offsets_;
    std::map<std::string, ros::Publisher> pub_total_;  

    // Gürültü Üreteçleri
    std::map<std::string, PinkNoise3D> noise_generators_;
    long long noise_seed_ = 42;

    // Drone ismine bağlı tohum: keşfedilme sırasından bağımsız olsun diye
    // (master topic listesi her çalıştırmada farklı sırada gelebilir).
    PinkNoise3D makeNoiseFor(const std::string& drone_name) {
        if (noise_seed_ < 0) return PinkNoise3D(-1);
        long long id = 0;
        for (char c : drone_name) id = id * 131 + static_cast<unsigned char>(c);
        id &= 0x3FFFFFFF;
        return PinkNoise3D((noise_seed_ * 1000003LL + id) & 0x3FFFFFFFFFFFLL);
    }

    // --- YARDIMCI FONKSİYONLAR ---
    
    // Kaynak drone'un (rüzgarı üreten) kendi kütlesine göre katsayı matrisi
    const Eigen::Matrix3d& getKMatrix(const std::string& drone_name) {
        auto it = k_matrices_.find(drone_name);
        if (it != k_matrices_.end()) return it->second;

        double m_drone;
        std::string param = "/" + drone_name + "/state_derivative_solver_node/mass";
        if (nh_.getParam(param, m_drone)) {
            ROS_INFO("Disturbance Manager: %s mass loaded: %.4f kg", drone_name.c_str(), m_drone);
        } else {
            m_drone = default_mass_;
            ROS_WARN("Disturbance Manager: %s not found, using default mass %.4f kg",
                     param.c_str(), m_drone);
        }

        const double g = 9.81;
        double thrust_per_rotor = m_drone * g / 4.0; // Hover itkisi / rotor
        double k_z = thrust_per_rotor * c_coup_;
        double k_x = k_z * r_lat_;
        double k_y = k_z * r_lat_;

        Eigen::Matrix3d k;
        k << k_x, 0.0, 0.0,
             0.0, k_y, 0.0,
             0.0, 0.0, k_z;
        return k_matrices_[drone_name] = k;
    }

    // İlgili Topic'leri Bulma: isim "/<ns>/<suffix>" biçiminde ve tam olarak suffix ile bitmeli
    // (örn. "/drone1/actual_position" eşleşir, "/drone1/actual_position_filtered" eşleşmez).
    std::vector<std::string> getTopicsByType(const std::string& suffix, const std::string& type) {
        ros::master::V_TopicInfo master_topics;
        ros::master::getTopics(master_topics);
        std::vector<std::string> found_topics;
        for (const auto& topic : master_topics) {
            const std::string& name = topic.name;
            bool ends_with = name.size() > suffix.size() &&
                             name.compare(name.size() - suffix.size(), suffix.size(), suffix) == 0;
            if (ends_with && topic.datatype == type) {
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
        drone_last_update_[drone_name] = ros::Time::now();
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
        if (first_topic_check_ || (ros::Time::now() - last_topic_check_) > topic_check_interval_) {
            first_topic_check_ = false;
            
            // 1. Pozisyon Topic'lerini Tara
            std::vector<std::string> pos_topics = getTopicsByType("/actual_position", "geometry_msgs/Vector3");
            for (const auto& topic : pos_topics) {
                if (sub_pos_.find(topic) == sub_pos_.end()) {
                    sub_pos_[topic] = nh_.subscribe<geometry_msgs::Vector3>(
                        topic, 10, boost::bind(&DroneDisturbanceManager::positionCallback, this, _1, topic));
                    
                    std::string drone_name = extractDroneName(topic);
                    
                    // Publisher'ları oluştur
                    pub_total_[drone_name] = nh_.advertise<geometry_msgs::Vector3>("/" + drone_name + "/disturbance_force", 10);
                    
                    // Gürültü üretecini başlat
                    if (noise_generators_.find(drone_name) == noise_generators_.end()) {
                        noise_generators_[drone_name] = makeNoiseFor(drone_name);
                    }
                }
            }

            // 2. Oryantasyon Topic'lerini Tara (reference_euler_angles)
            std::vector<std::string> orient_topics = getTopicsByType("/actual_euler_angles", "geometry_msgs/Vector3");
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
    // 4. HESAPLAMA DÖNGÜSÜ
    // ======================================================================================
    // Not: Drone sayısı az olduğu için (N <= 20) tüm çiftler doğrudan taranır (N*(N-1)).
    // Önceki ızgara (spatial hashing) yöntemi dikey menzili hücre sınırlarına göre
    // rastgele kesiyor ve kuvvette sıçramalara yol açıyordu.
    void computeAndPublishDisturbancesOptimized() {
        std::map<std::string, Eigen::Vector3d> acc_total_world;

        // --- 4.1. Başlangıç + Adım Başına Tek Gürültü Örneği ---
        // Her drone'un gürültü üreteci her döngü adımında TAM BİR KEZ ilerler.
        // Böylece pembe gürültünün zaman ölçeği etkileşim sayısından bağımsız olur.
        std::map<std::string, Eigen::Vector3d> noise_now;
        std::map<std::string, bool> active; // Konumu güncel olan drone'lar
        const ros::Time now = ros::Time::now();
        for (const auto& pair : drone_positions_) {
            acc_total_world[pair.first] = Eigen::Vector3d::Zero();
            active[pair.first] = position_timeout_ <= 0.0 ||
                !((now - drone_last_update_[pair.first]) > ros::Duration(position_timeout_));

            auto it = noise_generators_.find(pair.first);
            if (it == noise_generators_.end()) {
                it = noise_generators_.emplace(pair.first, makeNoiseFor(pair.first)).first;
            }
            noise_now[pair.first] = it->second.getVector();
        }

        // Ön hesaplamalar
        const double two_sigma_sq = 2.0 * sigma_ * sigma_;
        const double z0_sq = z0_ * z0_;

        // --- 4.2. Etkileşim Hesaplama ---
        for (const auto& top : drone_positions_) {
            const std::string& top_name = top.first; // Kaynak Drone (Rüzgarı Üreten)
            if (!active[top_name]) continue;          // Eski (bayat) konum
            Eigen::Vector3d P_top_world(top.second.x, top.second.y, top.second.z);

            // Dönüşüm Matrisleri
            Eigen::Matrix3d R_top_to_world = drone_orientations_[top_name].toRotationMatrix(); // Body -> World
            Eigen::Matrix3d R_world_to_top = R_top_to_world.transpose();                      // World -> Body
            const Eigen::Matrix3d& k_matrix = getKMatrix(top_name);

            for (const auto& bot : drone_positions_) {
                const std::string& bot_name = bot.first; // Hedef Drone (Etkilenen)
                if (top_name == bot_name) continue;      // Kendisiyle etkileşmez
                if (!active[bot_name]) continue;          // Eski (bayat) konum

                Eigen::Vector3d P_bot_world(bot.second.x, bot.second.y, bot.second.z);

                // 1. Bağıl Vektör (World -> Üstteki drone'un Body Frame'i)
                Eigen::Vector3d P_rel_body = R_world_to_top * (P_bot_world - P_top_world);

                // 2. Hedef drone, kaynak drone'un "altında" mı? (Body Z yukarı, alt taraf -Z)
                if (P_rel_body.z() >= 0) continue;

                double d_z = -P_rel_body.z();
                if (max_vertical_range_ > 0.0 && d_z > max_vertical_range_) continue; // Dikey menzil dışında

                // 3. Yatay mesafe ve Gauss kesme kontrolü
                double dist_xy_sq = P_rel_body.x() * P_rel_body.x() + P_rel_body.y() * P_rel_body.y();
                double exp_power_term = dist_xy_sq / two_sigma_sq;
                if (exp_power_term >= 5.0) continue; // Etki ihmal edilebilir (r > ~3.16 sigma)

                // Birim Yön Vektörü (Body Frame, aşağı -Z yönlü)
                Eigen::Vector3d n_vec_body = P_rel_body.normalized();

                // Bozulma çarpanı: 1 / (d_z^2 + z0^2) * exp(-r^2 / 2sigma^2)
                double decay_factor = (1.0 / (d_z * d_z + z0_sq)) * std::exp(-exp_power_term);

                // 4. Statik Kuvvet (Body Frame, rüzgar yönünde - aşağı)
                Eigen::Vector3d f_static_body = k_matrix * n_vec_body * decay_factor;

                // 5. Pink Noise (Türbülans) - Body Frame'de eklenir
                Eigen::Vector3d f_turb_body =
                    f_static_body.cwiseAbs().cwiseProduct(noise_now[bot_name] * turb_intensity_);

                // Toplam Kuvvet (4 pervane etkisi)
                Eigen::Vector3d f_total_body = (f_static_body + f_turb_body) * 4.0;

                // 6. World Frame'e Dönüş ve Toplama
                acc_total_world[bot_name] += R_top_to_world * f_total_body;
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