// Hakem 4 için: TrajectorySequencer'ı simülatörden bağımsız çalıştırır.
//  - Her İHA için 20 Hz örneklerde referans konum/hız/ivme/jerk/snap ve segment indisini yazar (samples.csv)
//  - Her yeniden planlamanın (yeni waypoint seti) ve her normal örneklemenin süresini ölçer (timing.csv)
//  - Yeniden planlama anında eski planın ivme ve jerk değerlerini yazar (replan.csv). Bu değerler tam
//    sıçramayı verir: yeni plan aynı tick'te eski planın üzerine yazıldığı için samples.csv'de görünmezler.
// Kullanım: traj_eval <spec_dosyası> <çıktı_klasörü> <tekrar>
// Spec biçimi:  drone <id>
//               traj <ad> <yöntem> <nokta_sayısı> [yeniden_planlama_segmenti]
//               x ... / y ... / z ... / t ...   (her biri tek satır)
#include "multi_quadcopter_control/sim/trajectory_sequencer.h"

#include <chrono>
#include <fstream>
#include <iostream>
#include <map>
#include <sstream>

using namespace mqc_sim;

int main(int argc, char** argv) {
    if (argc < 4) { std::cerr << "usage: traj_eval spec out_dir repeats\n"; return 1; }
    std::ifstream in(argv[1]);
    std::string out_dir = argv[2];
    const int repeats = std::stoi(argv[3]);

    std::map<int, std::vector<TrajectorySpec>> drones;
    std::string line; int cur = -1;
    while (std::getline(in, line)) {
        std::istringstream ss(line); std::string key; ss >> key;
        if (key == "drone") { ss >> cur; }
        else if (key == "traj") {
            TrajectorySpec s; int n; ss >> s.name >> s.method >> n; s.ros_rate = 20.0;
            int sw; if (ss >> sw) s.switch_segment = sw;
            auto read = [&](std::vector<double>& v) {
                std::getline(in, line); std::istringstream ls(line); std::string k; ls >> k; double d;
                while (ls >> d) v.push_back(d);
            };
            read(s.points_x); read(s.points_y); read(s.points_z); read(s.times);
            drones[cur].push_back(s);
        }
    }

    std::ofstream samples(out_dir + "/samples.csv"), timing(out_dir + "/timing.csv");
    samples << "drone,k,t,px,py,pz,vx,vy,vz,ax,ay,az,jx,jy,jz,sx,sy,sz,seg,traj\n";
    timing << "drone,rep,k,planning,method,ns\n";
    std::ofstream replan(out_dir + "/replan.csv");
    replan << "drone,traj,k,t,ax_old,ay_old,az_old,jx_old,jy_old,jz_old,sx_old,sy_old,sz_old\n";
    for (auto& kv : drones) {
        std::vector<int> replan_ticks;  // tekrar 0'daki yeniden planlama tick'leri (ilk plan hariç)
        for (int rep = 0; rep < repeats; ++rep) {
            TrajectorySequencer seq(kv.second, true, 0.05, 1.0);
            std::string last = "";
            for (int k = 0; k < 100000 && !seq.finished(); ++k) {
                auto t0 = std::chrono::steady_clock::now();
                seq.tick();
                auto t1 = std::chrono::steady_clock::now();
                const bool planning = (seq.currentName() != last);
                if (rep == 0 && planning && k > 0) replan_ticks.push_back(k);
                std::string method = "";
                for (const auto& s : kv.second) if (s.name == seq.currentName()) method = s.method;
                timing << kv.first << ',' << rep << ',' << k << ',' << planning << ',' << method << ','
                       << std::chrono::duration_cast<std::chrono::nanoseconds>(t1 - t0).count() << '\n';
                last = seq.currentName();
                if (rep == 0 && !seq.finished()) {
                    const auto &p = seq.position(), &v = seq.velocity(), &a = seq.acceleration(), &j = seq.jerk(),
                               &sn = seq.snap();
                    samples << kv.first << ',' << k << ',' << k * TrajectorySequencer::kSampleStep << ','
                            << p.x << ',' << p.y << ',' << p.z << ',' << v.x << ',' << v.y << ',' << v.z << ','
                            << a.x << ',' << a.y << ',' << a.z << ',' << j.x << ',' << j.y << ',' << j.z << ','
                            << sn.x << ',' << sn.y << ',' << sn.z << ',' << seq.segment() << ',' << seq.currentName() << '\n';
                }
            }
        }
        for (size_t p = 0; p < replan_ticks.size(); ++p) {
            std::vector<TrajectorySpec> head(kv.second.begin(), kv.second.begin() + p + 1);
            TrajectorySequencer old(head, true, 0.05, 1.0);
            for (int k = 0; k <= replan_ticks[p]; ++k) old.tick();
            const auto &a = old.acceleration(), &j = old.jerk(), &sn = old.snap();
            replan << kv.first << ',' << kv.second[p + 1].name << ',' << replan_ticks[p] << ','
                   << replan_ticks[p] * TrajectorySequencer::kSampleStep << ',' << a.x << ',' << a.y << ',' << a.z << ','
                   << j.x << ',' << j.y << ',' << j.z << ',' << sn.x << ',' << sn.y << ',' << sn.z << '\n';
        }
    }
    return 0;
}
