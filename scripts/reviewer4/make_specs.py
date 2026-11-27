# traj_eval için girdi dosyaları: her İHA'nın aktif yörüngeleri (noktalar, zamanlar, yöntem).
# spec_hybrid.txt yapılandırmadaki yöntemleri kullanır; diğerlerinde tüm segmentler tek yönteme çevrilir ve
# yeniden planlama segmenti hibritteki gibi kalır (aynı anlarda yeniden planlama, aynı süre). Aynı dönüşümle
# all_trajectories_<yöntem>.yaml dosyaları da yazılır (sequential_sim.launch trajectories_file argümanı için).
# Kullanım: python3 make_specs.py <config_klasörü> <çıktı_klasörü>
import re, sys
SWITCH = {'cubic_spline': 3, 'natural_cubic_spline': 5, 'minimum_jerk': 5, 'minimum_snap': 5}   # trajectory_sequencer.cpp
CFG, OUT = sys.argv[1], sys.argv[2]
allt = open(f'{CFG}/all_trajectories.yaml').read()

def block(nm):
    b = re.search(r'\n\s*' + re.escape(nm) + r':\s*\n(.*?)method:\s*"([^"]+)"', allt, re.S)
    arr = lambda k: re.search(k + r':\s*\[([^\]]*)\]', b.group(1)).group(1).replace(',', ' ')
    return dict(x=arr('points_x'), y=arr('points_y'), z=arr('points_z'), t=arr('times'), method=b.group(2))

for variant in ('hybrid', 'cubic_spline', 'minimum_jerk', 'minimum_snap'):
    with open(f'{OUT}/spec_{variant}.txt', 'w') as f:
        for d in range(1, 6):
            mgr = open(f'{CFG}/trajectory_manager_drone{d}.yaml').read()
            names = re.findall(r'"([^"]+)"', re.search(r'active_trajectories:\s*\[(.*?)\]', mgr, re.S).group(1))
            f.write(f'drone {d}\n')
            for nm in names:
                b = block(nm); m = b['method'] if variant == 'hybrid' else variant
                f.write(f'traj {nm} {m} {len(b["x"].split())} {SWITCH[b["method"]]}\nx {b["x"]}\ny {b["y"]}\nz {b["z"]}\nt {b["t"]}\n')
    print('yazıldı:', f'{OUT}/spec_{variant}.txt')
    if variant != 'hybrid':
        y = re.sub(r'^(\s*)method:\s*"([a-z_]+)"',
                   lambda m: f'{m.group(1)}method: "{variant}"\n{m.group(1)}switch_segment: {SWITCH[m.group(2)]}',
                   allt, flags=re.M)
        open(f'{OUT}/all_trajectories_{variant}.yaml', 'w').write(y)
