# Ek malzeme Fig. S7 (Denklem (8)'in ölçüm verisine uyumu) ve Fig. S8 (iki kaynağın iz kesişme sınırı),
# diğer figürlerle aynı yazı tipi, kalınlık ve çizgi genişlikleriyle.
# Veri: scripts/data/figS7_S8_digitized.json (önceki figürlerin vektör çıktılarından sayısallaştırıldı).
# Kullanım: python3 scripts/fig_S7_S8.py [çıktı_klasörü]   (varsayılan scripts/Figures)
import os, sys, json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.legend_handler import HandlerTuple

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = sys.argv[1] if len(sys.argv) > 1 else os.path.join(HERE, 'Figures')
os.makedirs(OUT, exist_ok=True)
D = json.load(open(os.path.join(HERE, 'data', 'figS7_S8_digitized.json')))
plt.rcParams.update({
    'font.family': 'Liberation Serif', 'font.weight': 'bold', 'axes.labelweight': 'bold',
    'mathtext.fontset': 'custom', 'mathtext.rm': 'Liberation Serif', 'mathtext.it': 'Liberation Serif:italic',
    'mathtext.bf': 'Liberation Serif:bold', 'mathtext.sf': 'Liberation Serif:weight=normal',
    'xtick.direction': 'in', 'ytick.direction': 'in', 'xtick.top': True, 'ytick.right': True,
    'svg.fonttype': 'path', 'axes.linewidth': 2, 'xtick.major.width': 1.5, 'ytick.major.width': 1.5,
    'xtick.major.size': 6, 'ytick.major.size': 6,
})
PX = 1 / 96.0
BLUE, ORANGE, GREY = [0.1, 0.45, 0.9], [0.93, 0.4, 0.1], [0.3, 0.3, 0.3]
FSL, FST, FSG, FSA = 18, 16, 16, 15          # eksen adı, eksen değerleri, lejant, açıklama yazıları
L_OURS = 0.106                               # m, motorlar arası mesafenin yarısı (ek malzeme, Ek D)

def save(fig, name):
    fig.savefig(os.path.join(OUT, name + '.svg'))
    if os.environ.get('PNG_CHECK'):
        os.makedirs(os.environ['PNG_CHECK'], exist_ok=True); fig.savefig(os.path.join(os.environ['PNG_CHECK'], name + '.png'), dpi=80)
    plt.close(fig); print('yazıldı:', os.path.join(OUT, name + '.svg'))

def frame(ax):
    ax.tick_params(labelsize=FST)
    for sp in ax.spines.values(): sp.set_linewidth(2)

def errbar(ax, pts, marker, ms):
    x = np.array([p['x'] for p in pts]); y = np.array([p['y'] for p in pts])
    lo = np.array([p['lo'] for p in pts]); hi = np.array([p['hi'] for p in pts])
    return ax.errorbar(x, y, yerr=[y - lo, hi - y], fmt=marker, ms=ms, mfc=ORANGE, mec='white', mew=1.2, ecolor=ORANGE,
                       elinewidth=2, capsize=4, capthick=2, zorder=4)

# ======================= Fig. S7 =======================
fig, axs = plt.subplots(1, 2, figsize=(1200 * PX, 560 * PX), constrained_layout=True)
YL = 'Force on lower vehicle /\nsource thrust'
ax = axs[0]
m = D['S7a_model']; hm, = ax.plot(m['x'], m['y'], '-', color=BLUE, lw=3, zorder=3)
he = errbar(ax, D['S7a'], 'o', 9)
xs = 0.5 / L_OURS                                         # simülasyondaki dikey mesafe 0.5 m
ax.axvline(xs, ls='--', color=GREY, lw=2, zorder=2)
ax.text(xs + 0.6, 0.585, 'simulation\nΔz = 0.5 m', ha='left', va='top', fontsize=FSA, color=GREY)
ax.set_xlim(0, 35); ax.set_ylim(-0.03, 0.62)
ax.set_xlabel('Vertical separation, Δz/ℓ (Δx = 0)', fontsize=FSL); ax.set_ylabel(YL, fontsize=FSL)
ax = axs[1]
hb = []
for key, pts, mk, ls in [('S7b_model_dz4.0', 'S7b_circle', 'o', '-'), ('S7b_model_dz5.2', 'S7b_square', 's', '--')]:
    m = D[key]; hl, = ax.plot(m['x'], m['y'], ls, color=BLUE, lw=3, zorder=3)
    errbar(ax, D[pts], mk, 9 if mk == 'o' else 8)
    hb.append((Line2D([], [], ls='none', marker=mk, ms=9, mfc=ORANGE, mec='white', mew=1.2), hl))
ax.legend(hb, ['Δz/ℓ = 4.0 (0.42 m)', 'Δz/ℓ = 5.2 (0.55 m)'], handler_map={tuple: HandlerTuple(ndivide=None, pad=0.6)},
          handlelength=3.2, loc='upper right', fontsize=FSG, frameon=False)
ax.set_xlim(0, 10); ax.set_ylim(-0.03, 0.62)
ax.set_xlabel('Horizontal separation, Δx/ℓ', fontsize=FSL); ax.set_ylabel(YL, fontsize=FSL)
for a, tag in zip(axs, 'ab'):
    frame(a); a.text(0.02, 0.98, f'({tag})', transform=a.transAxes, ha='left', va='top', fontsize=FSL)
fig.legend([hm, he], [r'Eq. (8): C$_\mathbf{coup}$ = 0.34 m$^2$, z$_\mathbf{0}$ = 0.8 m, $\mathsf{σ}$ = 0.22 m',
                      'Measured [35]: mean ± SD'], loc='outside upper center', ncol=2, fontsize=FSG, frameon=False)
save(fig, 'FigS7')

# ======================= Fig. S8 =======================
fig, ax = plt.subplots(figsize=(900 * PX, 640 * PX), constrained_layout=True)
lim = D['S8_limit']; h = np.array(lim['h']); s = np.array(lim['s'])
ax.fill_between(h, 0, s, color=[1, 0.9, 0.85], lw=0, zorder=0)
hl, = ax.plot(h, s, '-', color=ORANGE, lw=3, zorder=3)
hp, = ax.plot([p['h'] for p in D['S8']], [p['s'] for p in D['S8']], 'o', ms=9, mfc=ORANGE, mec='white', mew=1.2, zorder=4)
hc, = ax.plot([0.5, 0.5], [0.58, 0.82], '-', color=BLUE, lw=6, solid_capstyle='round', zorder=5)
ax.text(0.34, 1.12, 'wakes separate:\nsuperposition exact', ha='left', va='top', fontsize=FSA)
ax.text(0.34, 0.18, 'wakes overlap', ha='left', va='center', fontsize=FSA, color=[0.7, 0.25, 0.05])
ax.legend([hl, hp, hc], ['overlap limit, s = 0.31 + 0.36h', 'measured wake edge [39]', 'case (Q4, Q5 above Q2)'],
          loc='lower right', fontsize=FSG - 1, frameon=False)
ax.set_xlim(0.3, 1.3); ax.set_ylim(0, 1.2)
ax.set_xlabel('Distance below the sources, h (m)', fontsize=FSL)
ax.set_ylabel('Horizontal spacing of the sources, s (m)', fontsize=FSL)
frame(ax)
save(fig, 'FigS8')
