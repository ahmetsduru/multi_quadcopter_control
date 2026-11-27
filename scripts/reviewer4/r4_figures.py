# Hakem 4 (yörünge kısmı) için üç figür: fizibilite, segment türü karşılaştırması, hesaplama süresi.
# Kullanım: python3 r4_figures.py <log_kök> <traj_eval_kök> <çıktı_klasörü>   (run_r4_sims.sh ile üretilen klasörler)
#   <log_kök>/r4_scale_<μ>           : PD+FF, downwash açık, hibrit yörünge, trajectory_time_scale = μ
#   <log_kök>/r4_<v>                 : PD+FF, downwash açık, tüm segmentler <v> yöntemiyle (v = hybrid, cubic_spline,
#                                       minimum_jerk, minimum_snap); yeniden planlama anları hibritle aynı
#   <traj_eval_kök>/spec_<v>.txt      : traj_eval girdisi (make_specs.py)
#   <traj_eval_kök>/<v>/samples.csv   : 20 Hz referans örnekleri (ivme, jerk, snap, segment indisi)
#   <traj_eval_kök>/<v>/replan.csv    : yeniden planlama anında eski planın ivme / jerk / snap değeri
#   <traj_eval_kök>/<v>/timing.csv    : her tick'in süresi (planning=1: yeniden planlama)
# İHA seçimi: R4_DRONE ortam değişkeni (varsayılan 2, downwash'tan en çok etkilenen İHA).
import sys, os
import numpy as np, pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.patches import FancyArrow

LOG, EV, OUT = sys.argv[1], sys.argv[2], sys.argv[3]
os.makedirs(OUT, exist_ok=True)
plt.rcParams.update({
    'font.family': 'Liberation Serif', 'font.weight': 'bold', 'axes.labelweight': 'bold',
    'axes.titleweight': 'bold', 'mathtext.fontset': 'custom', 'mathtext.rm': 'Liberation Serif',
    'mathtext.it': 'Liberation Serif:italic', 'mathtext.bf': 'Liberation Serif:bold',
    'mathtext.sf': 'Liberation Serif:weight=normal',      # \mathsf: dik ve normal kalınlıkta (açı sembolleri)
    'xtick.direction': 'in', 'ytick.direction': 'in', 'xtick.top': True, 'ytick.right': True,
    'svg.fonttype': 'path', 'axes.linewidth': 2, 'xtick.major.width': 1.5, 'ytick.major.width': 1.5,
    'xtick.major.size': 6, 'ytick.major.size': 6, 'legend.handlelength': 2.0,
})
PX = 1 / 96.0
DRONE = np.array([[0, 0.4470, 0.7410], [0.8500, 0.3250, 0.0980], [0.9290, 0.6940, 0.1250],
                  [0.4940, 0.1840, 0.5560], [0.4660, 0.6740, 0.1880]])
C_PHASE = [[1, .88, .88], [.85, 1, 1], [.85, 1, .85], [1, 1, .9]]
T_SWITCH = [8.05, 21.05, 32.55]
PHASE = list(zip([0] + T_SWITCH, T_SWITCH + [60]))
PHASE_NAME = ['Cubic', 'Min. snap', 'Min. snap', 'Min. jerk']
LINE_W, FS = 3, 20
MASS = [0.382, 0.45, 0.45, 0.45, 0.45]
T_MAX = [15.905, 17.848, 17.848, 17.848, 17.848]       # max_thrust (params_1 / params_2): toplam itki komutunun sınırı
ANG_LIM = 0.6                                           # controllers.cpp: angle_limit (rad)
DT = 0.05                                               # referans örnekleme aralığı
VAR = ['hybrid', 'cubic_spline', 'minimum_jerk', 'minimum_snap']
VLAB = {'hybrid': 'Hybrid (H)', 'cubic_spline': 'Cubic spline (CS)', 'minimum_jerk': 'Minimum jerk (MJ)',
        'minimum_snap': 'Minimum snap (MS)'}
VABB = {'hybrid': 'H', 'cubic_spline': 'CS', 'minimum_jerk': 'MJ', 'minimum_snap': 'MS'}
VCOL = {'hybrid': [.2, .2, .2], 'cubic_spline': [.1, .45, .9], 'minimum_jerk': [.93, .4, .1], 'minimum_snap': [.05, .65, .45]}
VLW = {'hybrid': 3.0, 'cubic_spline': 2.0, 'minimum_jerk': 2.0, 'minimum_snap': 2.0}
VZ = {'hybrid': 6, 'cubic_spline': 3, 'minimum_jerk': 4, 'minimum_snap': 5}
SEL = int(os.environ.get('R4_DRONE', '2'))              # R2 ve R3'te gösterilen İHA

def simlog(base, d):
    f = f'{base}/drone{d}_log.txt'
    h = [s.strip() for s in open(f).readline().split(',')]
    A = np.loadtxt(f, delimiter=',', skiprows=1)
    T = {k: A[:, j] for j, k in enumerate(h)}; T['t'] = T['time'] - T['time'][0]
    return T

def save(fig, name):
    fig.savefig(f'{OUT}/{name}.svg')
    if os.environ.get('PNG_CHECK'):
        os.makedirs(os.environ['PNG_CHECK'], exist_ok=True); fig.savefig(f"{os.environ['PNG_CHECK']}/{name}.png", dpi=80)
    plt.close(fig); print('yazıldı:', name)

def frame(ax, fs=FS):
    ax.tick_params(labelsize=fs)
    for sp in ax.spines.values(): sp.set_linewidth(2)

FSL, FST, FSB = 17, 15, 13                              # R2 ve R3 yazı boyutları
ONLY = os.environ.get('R4_ONLY', 'R1,R2,R3').split(',')  # örn. R4_ONLY=R3: yalnızca hesaplama süresi

if 'R1' in ONLY:
    # ======================= Fig. R1: fizibilite =======================
    D = [simlog(f'{LOG}/r4_scale_1.0', d) for d in range(1, 6)]
    fig = plt.figure(figsize=(1000 * PX, 1220 * PX)); H = 1220
    left, w = 0.12, 0.85
    pos = []                                                # üç zaman paneli + μ taraması
    y = 1 - 70 / H
    for hpx, gap in [(200, 62), (200, 62), (200, 88), (250, 0)]:     # her panelde eksen adı (editör isteği)
        pos.append([left, y - hpx / H, w, hpx / H]); y -= (hpx + gap) / H
    axs = [fig.add_axes(p) for p in pos]
    hs = []
    for a, (ax, key) in enumerate(zip(axs[:3], ['thr', 'a_eul_ang.x', 'a_eul_ang.y'])):
        for (t0, t1), c in zip(PHASE, C_PHASE): ax.axvspan(t0, t1, color=c, zorder=0, lw=0)
        for i, T in enumerate(D):
            v = T['r_thr'] / T_MAX[i] * 100 if key == 'thr' else T[key]
            h, = ax.plot(T['t'][::5], v[::5], color=DRONE[i], lw=LINE_W if key != 'thr' else 2, zorder=3)
            if a == 0: hs.append(h)
        if key == 'thr':
            ax.axhline(100, ls='--', color='r', lw=2.5, zorder=4)
            ax.text(59.5, 96, r'f$_\mathbf{thrust,max}$', color='r', ha='right', va='top', fontsize=18, zorder=5)
            ax.set_ylim(0, 110); ax.set_ylabel('||$\\mathbf{f}_\\mathbf{thrust}$|| /\nf$_\\mathbf{thrust,max}$ (%)', fontsize=FS - 4)
            for (t0, t1), nm in zip(PHASE, PHASE_NAME):
                ax.text((t0 + t1) / 2, 82, nm, ha='center', va='top', fontsize=15, zorder=5)
        else:
            for s in (1, -1): ax.axhline(s * ANG_LIM, ls='--', color='r', lw=2.5, zorder=4)
            ax.text(59.5, ANG_LIM - 0.04, 'limit ±0.6 rad', color='r', ha='right', va='top', fontsize=16, zorder=5)
            ax.set_ylim(-0.75, 0.75)
            ax.set_ylabel([r'$\mathsf{φ}$ (rad)', r'$\mathsf{θ}$ (rad)'][a - 1], fontsize=FS)
        ax.set_xlim(0, 60); ax.set_xticks(range(0, 61, 10)); frame(ax)
        ax.set_xlabel('t (s)', fontsize=FS, labelpad=1)
    top = axs[0].get_position().y1
    fig.legend(hs, [f'Q {i + 1}' for i in range(5)], loc='lower center', bbox_to_anchor=(0.5, top + 6 / H), ncol=5,
               frameon=False, fontsize=FS, handlelength=1.8, columnspacing=1.2, handletextpad=0.5, borderaxespad=0, borderpad=0)

    # μ taraması: segment süreleri μ ile çarpılır (μ < 1 daha hızlı uçuş)
    SC = ['1.0', '0.9', '0.8', '0.7', '0.6', '0.55', '0.5', '0.45', '0.4']   # run_r4_sims.sh ile aynı
    rows = []
    for tag in SC:
        k = float(tag); P = [simlog(f'{LOG}/r4_scale_{tag}', d) for d in range(1, 6)]
        thr = max(T['r_thr'].max() / T_MAX[i] for i, T in enumerate(P))
        ref = max(max(np.abs(T['r_eul_ang.x']).max(), np.abs(T['r_eul_ang.y']).max()) for T in P)
        act = max(max(np.abs(T['a_eul_ang.x']).max(), np.abs(T['a_eul_ang.y']).max()) for T in P)
        rmse = max(np.sqrt(np.mean(sum((T[f'a_pos.{c}'] - T[f'r_pos.{c}'])**2 for c in 'xyz'))) for T in P)
        rows.append((k, thr * 100, ref / ANG_LIM * 100, act / ANG_LIM * 100, rmse * 1e3))
    rows = np.array(rows)
    pd.DataFrame(rows, columns=['time_scale_mu', 'peak_thrust_pct', 'peak_ref_tilt_pct_of_limit', 'peak_tilt_pct_of_limit',
                                'max_rmse_mm']).to_csv(f'{OUT}/FigR1_time_scale.csv', index=False)
    ax = axs[3]; stable = rows[:, 4] < 1000                 # μ ≤ 0.5: kontrol kaybı (RMSE metreler mertebesinde)
    k_lost = rows[~stable, 0].max()
    ax.axvspan(0.38, (k_lost + rows[stable, 0].min()) / 2, color=[.88, .88, .88], zorder=0, lw=0)
    ax.text((0.38 + (k_lost + rows[stable, 0].min()) / 2) / 2, 150, 'loss of\ncontrol\n' + r'($\mathsf{μ}$ ≤ ' + f'{k_lost:g})',
            ha='center', va='center', fontsize=16, zorder=5)
    ax.axhline(100, ls='--', color='r', lw=2.5, zorder=2)
    mk = [('o', [0, .4, 1], 1, r'peak ||$\mathbf{f}_\mathbf{thrust}$|| / f$_\mathbf{thrust,max}$'), ('s', [1, .5, 0], 2, r'peak |$\mathsf{φ}^\mathbf{des}$|, |$\mathsf{θ}^\mathbf{des}$| / limit'),
          ('D', [0, .65, 0], 3, r'peak |$\mathsf{φ}$|, |$\mathsf{θ}$| / limit')]
    hk = []
    for m, c, col, lab in mk:
        v = rows[:, col]
        h, = ax.plot(rows[stable, 0], v[stable], '-' + m, color=c, lw=LINE_W, ms=10, mec='k', zorder=4)
        hk.append(h)
    ax.annotate('nominal', (1.0, rows[0, 2]), xytext=(-12, 30), textcoords='offset points', ha='right', fontsize=16,
                arrowprops=dict(arrowstyle='->', lw=1.5))
    ax.set_xlim(0.38, 1.02); ax.set_ylim(0, 200); ax.set_xticks(np.round(np.arange(0.4, 1.01, 0.1), 1))
    ax.set_xlabel(r'time scale $\mathsf{μ}$ (segment durations × $\mathsf{μ}$)', fontsize=FS)
    ax.set_ylabel('peak value /\nlimit (%)', fontsize=FS - 2); frame(ax)
    ax.legend(hk, [x[3] for x in mk], loc='upper right', fontsize=15, frameon=False, handlelength=1.6, borderaxespad=0.3)
    for a_, tag in zip(axs, 'abcd'):                        # panel harfleri: sol üst köşe, eksen değerlerinin solunda
        a_.text(-0.105, 1.0, f'({tag})', transform=a_.transAxes, ha='left', va='top', fontsize=FS)
    save(fig, 'FigR1_feasibility')

if 'R2' in ONLY:
    # ======================= Fig. R2: segment türleri, zaman serisi =======================
    def read_spec(path):
        """spec dosyasından İHA -> {yörünge adı: zamanlar}"""
        out, cur, L = {}, None, open(path).read().split('\n')
        for i, line in enumerate(L):
            w = line.split()
            if w and w[0] == 'drone': cur = int(w[1]); out[cur] = {}
            elif w and w[0] == 'traj': out[cur][w[1]] = [float(x) for x in L[i + 4].split()[1:]]
        return out

    def knot_jumps(s, R, times_by_traj):
        """Polinom düğümlerinde ve yeniden planlama anlarında ivme ve jerk sıçramaları (tek taraflı limitler).
        Düğümler 0.05 s ızgarasına düşer; örneklenmeyen taraf komşu örnekten Taylor açılımıyla bulunur."""
        A, J, Sn = (s[[c + a for a in 'xyz']].to_numpy() for c in 'ajs')
        seg, tr, t = s.seg.to_numpy(), s.traj.to_numpy(), s.t.to_numpy()
        starts = [0] + [k for k in range(1, len(s)) if tr[k] != tr[k - 1]]
        old = {int(r.k): r for _, r in R.iterrows()}
        oldv = lambda r, c: np.array([r[f'{c}{a}_old'] for a in 'xyz'])
        res = []
        for p, k0 in enumerate(starts):
            times = times_by_traj[tr[k0]]
            tm = times if p == 0 else times + [times[-1] + 7.0]      # trajectory_sequencer.cpp: extra_time 7 s
            kend = starts[p + 1] if p + 1 < len(starts) else len(s)
            if p > 0:                                                # yeniden planlama: eski plan replan.csv'den
                r = old[k0]
                res.append((t[k0], np.linalg.norm(A[k0] - oldv(r, 'a')), np.linalg.norm(J[k0] - oldv(r, 'j'))))
            for i in range(1, len(tm) - 1):
                kk = k0 + int(round((tm[i] - tm[0]) / DT))
                if kk <= k0 or kk >= kend: continue
                if seg[kk] == i:                                      # örnek sağ polinomda, sol taraf kk-1'den
                    aL = A[kk - 1] + J[kk - 1] * DT + Sn[kk - 1] * DT**2 / 2; jL = J[kk - 1] + Sn[kk - 1] * DT
                    aR, jR = A[kk], J[kk]
                elif kk + 1 < kend:                                   # örnek sol polinomda, sağ taraf kk+1'den
                    aL, jL = A[kk], J[kk]
                    aR = A[kk + 1] - J[kk + 1] * DT + Sn[kk + 1] * DT**2 / 2; jR = J[kk + 1] - Sn[kk + 1] * DT
                elif kk + 1 in old:                                   # sağ polinom yalnızca eski plan olarak (replan.csv)
                    r = old[kk + 1]; aL, jL = A[kk], J[kk]
                    aR = oldv(r, 'a') - oldv(r, 'j') * DT + oldv(r, 's') * DT**2 / 2; jR = oldv(r, 'j') - oldv(r, 's') * DT
                else:
                    continue
                res.append((t[kk], np.linalg.norm(aR - aL), np.linalg.norm(jR - jL)))
        return res

    def window_rms(t, e, win=0.05):
        N = max(1, round(win / np.median(np.diff(t)))); n = len(e) // N
        return t[:n * N].reshape(n, N).mean(1), np.sqrt((e[:n * N]**2).reshape(n, N).mean(1))

    JUMP_TOL = 0.05                                         # bunun altındaki sıçramalar sayısal hata (m/s², m/s³)
    REF, SIM, JMP, SUMM = {}, {}, {}, {}
    for v in VAR:
        S = pd.read_csv(f'{EV}/{v}/samples.csv'); R = pd.read_csv(f'{EV}/{v}/replan.csv')
        s = S[S.drone == SEL].reset_index(drop=True); R = R[R.drone == SEL]
        REF[v] = s; JMP[v] = knot_jumps(s, R, read_spec(f'{EV}/spec_{v}.txt')[SEL])
        T = simlog(f'{LOG}/r4_{v}', SEL); SIM[v] = T
        jn = np.linalg.norm(s[['jx', 'jy', 'jz']].to_numpy(), axis=1); sn = np.linalg.norm(s[['sx', 'sy', 'sz']].to_numpy(), axis=1)
        e = np.sqrt(sum((T[f'a_pos.{c}'] - T[f'r_pos.{c}'])**2 for c in 'xyz'))
        tilt = np.arccos(np.clip(np.cos(T['a_eul_ang.x']) * np.cos(T['a_eul_ang.y']), -1, 1))
        tau = np.sqrt(T['r_torq.x']**2 + T['r_torq.y']**2 + T['r_torq.z']**2)
        ja = [x[1] for x in JMP[v]]; jj = [x[2] for x in JMP[v]]
        SUMM[v] = dict(acc_jump_max=max([0.0] + [a for a in ja if a > JUMP_TOL]),
                       n_acc_jumps=sum(a > JUMP_TOL for a in ja), n_jerk_jumps=sum(j > JUMP_TOL for j in jj),
                       jerk_jump_max=max([0.0] + [j for j in jj if j > JUMP_TOL]),
                       jerk_sq_int=np.sum(jn**2) * DT, jerk_max=jn.max(), snap_max=sn.max(),
                       rmse_mm=np.sqrt(np.mean(e**2)) * 1e3, tilt_max=tilt.max(),
                       thr_dev_rms=np.sqrt(np.mean((T['r_thr'] - MASS[SEL - 1] * 9.81)**2)),
                       tau_rms_mNm=np.sqrt(np.mean(tau**2)) * 1e3)
        SIM[v]['tilt'], SIM[v]['e'], SIM[v]['tau'] = tilt, e, tau
    pd.DataFrame(SUMM).T.to_csv(f'{OUT}/FigR2_segment_metrics_Q{SEL}.csv')
    t_end_ref = max(REF[v].t.iloc[-1] for v in VAR)
    T_END = max(SIM[v]['t'][-1] for v in VAR)

    fig = plt.figure(figsize=(1400 * PX, 2070 * PX)); H = 2070
    top, bottom, gap = 1 - 115 / H, 70 / H, 28 / H
    n = 7; h = (top - bottom - (n - 1) * gap) / n
    axs = [fig.add_axes([0.085, top - (a + 1) * h - a * gap, 0.895, h]) for a in range(n)]

    def base(ax, letter, ylabel, last=False):
        for (t0, t1), c in zip(PHASE, C_PHASE): ax.axvspan(t0, t1, color=c, zorder=0, lw=0)
        for ts in T_SWITCH: ax.axvline(ts, ls='--', color=[.3, .3, .3], lw=1.2, zorder=1)
        ax.set_xlim(0, T_END); ax.set_xticks(range(0, int(T_END) + 1, 10))
        ax.tick_params(labelsize=FST, labelbottom=last); frame(ax, FST)
        ax.set_ylabel(ylabel, fontsize=FSL)
        ax.text(-0.075, 1.0, f'({letter})', transform=ax.transAxes, ha='left', va='top', fontsize=FSL)
        if last: ax.set_xlabel('t (s)', fontsize=FSL)

    def summary(ax, title, fmt):
        hs = [Line2D([], [], color=VCOL[v], lw=3) for v in VAR]
        lg = ax.legend(hs, [f'{VABB[v]}: {fmt(v)}' for v in VAR], title=title, ncol=4, loc='upper right', fontsize=FSB,
                       title_fontsize=FSB, frameon=True, fancybox=False, edgecolor=[.6, .6, .6], framealpha=0.92,
                       handlelength=1.4, columnspacing=1.0, handletextpad=0.4, borderaxespad=0.3)
        lg.set_zorder(20)

    def lines(ax, getter, ymax_frac=1.55):
        top_val = 0
        for v in VAR:
            x, y = getter(v); top_val = max(top_val, np.nanmax(y))
            ax.plot(x, y, color=VCOL[v], lw=VLW[v], zorder=VZ[v])
        ax.set_ylim(0, top_val * ymax_frac)

    ref_t = lambda v: REF[v].t.to_numpy()
    nrm = lambda v, c: np.linalg.norm(REF[v][[c + a for a in 'xyz']].to_numpy(), axis=1)
    # Panel sırası: konum hatası ve yörünge türevleri (ivme, jerk, snap), ardından eğim, itki ve tork
    ORDER = ['err', 'acc', 'jerk', 'snap', 'tilt', 'thr', 'tau']
    AX = {k: axs[i] for i, k in enumerate(ORDER)}
    LET = {k: 'abcdefg'[i] for i, k in enumerate(ORDER)}
    LAST = {k: k == ORDER[-1] for k in ORDER}
    for (t0, t1), nm in zip(PHASE, ['CS', 'MS', 'MS', 'MJ']):      # hibritin segment türleri, en üst panelin üstünde
        axs[0].text((t0 + min(t1, t_end_ref)) / 2, 1.02, nm, transform=axs[0].get_xaxis_transform(), ha='center', va='bottom',
                    fontsize=FST, color=[.35, .35, .35])
    # konum hatası (0. mertebe)
    ax = AX['err']; lines(ax, lambda v: window_rms(SIM[v]['t'], SIM[v]['e'] * 1e3))
    base(ax, LET['err'], r'||$\mathbf{e}_\mathbf{p}$|| (mm)', last=LAST['err'])
    summary(ax, 'RMSE', lambda v: f'{SUMM[v]["rmse_mm"]:.1f} mm')
    # ivme (2. mertebe)
    ax = AX['acc']; lines(ax, lambda v: (ref_t(v), nrm(v, 'a')))
    base(ax, LET['acc'], r'||$\mathbf{a}^\mathbf{des}$|| (m/s$^2$)', last=LAST['acc'])
    summary(ax, 'max. jump', lambda v: f'{SUMM[v]["acc_jump_max"]:.2f} m/s$^2$')
    # itki sapması (ivmenin büyüklüğünden)
    ax = AX['thr']; lines(ax, lambda v: window_rms(SIM[v]['t'], SIM[v]['r_thr'] - MASS[SEL - 1] * 9.81))
    base(ax, LET['thr'], r'| ||$\mathbf{f}_\mathbf{thrust}$|| − mg | (N)', last=LAST['thr'])
    summary(ax, 'RMS', lambda v: f'{SUMM[v]["thr_dev_rms"]:.3f} N')
    # eğim açısı (ivmenin yönünden)
    ax = AX['tilt']; lines(ax, lambda v: (SIM[v]['t'][::5], SIM[v]['tilt'][::5]))
    base(ax, LET['tilt'], 'tilt angle (rad)', last=LAST['tilt'])
    summary(ax, 'max.', lambda v: f'{SUMM[v]["tilt_max"]:.2f} rad')
    # jerk (3. mertebe)
    ax = AX['jerk']; lines(ax, lambda v: (ref_t(v), nrm(v, 'j')))
    base(ax, LET['jerk'], r'||$\mathbf{j}^\mathbf{des}$|| (m/s$^3$)', last=LAST['jerk'])
    summary(ax, r'$\int$||$\mathbf{j}^\mathbf{des}$||$^2$dt' + (' (* plus acceleration jumps)' if any(SUMM[v]['n_acc_jumps'] for v in VAR) else ''),
            lambda v: f'{SUMM[v]["jerk_sq_int"]:.1f}' + ('*' if SUMM[v]['n_acc_jumps'] else '') + r' m$^2$/s$^5$')
    # snap (4. mertebe); jerk sıçramalarındaki snap darbeleri çizilmez, değerde * ile belirtilir
    ax = AX['snap']; lines(ax, lambda v: (ref_t(v), nrm(v, 's')))
    base(ax, LET['snap'], r'||$\mathbf{s}^\mathbf{des}$|| (m/s$^4$)', last=LAST['snap'])
    summary(ax, 'max.' + (' (* plus impulses at jerk jumps)' if any(SUMM[v]['n_jerk_jumps'] for v in VAR) else ''),
            lambda v: f'{SUMM[v]["snap_max"]:.2f}' + ('*' if SUMM[v]['n_jerk_jumps'] else '') + r' m/s$^4$')
    # tork (snap'ten)
    ax = AX['tau']; lines(ax, lambda v: window_rms(SIM[v]['t'], SIM[v]['tau'] * 1e3))
    base(ax, LET['tau'], r'||$\mathbf{τ}^\mathbf{des}$|| (mN·m)', last=LAST['tau'])
    summary(ax, 'RMS', lambda v: f'{SUMM[v]["tau_rms_mNm"]:.1f} mN·m')

    hs = [Line2D([], [], color=VCOL[v], lw=VLW[v] + 0.5) for v in VAR]
    fig.legend(hs, [VLAB[v] for v in VAR], loc='lower center',
               bbox_to_anchor=(0.5, top + 45 / H), ncol=4, frameon=False, fontsize=FST, handlelength=1.8,
               columnspacing=1.2, handletextpad=0.5, borderaxespad=0, borderpad=0)
    save(fig, f'FigR2_segment_comparison')

# ======================= Fig. R3: hesaplama süresi, zaman serisi =======================
# (a) dört planlayıcı, her tick'in tüm tekrarlardaki medyanı; (b) hibrit planlayıcı, medyan ve en kötü değer
TM = {}
for v in VAR:
    d = pd.read_csv(f'{EV}/{v}/timing.csv'); d = d[d.drone == SEL]
    TM[v] = d[d.k < d.groupby('rep').k.transform('max')]          # son tick: dizici bitti, ölçüm anlamsız
NREP = TM['hybrid'].rep.nunique()
if 'T_END' not in globals():                                       # R2 çizilmediyse: yörünge süresi + 2 s bekleme
    T_END = max(TM[v].k.max() for v in VAR) * DT + 2.0
fig = plt.figure(figsize=(1400 * PX, 870 * PX)); H = 870
top, bottom, gap = 1 - 95 / H, 75 / H, 80 / H     # her panelde eksen adı
h = (top - bottom - gap) / 2
axs = [fig.add_axes([0.085, top - (a + 1) * h - a * gap, 0.895, h]) for a in range(2)]
summ = []
def stats(d, agg):
    g = d.groupby('k').ns.agg(agg) / 1e3; pl = d[d.planning == 1].groupby('k').ns.agg(agg) / 1e3
    return g, pl, pl.max(), g[~g.index.isin(pl.index)].max()
for a, ax in enumerate(axs):
    for (t0, t1), c in zip(PHASE, C_PHASE): ax.axvspan(t0, t1, color=c, zorder=0, lw=0)
    for ts in T_SWITCH: ax.axvline(ts, ls='--', color=[.3, .3, .3], lw=1.2, zorder=1)
    ax.axhline(50e3, ls='--', color='r', lw=2.5, zorder=2)
    ax.text(0.5, 50e3 * 0.62, '50 ms loop period', color='r', ha='left', va='top', fontsize=FST)
    ax.set_yscale('log'); ax.set_ylim(0.2, 1e6); ax.set_xlim(0, T_END); ax.set_xticks(range(0, int(T_END) + 1, 10))
    frame(ax, FST); ax.set_xlabel('t (s)', fontsize=FSL, labelpad=1)
    ax.text(-0.075, 1.0, f'({"ab"[a]})', transform=ax.transAxes, ha='left', va='top', fontsize=FSL)
for v in VAR:                                                      # (a)
    g, pl, rp, tk = stats(TM[v], 'median')
    axs[0].plot(g.index * DT, g.to_numpy(), color=VCOL[v], lw=2.2 if v == 'hybrid' else 1.6, zorder=VZ[v])
    axs[0].plot(pl.index * DT, pl.to_numpy(), 'o', color=VCOL[v], ms=8, mec='k', zorder=VZ[v] + 5)
    summ.append(dict(method=v, stat=f'median of {NREP} runs', replan_us=rp, tick_us=tk))
axs[0].set_ylabel(f'time per tick (µs)\nmedian of {NREP} runs', fontsize=FSL)
lg = axs[0].legend([Line2D([], [], color=VCOL[v], lw=3) for v in VAR],
                   [f'{VABB[v]}: {summ[i]["replan_us"]:.0f} / {summ[i]["tick_us"]:.1f}' for i, v in enumerate(VAR)],
                   title='regeneration / sampling, median (µs)', ncol=4, loc='upper right', fontsize=FSB,
                   title_fontsize=FSB, frameon=True, fancybox=False, edgecolor=[.6, .6, .6], framealpha=0.92,
                   handlelength=1.4, columnspacing=1.0, handletextpad=0.4, borderaxespad=0.3)
lg.set_zorder(20)
GREY = [.6, .6, .6]                                                # (b)
gx, plx, rpx, tkx = stats(TM['hybrid'], 'max'); gm, plm, rpm, tkm = stats(TM['hybrid'], 'median')
axs[1].plot(gx.index * DT, gx.to_numpy(), color=GREY, lw=1.2, zorder=3)
axs[1].plot(gm.index * DT, gm.to_numpy(), color=VCOL['hybrid'], lw=2.2, zorder=4)
axs[1].plot(plx.index * DT, plx.to_numpy(), 'o', color=GREY, ms=8, mec='k', zorder=6)
axs[1].plot(plm.index * DT, plm.to_numpy(), 'o', color=VCOL['hybrid'], ms=8, mec='k', zorder=7)
summ.append(dict(method='hybrid', stat=f'maximum of {NREP} runs', replan_us=rpx, tick_us=tkx))
axs[1].set_ylabel('time per tick (µs)\nhybrid planner (H)', fontsize=FSL)
lg = axs[1].legend([Line2D([], [], color=VCOL['hybrid'], lw=3), Line2D([], [], color=GREY, lw=3)],
                   [f'median: {rpm:.0f} / {tkm:.1f}', f'maximum of {NREP} runs: {rpx:.0f} / {tkx:.0f}'],
                   title='regeneration / sampling (µs)', ncol=2, loc='upper right', fontsize=FSB,
                   title_fontsize=FSB, frameon=True, fancybox=False, edgecolor=[.6, .6, .6], framealpha=0.92,
                   handlelength=1.4, columnspacing=1.0, handletextpad=0.4, borderaxespad=0.3)
lg.set_zorder(20)
hw = f'{EV}/hardware.txt'                                # run_r4_sims.sh yazar (traj_eval'in koştuğu makine)
if os.path.exists(hw):
    axs[0].text(0.5, 1.5e3, open(hw).readline().strip(), ha='left', va='center', fontsize=FST, zorder=10)
hs = [Line2D([], [], color=VCOL[v], lw=VLW[v] + 0.5) for v in VAR] + \
     [Line2D([], [], ls='none', marker='o', color='w', mec='k', ms=9)]
fig.legend(hs, [VLAB[v] for v in VAR] + ['regeneration tick'], loc='lower center', bbox_to_anchor=(0.5, top + 30 / H),
           ncol=5, frameon=False, fontsize=FST, handlelength=1.8, columnspacing=1.2, handletextpad=0.5,
           borderaxespad=0, borderpad=0)
pd.DataFrame(summ).to_csv(f'{OUT}/FigR3_timing_Q{SEL}.csv', index=False)
save(fig, 'FigR3_computation_time')
if 'R2' in ONLY: print(pd.DataFrame(SUMM).T.round(3).to_string())
print(pd.DataFrame(summ).round(2).to_string(index=False))
