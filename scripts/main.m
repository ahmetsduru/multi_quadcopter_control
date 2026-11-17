%% MAKALE GRAFİKLERİ - Disturbance-Aware Hybrid Trajectory Generation and Control for Multirotor Swarms
% Sıralı simülasyonun (launch/sequential_sim.launch) loglarından makaledeki ve ek materyaldeki
% grafikleri ve tabloları üretir:
%
% Her simülasyon koşusu kendi klasörüne yazar: log/<koşu>/
%   pid_ff, pid, ndob, se3, se3_ff : bozuculu koşular, kontrolcü adıyla (launch: controller:=...)
%   pid_nominal                     : bozucusuz PID (disturbance_enabled:=false)
%   pid_ff_beta0.1                  : PID+FF, yaw filtresi beta = 0.1
% Hepsi tek komutla üretilir:
%   rosrun multi_quadcopter_control run_paper_sims.sh
% veya tek tek (log_dir verilmezse log/<controller>/ klasörüne yazılır):
%   roslaunch multi_quadcopter_control sequential_sim.launch rviz:=false real_time_factor:=0 shutdown_on_finish:=true controller:=se3
%
% Hangi grafiğin hangi koşuyu kullanacağı AYARLAR bölümünde seçilir (main_ctrl, fig2_ctrls, ...).
% Varsayılanlarla makaledeki ve ek materyaldeki grafik ve tablolar üretilir:
%   Fig. 1  Pozisyon (x, y, z) + z zoom                         -> main_ctrl
%   Fig. 2  Anlık konum hatası (50 ms pencere RMS), 2-5 koşu   -> fig2_ctrls
%   Fig. 3  Hız / Fig. 4 İvme / Fig. 5 Euler açıları            -> main_ctrl
%   Fig. 6  Bozucu kuvvetler / Fig. 7 Thrust                    -> main_ctrl
%   NDOB    NDOB bozucu kestirimi, 3 eksen, tüm İHA'lar          -> NDOB koşusu (mismatch_ctrls)
%   Fig. 8  Ablasyon: ileri besleme var / yok (50 ms pencere RMS) -> fig8_ctrls
%   Fig. 9  Uyumsuz kompanzatör: downwash vs NDOB kestirimi ve konum hatası -> mismatch_ctrls
%   Fig. S5 Drone 1 hibrit yörünge ve waypointler               -> main_ctrl + config/*.yaml
%   Fig. S6 Drone 2 yönelim hatası, beta karşılaştırması        -> main_ctrl vs beta_run
%   Fig. F1 Komşu İHA'lar arası bağıl mesafe ve formasyon hatası -> formation_ctrls
%   Table 1 RMSE/ISE, ilk koşunun diğerlerine göre farkı (%)   -> fig2_ctrls
%   Table 2 RMSE/ISE, ablasyon (çift çift)                      -> fig8_ctrls
%   Table 3 İHA çiftleri matrisi: üst üçgen formasyon hatası (RMS, baskın yön xy/z),
%           alt üçgen bağıl mesafe hatası (RMS, işaretli ortalama) -> formation_ctrls
%   Table 4 Uyumsuz kompanzatör: RMSE ve bozucusuz koşuya göre artış (ΔRMSE) -> mismatch_ctrls + nominal_run
%   Table S2 Simülasyon parametreleri (config/*.yaml + launch/sequential_sim.launch)
%   Table S3 Kontrolcü kazançları (PID, SE(3), NDOB)                -> config/*.yaml
%   (Fig. S2 için: fig_S2_downwash_field.m)
% Her figür SVG (vektör) olarak scripts/Figures/<main_ctrl>/ klasörüne
% yazılır. Eksik koşulara bağlı grafikler uyarı verilerek atlanır.

clear; clc; close all;
warning('off', 'MATLAB:print:ContentTypeImageSuggested');   % vektör çıktı isteniyor

%% === AYARLAR ===
num_drones = 5;

script_dir = fileparts(mfilename('fullpath'));
if isempty(script_dir), script_dir = pwd; end
pkg_dir    = fullfile(script_dir, '..');

% --- Hangi grafik hangi koşuyu kullanır? (koşu adı = log/ altındaki klasör adı) ---
% Kontrolcüler: pid_ff (PID+FF), pid (PID), ndob (PID+NDOB), se3 (SE(3)), se3_ff (SE(3)+FF)
main_ctrl       = 'pid_ff';                    % Fig. 1, 3-7, S5, S6'nın ana koşusu
fig2_ctrls      = {'pid_ff', 'se3_ff', 'ndob'};  % Fig. 2 / Table 1: 2-5 koşu (ilki referans: Table 1'deki
                                                       % fark % buna göre; çizgilerde en üstte). Renkler: ctrlColor
fig2_window     = 0.05;   % s; Fig. 2'deki hata bu pencerelerdeki RMS'tir. Referans 20 Hz'de (50 ms) güncellendiği
                          % için ham hata her periyotta testere dişi çizer; 50 ms RMS bunu düzler ve karesel
                          % ortalaması tablodaki RMSE'ye eşittir.
fig8_ctrls      = {'pid_ff', 'pid', 'se3_ff', 'se3'};   % Fig. 8 / Table 2: ablasyon çiftleri sırayla
                                               % (ileri beslemeli, ileri beslemesiz), (..., ...)
formation_ctrls = {'pid_ff', 'pid', 'se3_ff'};   % Fig. F1 / Table 3
mismatch_ctrls  = {'pid', 'ndob', 'pid_ff'};   % Fig. 9 / Table 4: kompanzasyon yok / gözlemci / model FF
nominal_run     = 'pid_nominal';               % Table 4: bozucusuz koşu (disturbance_enabled:=false)
beta_main       = 0.05;                        % ana koşudaki psi_filter_beta
beta_alt        = 0.1;                         % Fig. S6'da karşılaştırılan psi_filter_beta
beta_run        = sprintf('%s_beta%g', main_ctrl, beta_alt);

log_root   = fullfile(pkg_dir, 'log');
config_dir = fullfile(pkg_dir, 'config');      % Fig. S5 waypointleri

s6_drone    = 2;      % Fig. S6 drone'u
fig8_drones = [2 3];  % Fig. 8 drone'ları
fig9_drones = [2 3];  % Fig. 9 drone'ları (downwash etkisindeki İHA'lar)
s5_drone    = 1;      % Fig. S5 drone'u

% Formasyon ölçütleri (Fig. F1, Table 3)
% Komşuluk zamana bağlıdır: referans (istenen) mesafesi neighbor_radius'tan küçük olan çiftler
% o anda komşudur (formasyon uçuş boyunca değiştiği için sabit bir graf kullanılmaz).
neighbor_radius = 3.0;   % m

out_folder = fullfile(script_dir, 'Figures', main_ctrl);   % her ana koşunun çıktıları ayrı klasörde (SVG + CSV)
output_pdf = fullfile(out_folder, 'all_figures.pdf');     % tüm figürler tek dosyada (göz atmak için)

% Zaman ekseni (makaledeki gibi 0-60 s)
t_axis_max = 60;

% Yörünge geçiş anları (s). Sıralı simülasyonda deterministiktir:
% cubic spline -> min snap (8.05 s) -> min snap (21.05 s) -> min jerk (32.55 s), bitiş 54.55 s.
t_switch = [8.05 21.05 32.55];

% Downwash etkileşim bölgeleri (Fig. 6, 7, 9) ana koşunun loglarından bulunur (|f_dist| > interact_thr):
%   yeşil: ilk downwash etkisinin başladığı an -> iki İHA'nın aynı anda etkilendiği son an
%          (Q5 -> Q2 ve Q4 -> Q3)
%   sarı : yeşilin sonu -> t_axis_max (yalnızca Q2 etkileniyor: Q5, sonra Q4 ve Q5)
interact_thr = 0.05;   % N

% Fig. 1 z zoom pencereleri (s). Zoom'un z aralığı, zoom_drones'un (downwash etkisindeki İHA'lar)
% o penceredeki gerçek ve istenen z değerlerinden belirlenir.
zoom_windows = [23 25; 34 36];
zoom_colors  = {'r', 'b'};
zoom_drones  = [2 3];

% Çubuk grafikleri (Fig. 2, 8, S6): her sparse_factor örnekten biri (500 Hz log -> 0.6 s)
sparse_factor = 300;

% Çizgi grafikleri her plot_step örnekten birini çizer (500 Hz log -> 100 Hz). Vektör SVG dosyalarını
% küçültür ve hızlandırır; metrikler (RMSE, ISE) ve çubuk grafikleri her zaman tam veriyi kullanır.
plot_step = 5;

% Grafik stili
font_name = 'Times New Roman';
font_size = 20;
line_w    = 3;     % gerçek değer çizgileri
ref_w     = 2;     % istenen değer çizgileri: siyah noktalı, gerçek değerlerin üstünde (Fig. 5'te yok)
ref_col   = [0 0 0];
colors    = lines(num_drones);
dark_gray = [0.2 0.2 0.2];

c_phase  = [1 0.88 0.88; 0.85 1 1; 0.85 1 0.85; 1 1 0.9];   % pembe, mavi, yeşil, sarı
c_green  = [0.85 1 0.85];
c_yellow = [1 1 0.9];

%% === HAZIRLIK ===
if ~exist(out_folder, 'dir'), mkdir(out_folder); end
if isfile(output_pdf), delete(output_pdf); end


phase_zones    = [[0 t_switch]' [t_switch t_axis_max]'];
axis_names     = ["x", "y", "z"];

S = struct('font_name', font_name, 'font_size', font_size, 'line_w', line_w, 'ref_w', ref_w, ...
           'ref_col', ref_col, 't_max', t_axis_max);

%% === VERİ OKUMA (gereken her koşu bir kez) ===
% Her koşu log/<koşu>/ klasöründen okunur ve run_info.txt ile beklenen kontrolcü, bozucu ve beta
% ayarına sahip olduğu kontrol edilir.
run_names = unique([{main_ctrl, nominal_run, beta_run}, fig8_ctrls, ...
                    fig2_ctrls, formation_ctrls, mismatch_ctrls], 'stable');
run_data = containers.Map();
for k = 1:numel(run_names)
    rdir = fullfile(log_root, run_names{k});
    run_data(run_names{k}) = readLogs(rdir, num_drones, plot_step);
    checkRunInfo(rdir, run_data(run_names{k}), run_names{k}, expectedRun(run_names{k}, beta_main));
end

D = run_data(main_ctrl);
valid = find([D.ok]);
if isempty(valid)
    error('Ana koşunun (%s) logları okunamadı: %s', main_ctrl, fullfile(log_root, main_ctrl));
end
fprintf('Ana koşu: %s (%s), %d drone logu\n', main_ctrl, ctrlLabel(main_ctrl), numel(valid));
D_f2   = cellfun(@(n) run_data(n), fig2_ctrls, 'UniformOutput', false);   % Fig. 2 / Table 1
f2_ok  = cellfun(@(x) any([x.ok]), D_f2);
D_f8   = cellfun(@(n) run_data(n), fig8_ctrls, 'UniformOutput', false);   % Fig. 8 / Table 2
f8_ok  = cellfun(@(x) any([x.ok]), D_f8);
D_beta = run_data(beta_run);         % Fig. S6
D_nom  = run_data(nominal_run);      % Table 4
D_none = run_data(mismatch_ctrls{1});   % Fig. 9 / Table 4: kompanzasyon yok
D_obs  = run_data(mismatch_ctrls{2});   %                    gözlemci (NDOB)
D_ff   = run_data(mismatch_ctrls{3});   %                    model tabanlı FF
beta_log_dir     = fullfile(log_root, beta_run);
nominal_log_dir  = fullfile(log_root, nominal_run);

% Downwash etkileşim bölgeleri (ana koşunun bozucu kuvvetlerinden)
interact_zones = interactionZones(D, interact_thr, t_axis_max);
interact_cols  = [c_green; c_yellow];
interact_cols  = interact_cols(1:size(interact_zones, 1), :);
fprintf('Downwash etkileşim bölgeleri (s): %s\n', mat2str(interact_zones));

%% === Fig. 1 - POZİSYON + Z ZOOM ===
fig = figure('Position', [100, 100, 1000, 900]);
axs = stackedAxes(fig, 3);
lgd_handles = gobjects(0);
for a = 1:3
    ax = axs(a); hold(ax, 'on');
    if a == 1
        lgd_handles(end+1) = plot(ax, NaN, NaN, ':', 'Color', ref_col, 'LineWidth', ref_w, ...
                                  'DisplayName', 'Desired'); %#ok<SAGROW>
    end
    for i = valid
        h = plot(ax, D(i).tp, D(i).Tp{:, "a_pos." + axis_names(a)}, '-', 'Color', colors(i,:), ...
                 'LineWidth', line_w, 'DisplayName', sprintf('Q %d', i));
        if a == 1, lgd_handles(end+1) = h; else, h.HandleVisibility = 'off'; end %#ok<SAGROW>
    end
    for i = valid                                 % istenen değerler (üstte, siyah noktalı)
        plot(ax, D(i).tp, D(i).Tp{:, "r_pos." + axis_names(a)}, ':', 'Color', ref_col, 'LineWidth', ref_w, ...
             'HandleVisibility', 'off');
    end
    yl = ylim(ax);
    if a < 3
        ylim(ax, [yl(1) - 1, yl(2) + 1]);
    else
        ylim(ax, [yl(1) - 0.4, yl(2) + 0.2]);   % altta zoom inset'leri için boşluk
    end
    xlabel(ax, 't (s)', 'FontWeight', 'bold', 'FontSize', font_size, 'FontName', font_name);
    addZones(ax, phase_zones, c_phase);
    styleAxes(ax, S);
    ylabel(ax, sprintf('%s (m)', axis_names(a)), 'FontWeight', 'bold', 'FontSize', font_size, ...
           'FontName', font_name);
end
hZ = ax;
zoom_zlim = zeros(size(zoom_windows));
for w = 1:size(zoom_windows, 1)
    vals = [];
    for i = zoom_drones
        if ~D(i).ok, continue; end
        m = D(i).t >= zoom_windows(w, 1) & D(i).t <= zoom_windows(w, 2);
        vals = [vals; D(i).T{m, "a_pos.z"}; D(i).T{m, "r_pos.z"}]; %#ok<AGROW>
    end
    zoom_zlim(w, :) = padLimits(vals, 0.1);
    for tw = zoom_windows(w, :)
        line(hZ, [tw tw], zoom_zlim(w, :), 'Color', zoom_colors{w}, 'LineStyle', '--', ...
             'LineWidth', line_w, 'HandleVisibility', 'off');
    end
end
main_pos     = get(hZ, 'Position');
inset_height = main_pos(4) * 0.26;
inset_y      = main_pos(2) + 0.03;
inset_x      = [0.05 0.35];
for w = 1:size(zoom_windows, 1)
    ax_zoom = axes(fig, 'Position', [main_pos(1) + inset_x(w), inset_y, main_pos(3) * 0.25, inset_height]);
    box(ax_zoom, 'on'); hold(ax_zoom, 'on');
    for i = valid
        plot(ax_zoom, D(i).tp, D(i).Tp{:, "a_pos.z"}, 'Color', colors(i,:), 'LineWidth', line_w);
    end
    for i = valid
        plot(ax_zoom, D(i).tp, D(i).Tp{:, "r_pos.z"}, ':', 'Color', ref_col, 'LineWidth', ref_w);
    end
    xlim(ax_zoom, zoom_windows(w, :));
    ylim(ax_zoom, zoom_zlim(w, :));
    set(ax_zoom, 'FontSize', 9, 'FontName', font_name, 'XTick', [], 'YTick', [], 'LineWidth', 1.5);
end
addTopLegend(lgd_handles, [0.1 0.94 0.8 0.05], S);
saveFigure(fig, out_folder, 'Fig1.svg', output_pdf);

%% === Fig. 2 - ANLIK KONUM HATASI: fig2_ctrls (2-5 koşu, 50 ms pencere RMS) ===
if numel(fig2_ctrls) < 2 || numel(fig2_ctrls) > 5
    error('fig2_ctrls 2-5 koşu içermeli (şu an %d).', numel(fig2_ctrls));
end
f2_labels = cellfun(@ctrlLabel, fig2_ctrls, 'UniformOutput', false);
if all(f2_ok)
    fig = figure('Position', [100, 100, 1200, 1000]);
    tl = tiledlayout(fig, 3, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    for i = 1:num_drones
        ax = nexttile(tl); hold(ax, 'on'); box(ax, 'on');
        hs = plotErrorLines(ax, cellfun(@(x) x(i), D_f2, 'UniformOutput', false), fig2_ctrls, fig2_window);
        styleErrorAxes(ax, i, phase_zones, c_phase, S);
    end
    % Ortak açıklama, boş kalan son kutuda
    lgd = legend(hs(isgraphics(hs)), f2_labels(isgraphics(hs)), 'FontSize', 18, 'FontWeight', 'bold', ...
                 'FontName', font_name, 'Box', 'on');
    lgd.Layout.Tile = num_drones + 1;
    saveFigure(fig, out_folder, 'Fig2.svg', output_pdf);
else
    warning('Fig. 2 / Table 1 atlandı: %s logları bulunamadı (log/ altında).', strjoin(fig2_ctrls(~f2_ok), ', '));
end

%% === Fig. 3 - HIZ ===
plotStateFigure(D, valid, "a_vel.", "r_vel.", ["v_x (m/s)", "v_y (m/s)", "v_z (m/s)"], ...
                phase_zones, c_phase, colors, S, out_folder, 'Fig3.svg', output_pdf);

%% === Fig. 4 - İVME ===
% Not: a_z gerçekten "a_acc.z" sütunundan okunur (eski script bu panele z konumunu çiziyordu).
plotStateFigure(D, valid, "a_acc.", "r_acc.", ["a_x (m/s^2)", "a_y (m/s^2)", "a_z (m/s^2)"], ...
                phase_zones, c_phase, colors, S, out_folder, 'Fig4.svg', output_pdf);

%% === Fig. 5 - EULER AÇILARI ===
% Açı sembolleri skaler: dik ve kalın değil. TeX'in \phi'si yerine Unicode φ, θ, ψ; "\rm{}" sembolü normal,
% "\bf" birimi diğer etiketler gibi kalın yazar ("{}" komut adını sembolden ayırır).
plotStateFigure(D, valid, "a_eul_ang.", "r_eul_ang.", ...
                "\rm{}" + [string(char(966)), string(char(952)), string(char(968))] + "\bf (rad)", ...
                phase_zones, c_phase, colors, S, out_folder, 'Fig5.svg', output_pdf, false);   % referans çizilmez

%% === Fig. 6 - BOZUCU KUVVETLER ===
fig = figure('Position', [100, 100, 1000, 900]);
axs = stackedAxes(fig, 3);
lgd_handles = gobjects(0);
for a = 1:3
    ax = axs(a); hold(ax, 'on');
    all_vals = [];
    for i = valid
        v = D(i).Tp{:, "dist_f." + axis_names(a)};
        all_vals = [all_vals; v]; %#ok<AGROW>
        h = plot(ax, D(i).tp, v, 'LineWidth', line_w, 'Color', colors(i,:), 'DisplayName', sprintf('Q %d', i));
        if a == 1, lgd_handles(end+1) = h; else, h.HandleVisibility = 'off'; end %#ok<SAGROW>
    end
    ylim(ax, padLimits(all_vals, 0.1));
    addZones(ax, interact_zones, interact_cols);
    styleAxes(ax, S);
    ylabel(ax, sprintf('f_{dist,%s} (N)', axis_names(a)), 'FontWeight', 'bold', ...
           'FontSize', font_size, 'FontName', font_name);
    xlabel(ax, 't (s)', 'FontWeight', 'bold', 'FontSize', font_size, 'FontName', font_name);
end
addTopLegend(lgd_handles, [0.1 0.94 0.8 0.05], S);
saveFigure(fig, out_folder, 'Fig6.svg', output_pdf);

%% === NDOB KESTİRİMİ (controller:=ndob koşusu, ndob_f sütunları) ===
% Kaynak: ana koşu bir NDOB koşusuysa o, değilse mismatch_ctrls'deki gözlemci koşusu.
D_est = [];
if hasNdobColumns(D)
    D_est = D;
elseif hasNdobColumns(D_obs)
    D_est = D_obs;
end
if ~isempty(D_est)
    valid_est = find([D_est.ok]);
    fig = figure('Position', [100, 100, 1000, 900]);
    axs = stackedAxes(fig, 3);
    lgd_handles = gobjects(0);
    for a = 1:3
        ax = axs(a); hold(ax, 'on');
        all_vals = [];
        for i = valid_est
            v = D_est(i).Tp{:, "ndob_f." + axis_names(a)};
            all_vals = [all_vals; v]; %#ok<AGROW>
            h = plot(ax, D_est(i).tp, v, 'LineWidth', line_w, 'Color', colors(i,:), 'DisplayName', sprintf('Q %d', i));
            if a == 1, lgd_handles(end+1) = h; else, h.HandleVisibility = 'off'; end %#ok<SAGROW>
        end
        ylim(ax, padLimits(all_vals, 0.1));
        addZones(ax, interact_zones, interact_cols);
        styleAxes(ax, S);
        ylabel(ax, sprintf('\\Delta f_{%s} (N)', axis_names(a)), 'FontWeight', 'bold', ...
               'FontSize', font_size, 'FontName', font_name);
        xlabel(ax, 't (s)', 'FontWeight', 'bold', 'FontSize', font_size, 'FontName', font_name);
    end
    addTopLegend(lgd_handles, [0.1 0.94 0.8 0.05], S);
    saveFigure(fig, out_folder, 'Fig_NDOB_estimate.svg', output_pdf);
end

%% === Fig. 7 - THRUST ===
fig = figure;
ax = axes(fig); hold(ax, 'on');
lgd_handles = gobjects(0);
all_vals = [];
for i = valid
    v = D(i).Tp{:, "r_thr"};
    all_vals = [all_vals; v]; %#ok<AGROW>
    lgd_handles(end+1) = plot(ax, D(i).tp, v, 'LineWidth', line_w, 'Color', colors(i,:), ...
                              'DisplayName', sprintf('Q %d', i)); %#ok<SAGROW>
end
ylim(ax, padLimits(all_vals, 0.1));
addZones(ax, interact_zones, interact_cols);
styleAxes(ax, S);
xlabel(ax, 't (s)', 'FontWeight', 'bold', 'FontSize', font_size, 'FontName', font_name);
ylabel(ax, '||f_{thrust}|| (N)', 'FontWeight', 'bold', 'FontSize', font_size, 'FontName', font_name);
addTopLegend(lgd_handles, [0.15 0.94 0.7 0.05], S);
saveFigure(fig, out_folder, 'Fig7.svg', output_pdf);

%% === Fig. 8 - ABLASYON: İLERİ BESLEME VAR / YOK (fig8_ctrls, 50 ms pencere RMS) ===
if mod(numel(fig8_ctrls), 2) ~= 0
    error('fig8_ctrls çift sayıda koşu içermeli: (ileri beslemeli, ileri beslemesiz) çiftleri.');
end
f8_labels = cellfun(@ctrlLabel, fig8_ctrls, 'UniformOutput', false);
if all(f8_ok)
    fig = figure('Position', [100, 100, 1200, 560]);
    tl = tiledlayout(fig, 1, numel(fig8_drones), 'TileSpacing', 'compact', 'Padding', 'compact');
    for i = fig8_drones
        ax = nexttile(tl); hold(ax, 'on'); box(ax, 'on');
        hs = plotErrorLines(ax, cellfun(@(x) x(i), D_f8, 'UniformOutput', false), fig8_ctrls, fig2_window);
        styleErrorAxes(ax, i, phase_zones, c_phase, S);
    end
    % Ortak açıklama figürün üstünde, yatay
    lgd = legend(hs(isgraphics(hs)), f8_labels(isgraphics(hs)), 'Orientation', 'horizontal', ...
                 'FontSize', 18, 'FontWeight', 'bold', 'FontName', font_name, 'Box', 'off');
    lgd.Layout.Tile = 'north';
    saveFigure(fig, out_folder, 'Fig8.svg', output_pdf);
else
    warning('Fig. 8 / Table 2 atlandı: %s logları bulunamadı (log/ altında).', strjoin(fig8_ctrls(~f8_ok), ', '));
end

%% === Fig. 9 - UYUMSUZ KOMPANZATÖR: NDOB (Hakem 1) ===
% Üst satır: NDOB koşusunda simülatörün uyguladığı downwash kuvveti (dist_f.z) ve NDOB kestirimi (ndob_f.z).
%   NDOB downwash modelini kullanmaz; kestirim, modellenmemiş diğer etkileri (sürükleme vb.) de içerir.
% Alt satır: konum hatası; PID (geniş), PID+NDOB (orta), PID+FF (dar); renkler ctrlColor'dan.
if hasNdobColumns(D_obs) && any([D_none.ok]) && any([D_ff.ok])
    fig = figure('Position', [100, 100, 1200, 800]);
    nc = numel(fig9_drones);
    tl = tiledlayout(fig, 2, nc, 'TileSpacing', 'compact', 'Padding', 'compact');
    for c = 1:nc
        i = fig9_drones(c);
        ax = nexttile(tl, c); hold(ax, 'on'); box(ax, 'on');
        f_true = D_obs(i).Tp{:, "dist_f.z"};
        f_est  = D_obs(i).Tp{:, "ndob_f.z"};
        h1 = plot(ax, D_obs(i).tp, f_true, '-', 'Color', [0 0 0], 'LineWidth', line_w);
        h2 = plot(ax, D_obs(i).tp, f_est, '--', 'Color', ctrlColor(mismatch_ctrls{2}), 'LineWidth', line_w);
        ylim(ax, padLimits([f_true; f_est], 0.1));
        addZones(ax, interact_zones, interact_cols);
        styleAxes(ax, S);
        set(ax, 'FontSize', 16, 'XTick', 0:10:t_axis_max);
        xlabel(ax, 't (s)', 'FontWeight', 'bold', 'FontSize', 16, 'FontName', font_name);
        ylabel(ax, 'f_{dist,z} (N)', 'FontWeight', 'bold', 'FontSize', 16, 'FontName', font_name);
        lgd = legend(ax, [h1 h2], {'Applied downwash', 'NDOB estimate'}, 'Location', 'SouthWest', ...
                     'FontSize', 12, 'FontWeight', 'bold', 'FontName', font_name, 'Box', 'on');
        title(lgd, sprintf('Q %d', i));
    end
    for c = 1:nc
        i = fig9_drones(c);
        ax = nexttile(tl, nc + c); hold(ax, 'on'); box(ax, 'on');
        [hs, names] = errorBars(ax, {D_none(i), D_obs(i), D_ff(i)}, cellfun(@ctrlColor, mismatch_ctrls, 'UniformOutput', false), ...
                                cellfun(@ctrlLabel, mismatch_ctrls, 'UniformOutput', false), sparse_factor);
        finishBarAxes(ax, interact_zones, interact_cols, S, 16);
        barLegend(ax, hs, names, sprintf('Q %d', i), font_name);
        xlabel(ax, 't (s)', 'FontWeight', 'bold', 'FontSize', 16, 'FontName', font_name);
        ylabel(ax, '||e_p|| (m)', 'FontWeight', 'bold', 'FontSize', 16, 'FontName', font_name);
    end
    saveFigure(fig, out_folder, 'Fig9.svg', output_pdf);
else
    warning('Fig. 9 atlandı: %s logları bulunamadı (log/ altında).', strjoin(mismatch_ctrls, ', '));
end

%% === Fig. S5 - DRONE 1 HİBRİT YÖRÜNGE VE WAYPOINTLER ===
wp = [];
try
    wp = loadWaypointSets(config_dir, s5_drone);
catch err
    warning('Fig. S5 atlandı: waypointler okunamadı (%s).', err.message);
end
if ~isempty(wp) && D(s5_drone).ok
    plotFigS5(D(s5_drone), wp, colors(s5_drone,:), font_name, out_folder, output_pdf);
end

%% === Fig. S6 - DRONE 2 YÖNELİM HATASI (DCM), BETA KARŞILAŞTIRMASI ===
if D(s6_drone).ok && D_beta(s6_drone).ok
    fig = figure('Position', [100, 100, 800, 500]);
    ax = axes(fig); hold(ax, 'on'); box(ax, 'on');
    runs   = {D(s6_drone), D_beta(s6_drone)};
    cols   = {colors(s6_drone,:), dark_gray};
    widths = [0.5 0.3];
    alphas = [0.6 0.9];
    hs = gobjects(0);
    for k = 1:2
        e = dcmError(runs{k}.T);
        idx = 1:sparse_factor:numel(e);
        hs(end+1) = bar(ax, runs{k}.t(idx), e(idx), 'FaceColor', cols{k}, 'EdgeColor', 'none', ...
                        'BarWidth', widths(k), 'FaceAlpha', alphas(k)); %#ok<SAGROW>
    end
    finishBarAxes(ax, phase_zones, c_phase, S, 16);
    xlabel(ax, 't (s)', 'FontWeight', 'bold', 'FontSize', 16, 'FontName', font_name);
    ylabel(ax, 'Orientation Error (rad)', 'FontWeight', 'bold', 'FontSize', 16, 'FontName', font_name);
    legend(ax, hs, {sprintf('\\beta = %g', beta_main), sprintf('\\beta = %g', beta_alt)}, ...
           'Location', 'NorthEast', 'FontSize', 12, 'FontWeight', 'bold', 'Box', 'on');
    saveFigure(fig, out_folder, 'FigS6.svg', output_pdf);
else
    warning('Fig. S6 atlandı: beta logları bulunamadı (%s).', beta_log_dir);
end

%% === FORMASYON ÖLÇÜTLERİ (Fig. F1, Table 3) ===
% Komşu çift (i,j) için (komşu: istenen mesafe <= neighbor_radius):
%   bağıl mesafe hatası  e_d,ij = ||p_i - p_j|| - ||p_i^ref - p_j^ref||
%   formasyon hatası     e_ij   = (p_i - p_j) - (p_i^ref - p_j^ref)
% Her an komşu çiftler üzerinden RMS alınır: E_d(t), E_F(t).
runs_F   = cellfun(@(n) run_data(n), formation_ctrls, 'UniformOutput', false);
labels_F = cellfun(@ctrlLabel, formation_ctrls, 'UniformOutput', false);
cols_F   = cellfun(@ctrlColor, formation_ctrls, 'UniformOutput', false);
F = cell(1, 3);
for r = 1:3
    if all([runs_F{r}.ok])
        F{r} = formationMetrics(runs_F{r}, neighbor_radius);
    end
end
avail_F = find(~cellfun(@isempty, F));

fig = figure('Position', [100, 100, 1000, 700]);
axs = stackedAxes(fig, 2);
fields_F  = {'Ed', 'EF'};
ylabels_F = {'E_d (m)', 'E_F (m)'};
hs = gobjects(1, numel(formation_ctrls));
[~, ord_F] = sort(cellfun(@ctrlLineWidth, formation_ctrls(avail_F)), 'descend');   % kalından inceye çiz
for a = 1:2
    ax = axs(a); hold(ax, 'on');
    for r = avail_F(ord_F)
        idx = 1:plot_step:numel(F{r}.t);
        h = plot(ax, F{r}.t(idx), F{r}.(fields_F{a})(idx), ctrlLineStyle(formation_ctrls{r}), ...
                 'LineWidth', ctrlLineWidth(formation_ctrls{r}), 'Color', cols_F{r}, ...
                 'DisplayName', labels_F{r});
        if a == 1, hs(r) = h; end
    end
    yl = ylim(ax); ylim(ax, [0 yl(2) * 1.05]);
    addZones(ax, phase_zones, c_phase);
    styleAxes(ax, S);
    ylabel(ax, ylabels_F{a}, 'FontWeight', 'bold', 'FontSize', font_size, 'FontName', font_name);
    xlabel(ax, 't (s)', 'FontWeight', 'bold', 'FontSize', font_size, 'FontName', font_name);
end
addTopLegend(hs(isgraphics(hs)), [0.1 0.94 0.8 0.05], S);   % açıklama formation_ctrls sırasıyla
saveFigure(fig, out_folder, 'FigF1_formation_error.svg', output_pdf);

% Table 3: İHA çiftleri matrisi (her kontrolcü için). Satır/sütun Q1..Q5;
%   köşegenin üstü: formasyon hatası RMS ||e_ij|| (m) ve parantez içinde baskın yönü:
%       (z) dikey bileşen, (xy) yatay bileşen daha büyük. Downwash dikey olduğundan (z) bozucunun izidir.
%   köşegenin altı: bağıl mesafe hatası RMS e_d,ij (m) ve parantez içinde işaretli ortalaması:
%       (+) çift istenenden uzak, (-) istenenden yakın (çarpışma yönü), ~0 ise yönsüz (gürültü benzeri) hata.
%   RMS işareti ve yönü kaybettiği için bu ekler yönü gösterir. e_d, e_ij'nin iki İHA'yı birleştiren
%   doğrultudaki bileşenidir; bu yüzden alt üçgen üst üçgenden büyük olamaz, aradaki fark formasyonun
%   mesafeyi değiştirmeden bozulmasıdır (ör. yan yana iki İHA'dan birinin aşağı kayması).
% Değerler çiftin komşu olduğu süre boyunca hesaplanır; hiç komşu olmayan çiftler '-' ile gösterilir.
names_Q = "Q" + string(1:num_drones);
file_tag = formation_ctrls;
for r = avail_F
    M = repmat("-", num_drones, num_drones);
    for k = 1:size(F{r}.pairs, 1)
        i1 = F{r}.pairs(k, 1); j1 = F{r}.pairs(k, 2);
        if ~isnan(F{r}.pair_EF(k))
            if F{r}.pair_EF_z(k) >= F{r}.pair_EF_xy(k), dir_F = 'z'; else, dir_F = 'xy'; end
            mean_d = sprintf('%+.3f', F{r}.pair_Ed_mean(k));
            if abs(F{r}.pair_Ed_mean(k)) < 5e-4, mean_d = '0.000'; end      % '+0.000' / '-0.000' yazılmasın
            M(i1, j1) = sprintf('%.3f (%s)', F{r}.pair_EF(k), dir_F);        % üst üçgen: formasyon hatası
            M(j1, i1) = sprintf('%.3f (%s)', F{r}.pair_Ed(k), mean_d);       % alt üçgen: bağıl mesafe hatası
        end
    end
    tbl3 = array2table(M, 'RowNames', cellstr(names_Q), 'VariableNames', cellstr(names_Q));
    fprintf('\nTable 3 (%s), m\n', labels_F{r});
    fprintf('  üst üçgen: formasyon hatası RMS (baskın yön: xy yatay, z dikey)\n');
    fprintf('  alt üçgen: bağıl mesafe hatası RMS (işaretli ortalama: + istenenden uzak, - istenenden yakın)\n');
    disp(tbl3);
    writetable(tbl3, fullfile(out_folder, sprintf('Table3_formation_%s.csv', file_tag{r})), 'WriteRowNames', true);
end

%% === Table 1 / Table 2 - PERFORMANS METRİKLERİ ===
M = metrics(D);
fprintf('\n=====================================================================\n');
fprintf('   %s (RMSE: ortalama, ISE: toplam, MAX: en büyük)\n', ctrlLabel(main_ctrl));
fprintf('=====================================================================\n');
for i = 1:num_drones
    if D(i).ok
        fprintf('Q %d -> RMSE: %.5f m | ISE: %.5f m^2s | MAX: %.5f m\n', i, M.rmse(i), M.ise(i), M.max(i));
    else
        fprintf('Q %d: Dosya bulunamadı.\n', i);
    end
end

if all(f2_ok)
    tbl1 = multiComparisonTable(cellfun(@metrics, D_f2, 'UniformOutput', false), ...
                                cellfun(@ctrlVar, fig2_ctrls, 'UniformOutput', false));
    fprintf('\nTable 1: %s (fark %% = (diğeri - %s) / diğeri * 100)\n', strjoin(f2_labels, ' vs '), f2_labels{1});
    disp(tbl1);
    writetable(tbl1, fullfile(out_folder, 'Table1.csv'));
end
if all(f8_ok) && mod(numel(fig8_ctrls), 2) == 0
    tbl2 = ablationTable(cellfun(@metrics, D_f8, 'UniformOutput', false), ...
                         cellfun(@ctrlVar, fig8_ctrls, 'UniformOutput', false));
    fprintf('\nTable 2: Ablasyon - ileri beslemeli / ileri beslemesiz çiftler: %s (iyileşme %% = (ileri beslemesiz - ileri beslemeli) / ileri beslemesiz * 100)\n', ...
            strjoin(f8_labels, ', '));
    disp(tbl2);
    writetable(tbl2, fullfile(out_folder, 'Table2.csv'));
end

%% === Table 4 - UYUMSUZ KOMPANZATÖR (Hakem 1) ===
% Model tabanlı FF (simülatörle aynı model), NDOB (model yok, gözlemci kestirimi) ve kompanzasyonsuz koşu.
%   dRMSE        = RMSE - bozucusuz RMSE (downwash'ın yarattığı ek hata)
%   PID_NDOB_vs_PID_FF_pct = PID+NDOB'un PID+FF'ye göre RMSE artışı (%)
%   residual_pct = kompanzasyondan sonra kalan downwash hatası / kompanzasyonsuz downwash hatası (%);
%                  yalnızca downwash'a maruz kalan İHA'lar için (kompanzasyonsuz dRMSE > 0.1 mm).
if any([D_none.ok]) && any([D_obs.ok]) && any([D_ff.ok])
    M_ndob = metrics(D_obs);
    M_none = metrics(D_none);
    M_ff   = metrics(D_ff);
    if any([D_nom.ok])
        M_nom = metrics(D_nom);
    else
        M_nom = struct('rmse', nan(num_drones, 1));
        warning('Table 4: bozucusuz loglar bulunamadı (%s); dRMSE sütunları NaN.', nominal_log_dir);
    end
    d_none = M_none.rmse - M_nom.rmse;
    d_ndob = M_ndob.rmse - M_nom.rmse;
    d_ff   = M_ff.rmse - M_nom.rmse;
    disturbed = d_none > 1e-4;
    res_ndob = nan(num_drones, 1); res_ff = res_ndob;
    res_ndob(disturbed) = d_ndob(disturbed) ./ d_none(disturbed) * 100;
    res_ff(disturbed)   = d_ff(disturbed) ./ d_none(disturbed) * 100;
    Quadcopter = "Q " + string((1:num_drones)');
    v = cellfun(@ctrlVar, mismatch_ctrls, 'UniformOutput', false);
    tbl4 = table(Quadcopter, M_nom.rmse, M_none.rmse, M_ndob.rmse, M_ff.rmse, d_none, d_ndob, d_ff, ...
                 (M_ndob.rmse - M_ff.rmse) ./ M_ff.rmse * 100, res_ndob, res_ff, ...
                 'VariableNames', {'Quadcopter', 'DisturbanceFree_RMSE_m', [v{1} '_RMSE_m'], ...
                                   [v{2} '_RMSE_m'], [v{3} '_RMSE_m'], [v{1} '_dRMSE_m'], [v{2} '_dRMSE_m'], ...
                                   [v{3} '_dRMSE_m'], [v{2} '_vs_' v{3} '_pct'], [v{2} '_residual_pct'], ...
                                   [v{3} '_residual_pct']});
    fprintf('\nTable 4: Uyumsuz kompanzatör - %s\n', strjoin(cellfun(@ctrlLabel, mismatch_ctrls, 'UniformOutput', false), ' / '));
    disp(tbl4);
    writetable(tbl4, fullfile(out_folder, 'Table4_compensator_mismatch.csv'));

    % NDOB kestirim hatası (z), downwash etkinken (|f_dist,z| > 0.05 N)
    if hasNdobColumns(D_obs)
        for i = fig9_drones
            if ~D_obs(i).ok, continue; end
            f_true = D_obs(i).T{:, "dist_f.z"};
            f_est  = D_obs(i).T{:, "ndob_f.z"};
            act = abs(f_true) > 0.05;
            if ~any(act), continue; end
            fprintf('NDOB kestirimi Q %d (downwash etkin, %.1f s): RMS hata %.3f N, RMS downwash %.3f N (%%%.0f)\n', ...
                    i, sum(act) * mean(diff(D_obs(i).t)), sqrt(mean((f_est(act) - f_true(act)).^2)), ...
                    sqrt(mean(f_true(act).^2)), ...
                    100 * sqrt(mean((f_est(act) - f_true(act)).^2)) / sqrt(mean(f_true(act).^2)));
        end
    end
else
    warning('Table 4 atlandı: %s logları bulunamadı (log/ altında).', strjoin(mismatch_ctrls, ', '));
end
%% === Table S2 / Table S3 - SİMÜLASYON PARAMETRELERİ VE KONTROLCÜ KAZANÇLARI ===
% Değerler doğrudan config/*.yaml ve launch/sequential_sim.launch dosyalarından okunur.
try
    [tblS2, tblS3] = parameterTables(config_dir, fullfile(pkg_dir, 'launch', 'sequential_sim.launch'));
    fprintf('\nTable S2: Simülasyon parametreleri\n');
    disp(tblS2);
    writetable(tblS2, fullfile(out_folder, 'TableS2_parameters.csv'));
    fprintf('\nTable S3: Kontrolcü kazançları\n');
    disp(tblS3);
    writetable(tblS3, fullfile(out_folder, 'TableS3_gains.csv'));
catch err
    warning('Table S2 / S3 atlandı: %s', err.message);
end

fprintf('=====================================================================\n');
fprintf('Grafikler "%s" klasörüne ve "%s" dosyasına kaydedildi.\n', out_folder, output_pdf);


%% =====================================================================
%% YARDIMCI FONKSİYONLAR
%% =====================================================================
function D = readLogs(dir_path, num_drones, plot_step)
    % droneN_log.txt dosyalarını okur; bulunamayanlar ok = false.
    % T/t: tam veri (metrikler, çubuklar), Tp/tp: çizgi grafikleri için seyreltilmiş veri
    D = repmat(struct('ok', false, 't', [], 'T', [], 'tp', [], 'Tp', []), num_drones, 1);
    if isempty(dir_path) || ~isfolder(dir_path), return; end
    for i = 1:num_drones
        filename = fullfile(dir_path, sprintf('drone%d_log.txt', i));
        if ~isfile(filename), continue; end
        T = readtable(filename, 'VariableNamingRule', 'preserve');
        D(i).ok = true;
        D(i).T  = T;
        D(i).t  = T{:, "time"} - T{1, "time"};
        D(i).Tp = T(1:plot_step:end, :);
        D(i).tp = D(i).t(1:plot_step:end);
    end
end

function checkRunInfo(dir_path, D, label, ex)
    % run_info.txt: koşunun kontrolcüsü, bozucu ve beta ayarı ve terminalde basılan RMS değerleri.
    % Beklenen ayarlar (ex, bkz. expectedRun) ve loglardan hesaplanan RMS ile karşılaştırılır; fark ya da
    % dosyanın olmaması logların başka bir koşudan geldiğini gösterir.
    if isempty(dir_path) || ~any([D.ok]), return; end
    info_file = fullfile(dir_path, 'run_info.txt');
    if ~isfile(info_file)
        warning('%s: %s yok. Loglar sıralı simülasyondan değil (eski ya da başka bir kaynaktan) olabilir.', ...
                label, info_file);
        return;
    end
    txt = fileread(info_file);
    ctrl = regexp(txt, 'controller:\s*(\S+)', 'tokens', 'once');
    dt   = regexp(txt, 'date:\s*([^\n]+)', 'tokens', 'once');
    if isempty(ctrl), ctrl = {'?'}; end
    if isempty(dt), dt = {'?'}; end
    fprintf('%s: %s | controller = %s | %s\n', label, dir_path, ctrl{1}, strtrim(dt{1}));
    if nargin > 3
        dist = regexp(txt, 'disturbance_enabled:\s*(\S+)', 'tokens', 'once');
        beta = regexp(txt, 'psi_filter_beta:\s*([-0-9.eE+]+)', 'tokens', 'once');
        if ~strcmp(ctrl{1}, ex.ctrl)
            warning('%s: koşu controller = %s, beklenen %s. Doğru klasör mü?', label, ctrl{1}, ex.ctrl);
        end
        if ~isempty(dist) && strcmp(dist{1}, 'true') ~= ex.dist
            warning('%s: disturbance_enabled = %s, beklenen %d.', label, dist{1}, ex.dist);
        end
        if ~isempty(beta) && abs(str2double(beta{1}) - ex.beta) > 1e-9
            warning('%s: psi_filter_beta = %s, beklenen %g.', label, beta{1}, ex.beta);
        end
    end
    for i = 1:numel(D)
        if ~D(i).ok, continue; end
        tok = regexp(txt, sprintf('drone%d_rms:\\s*([-0-9.eE+]+)', i), 'tokens', 'once');
        if isempty(tok), continue; end
        rms_sim = str2double(tok{1});
        rms_log = sqrt(mean(positionError(D(i).T).^2));
        if abs(rms_sim - rms_log) > 1e-4
            warning('%s: drone%d RMS logda %.5f, simülasyonda %.5f. Loglar bu koşuya ait değil olabilir.', ...
                    label, i, rms_log, rms_sim);
        end
    end
end

function e = positionError(T)
    e = sqrt((T{:, "a_pos.x"} - T{:, "r_pos.x"}).^2 + ...
             (T{:, "a_pos.y"} - T{:, "r_pos.y"}).^2 + ...
             (T{:, "a_pos.z"} - T{:, "r_pos.z"}).^2);
end

function ex = expectedRun(name, beta_default)
    % Koşu adından beklenen ayarlar: <kontrolcü>[_nominal | _beta<değer>]
    ex.ctrl = regexprep(name, '_(nominal|beta[-0-9.eE+]*)$', '');
    ex.dist = ~endsWith(name, '_nominal');
    tok = regexp(name, '_beta([-0-9.eE+]+)$', 'tokens', 'once');
    if isempty(tok), ex.beta = beta_default; else, ex.beta = str2double(tok{1}); end
end

function s = ctrlLabel(name)
    % Grafik etiketi: pid_ff -> PID+FF, se3 -> SE(3), pid_nominal -> PID (no disturbance), ...
    ex = expectedRun(name, NaN);
    switch ex.ctrl
        case 'pid_ff', s = 'PID+FF';
        case 'pid',    s = 'PID';
        case 'ndob',   s = 'PID+NDOB';
        case 'se3',    s = 'SE(3)';
        case 'se3_ff', s = 'SE(3)+FF';
        otherwise,     s = ex.ctrl;
    end
    if ~ex.dist, s = [s ' (no disturbance)']; end
    if ~isnan(ex.beta), s = sprintf('%s (\\beta = %g)', s, ex.beta); end
end

function s = ctrlVar(name)
    % Tablo sütun adı: PID+FF -> PID_FF, SE(3)+FF -> SE3_FF
    s = regexprep(strrep(ctrlLabel(name), '(3)', '3'), '[^A-Za-z0-9]+', '_');
    s = regexprep(s, '^_+|_+$', '');
end

function tf = hasNdobColumns(Dx)
    % Log, controller:=ndob koşusundan mı? (ndob_f.x sütunu var mı)
    v = find([Dx.ok], 1);
    tf = ~isempty(v) && ismember("ndob_f.x", Dx(v).T.Properties.VariableNames);
end

function M = metrics(D)
    n = numel(D);
    M = struct('rmse', nan(n, 1), 'ise', nan(n, 1), 'max', nan(n, 1));
    for i = 1:n
        if ~D(i).ok, continue; end
        sq = positionError(D(i).T).^2;
        dt = [0; diff(D(i).t)];
        M.rmse(i) = sqrt(mean(sq));
        M.ise(i)  = sum(sq .* dt);
        M.max(i)  = sqrt(max(sq));
    end
end

function [t_b, e_b] = windowRms(D1, window)
    % Konum hatasının ardışık, örtüşmeyen pencerelerdeki RMS'i (pencere ortası zamanında).
    % Pencere referansın güncelleme periyoduna eşitse (20 Hz -> 0.05 s) testere dişi düzlenir ve
    % pencere RMS'lerinin karesel ortalaması RMSE'ye eşit olur.
    e  = positionError(D1.T);
    dt = median(diff(D1.t));
    N  = max(1, round(window / dt));
    n  = floor(numel(e) / N);
    e_b = sqrt(mean(reshape(e(1:n * N).^2, N, n), 1))';
    t_b = mean(reshape(D1.t(1:n * N), N, n), 1)';
end

function c = ctrlColor(name)
    % Kontrolcü başına sabit, canlı (doygun) renk (Fig. 2, 8, 9, F1): mavi, siyah, turuncu, yeşil, mor;
    % birbirine yakın ton yok ve açık renkli faz bölgelerinin üstünde de belirgin.
    ex = expectedRun(name, NaN);
    switch ex.ctrl
        case 'pid_ff', c = [0.00 0.40 1.00];   % canlı mavi
        case 'se3_ff', c = [0.00 0.00 0.00];   % siyah
        case 'ndob',   c = [1.00 0.50 0.00];   % turuncu
        case 'pid',    c = [0.00 0.65 0.00];   % yeşil
        case 'se3',    c = [0.50 0.00 0.75];   % mor (kesikli çizilir)
        otherwise,     c = [0.55 0.35 0.15];   % kahverengi
    end
end

function w = ctrlLineWidth(name)
    % Katmanlı kalınlık: altta çizilenler kalın, üsttekiler ince; üst üste binen eğriler iç içe
    % şeritler olarak birlikte görünür (çizim sırası: PID, PID+NDOB, SE(3)+FF, PID+FF, kesikli SE(3)).
    ex = expectedRun(name, NaN);
    switch ex.ctrl
        case 'pid',    w = 3.5;
        case 'ndob',   w = 3.0;
        case 'se3_ff', w = 2.5;
        case 'pid_ff', w = 2.0;
        case 'se3',    w = 2.2;
        otherwise,     w = 2.0;
    end
end

function ls = ctrlLineStyle(name)
    % SE(3) kesikli: bozucu altında PID ile, bozucusuz İHA'larda SE(3)+FF ile üst üste biner
    ex = expectedRun(name, NaN);
    if strcmp(ex.ctrl, 'se3'), ls = '--'; else, ls = '-'; end
end

function axs = stackedAxes(fig, n)
    % n satırlık dikey eksen düzeni; her eksenin kendi x etiketi sığsın diye satırlar arasında
    % sabit piksel boşluğu bırakır (üstte ortak açıklama için yer).
    H = fig.Position(4);
    top = 1 - 95 / H; bottom = 85 / H; gap = 95 / H; left = 0.12; width = 0.85;
    h = (top - bottom - (n - 1) * gap) / n;
    axs = gobjects(n, 1);
    for a = 1:n
        axs(a) = axes(fig, 'Position', [left, top - a * h - (a - 1) * gap, width, h]);
    end
end

function tbl = ablationTable(Ms, names)
    % Çiftler sırayla (ileri beslemeli, ileri beslemesiz): her ikisinin RMSE/ISE'si ve ileri beslemenin
    % sağladığı iyileşme (%) = (ileri beslemesiz - ileri beslemeli) / ileri beslemesiz * 100
    n = numel(Ms{1}.rmse);
    tbl = table("Q " + string((1:n)'), 'VariableNames', {'Quadcopter'});
    for k = 1:2:numel(Ms) - 1
        A = Ms{k}; B = Ms{k + 1};
        tbl.([names{k} '_RMSE_m'])     = A.rmse;
        tbl.([names{k} '_ISE'])        = A.ise;
        tbl.([names{k + 1} '_RMSE_m']) = B.rmse;
        tbl.([names{k + 1} '_ISE'])    = B.ise;
        tbl.([names{k} '_vs_' names{k + 1} '_RMSE_pct']) = (B.rmse - A.rmse) ./ B.rmse * 100;
        tbl.([names{k} '_vs_' names{k + 1} '_ISE_pct'])  = (B.ise - A.ise) ./ B.ise * 100;
    end
end

function hs = plotErrorLines(ax, runs, names, window)
    % Konum hatasının pencere RMS'i (bkz. windowRms), kontrolcü başına sabit renk / kalınlık / çizgi tipiyle.
    % Çizim sırası: düz çizgiler kalından inceye, sonra kesikli çizgiler; üst üste binen eğriler iç içe
    % şeritler olarak birlikte görünür. hs names sırasındadır (çizilmeyenler boş grafik nesnesi).
    hs = gobjects(1, numel(names));
    is_dash = cellfun(@(n) strcmp(ctrlLineStyle(n), '--'), names);
    w = cellfun(@ctrlLineWidth, names);
    solid = find(~is_dash);
    [~, o] = sort(w(solid), 'descend');
    for k = [solid(o), find(is_dash)]
        if ~runs{k}.ok, continue; end
        [tb, eb] = windowRms(runs{k}, window);
        hs(k) = plot(ax, tb, eb, ctrlLineStyle(names{k}), 'Color', ctrlColor(names{k}), 'LineWidth', w(k));
    end
end

function styleErrorAxes(ax, drone, zones, zone_cols, S)
    % Fig. 2 ve Fig. 8 panelleri: faz bölgeleri, eksen etiketleri ve İHA etiketi (sağ üstte, çerçevesiz,
    % saydam; uçuşun sonunda hata küçük olduğundan eğrilerle çakışmaz)
    yl = ylim(ax); ylim(ax, [0 yl(2) * 1.05]);
    addZones(ax, zones, zone_cols);
    xlim(ax, [0 S.t_max]);
    set(ax, 'XTick', 0:10:S.t_max, 'FontName', S.font_name, 'FontSize', 16, 'FontWeight', 'bold', ...
        'LineWidth', 1.5, 'Layer', 'top', 'Box', 'on');
    text(ax, 0.985, 0.96, sprintf('Q %d', drone), 'Units', 'normalized', 'HorizontalAlignment', 'right', ...
         'VerticalAlignment', 'top', 'FontWeight', 'bold', 'FontSize', 18, 'FontName', S.font_name, ...
         'BackgroundColor', 'none', 'EdgeColor', 'none', 'Margin', 2);
    xlabel(ax, 't (s)', 'FontWeight', 'bold', 'FontSize', 18, 'FontName', S.font_name);
    ylabel(ax, '||e_p|| (m)', 'FontWeight', 'bold', 'FontSize', 18, 'FontName', S.font_name);
end

function tbl = multiComparisonTable(Ms, names)
    % Her koşunun RMSE/ISE'si ve ilk koşunun diğerlerine göre iyileşmesi (%) = (B - A) / B * 100
    n = numel(Ms{1}.rmse);
    tbl = table("Q " + string((1:n)'), 'VariableNames', {'Quadcopter'});
    for k = 1:numel(Ms)
        tbl.([names{k} '_RMSE_m']) = Ms{k}.rmse;
        tbl.([names{k} '_ISE'])    = Ms{k}.ise;
    end
    A = Ms{1};
    for k = 2:numel(Ms)
        B = Ms{k};
        tbl.([names{1} '_vs_' names{k} '_RMSE_pct']) = (B.rmse - A.rmse) ./ B.rmse * 100;
        tbl.([names{1} '_vs_' names{k} '_ISE_pct'])  = (B.ise - A.ise) ./ B.ise * 100;
    end
end

function e = dcmError(T)
    % Geodesic yönelim hatası: acos((trace(R_ref' * R_act) - 1) / 2), Z-Y-X Euler
    Rr = eulerToRotCols(T{:, "r_eul_ang.x"}, T{:, "r_eul_ang.y"}, T{:, "r_eul_ang.z"});
    Ra = eulerToRotCols(T{:, "a_eul_ang.x"}, T{:, "a_eul_ang.y"}, T{:, "a_eul_ang.z"});
    tr = sum(Rr .* Ra, 2);                    % trace(Rr' * Ra) = sum_ij Rr_ij * Ra_ij
    e  = acos(max(min((tr - 1) / 2, 1), -1));
end

function R = eulerToRotCols(phi, theta, psi)
    % R = Rz(psi) * Ry(theta) * Rx(phi); her satır [R11 R12 R13 R21 R22 R23 R31 R32 R33]
    cf = cos(phi); sf = sin(phi); ct = cos(theta); st = sin(theta); cp = cos(psi); sp = sin(psi);
    R = [cp.*ct, cp.*st.*sf - sp.*cf, cp.*st.*cf + sp.*sf, ...
         sp.*ct, sp.*st.*sf + cp.*cf, sp.*st.*cf - cp.*sf, ...
         -st,    ct.*sf,              ct.*cf];
end

function [hs, names] = errorBars(ax, runs, cols, labels, sparse_factor)
    % Seyreltilmiş konum hatası çubukları (ilk koşu geniş/renkli, ikinci dar/koyu)
    widths = [1.0 0.5 0.25];
    alphas = [0.6 0.9 0.9];
    hs = gobjects(0); names = {};
    for k = 1:numel(runs)
        if ~runs{k}.ok, continue; end
        e = positionError(runs{k}.T);
        idx = 1:sparse_factor:numel(e);
        hs(end+1) = bar(ax, runs{k}.t(idx), e(idx), 'FaceColor', cols{k}, 'EdgeColor', 'none', ...
                        'BarWidth', widths(k), 'FaceAlpha', alphas(k)); %#ok<AGROW>
        names{end+1} = labels{k}; %#ok<AGROW>
    end
end

function finishBarAxes(ax, zones, zone_cols, S, fs)
    yl = ylim(ax);
    ylim(ax, [0 yl(2) * 1.05]);
    addZones(ax, zones, zone_cols);
    xlim(ax, [0 S.t_max]);
    set(ax, 'XTick', 0:10:S.t_max, 'FontName', S.font_name, 'FontSize', fs, 'FontWeight', 'bold', ...
        'LineWidth', 1.5, 'Layer', 'top', 'Box', 'on');
end

function barLegend(ax, hs, names, ttl, font_name)
    if isempty(hs), return; end
    lgd = legend(ax, hs, names, 'Location', 'NorthEast', 'FontSize', 12, 'FontWeight', 'bold', ...
                 'FontName', font_name, 'Box', 'on');
    title(lgd, ttl);
end

function plotStateFigure(D, valid, act_prefix, ref_prefix, ylabels, zones, zone_cols, colors, S, ...
                         out_folder, out_name, output_pdf, show_ref)
    % 3 eksenli gerçek (renkli) + istenen (siyah noktalı, üstte) zaman grafiği.
    % show_ref = false: istenen değerler çizilmez (Fig. 5, Euler açıları).
    if nargin < 13, show_ref = true; end
    fig = figure('Position', [100, 100, 1000, 900]);
    axs = stackedAxes(fig, 3);
    axis_names = ["x", "y", "z"];
    lgd_handles = gobjects(0);
    for a = 1:3
        ax = axs(a); hold(ax, 'on');
        if a == 1 && show_ref
            lgd_handles(end+1) = plot(ax, NaN, NaN, ':', 'Color', S.ref_col, 'LineWidth', S.ref_w, ...
                                      'DisplayName', 'Desired'); %#ok<AGROW>
        end
        all_vals = [];
        V = cell(1, numel(D)); R = V;
        for i = valid
            v = D(i).Tp{:, act_prefix + axis_names(a)};
            r = D(i).Tp{:, ref_prefix + axis_names(a)};
            if act_prefix == "a_eul_ang." && a == 3
                % yaw: gerçek açı +/-pi'ye sarılı, referans sürekli -> ikisini de sürekli çiz
                v = unwrap(v); r = unwrap(r);
                k0 = round((r(1) - v(1)) / (2 * pi)); v = v + 2 * pi * k0;
            end
            V{i} = v; R{i} = r;
            all_vals = [all_vals; v]; %#ok<AGROW>
            if show_ref, all_vals = [all_vals; r]; end %#ok<AGROW>
        end
        for i = valid
            h = plot(ax, D(i).tp, V{i}, '-', 'Color', colors(i,:), 'LineWidth', S.line_w, ...
                     'DisplayName', sprintf('Q %d', i));
            if a == 1, lgd_handles(end+1) = h; else, h.HandleVisibility = 'off'; end %#ok<AGROW>
        end
        if show_ref
            for i = valid                         % istenen değerler (üstte, siyah noktalı)
                plot(ax, D(i).tp, R{i}, ':', 'Color', S.ref_col, 'LineWidth', S.ref_w, 'HandleVisibility', 'off');
            end
        end
        ylim(ax, padLimits(all_vals, 0.1));
        addZones(ax, zones, zone_cols);
        styleAxes(ax, S);
        ylabel(ax, ylabels(a), 'FontWeight', 'bold', 'FontSize', S.font_size, 'FontName', S.font_name);
        xlabel(ax, 't (s)', 'FontWeight', 'bold', 'FontSize', S.font_size, 'FontName', S.font_name);
    end
    addTopLegend(lgd_handles, [0.1 0.94 0.8 0.05], S);
    saveFigure(fig, out_folder, out_name, output_pdf);
end

function wp = loadWaypointSets(config_dir, drone)
    % trajectory_manager_droneN.yaml + all_trajectories.yaml dosyalarından waypoint setlerini okur
    mgr = fileread(fullfile(config_dir, sprintf('trajectory_manager_drone%d.yaml', drone)));
    lst = regexp(mgr, 'active_trajectories:\s*\[(.*?)\]', 'tokens', 'once');
    names = regexp(lst{1}, '"([^"]+)"', 'tokens');
    names = cellfun(@(c) c{1}, names, 'UniformOutput', false);

    all_txt = fileread(fullfile(config_dir, 'all_trajectories.yaml'));
    wp = struct('name', {}, 'x', {}, 'y', {}, 'z', {}, 'method', {});
    for k = 1:numel(names)
        pattern = ['\n\s*' regexptranslate('escape', names{k}) ':\s*\n(.*?)method:\s*"([^"]+)"'];
        blk = regexp(all_txt, pattern, 'tokens', 'once');
        if isempty(blk), error('%s all_trajectories.yaml içinde bulunamadı', names{k}); end
        wp(k).name   = names{k};
        wp(k).x      = yamlArray(blk{1}, 'points_x');
        wp(k).y      = yamlArray(blk{1}, 'points_y');
        wp(k).z      = yamlArray(blk{1}, 'points_z');
        wp(k).method = blk{2};
    end
end

function v = yamlArray(txt, key)
    tok = regexp(txt, [key ':\s*\[([^\]]*)\]'], 'tokens', 'once');
    v = str2double(strsplit(strtrim(tok{1}), ','));
end

function plotFigS5(Dd, wp, path_color, font_name, out_folder, output_pdf)
    % Hibrit yörünge: 3B ve XY görünüm, waypoint setleri ve atlanan ara noktalar (numaralı)
    wp_colors = {[1 0.2 0.2], [0.2 0.6 1], [0.2 0.8 0.2], [0.9 0.8 0]};
    T = Dd.Tp;
    px = T{:, "a_pos.x"}; py = T{:, "a_pos.y"}; pz = T{:, "a_pos.z"};

    % Atlanan waypointler: yeni set geçiş segmentinin başında alındığı için sonraki noktalara
    % ulaşılmaz (cubic_spline: segment 3, diğerleri: segment 5; ilk set dışındakilerin başına
    % mevcut referans noktası eklenir). Son set tamamen uçulur.
    skipped = cell(numel(wp), 1);
    for k = 1:numel(wp) - 1
        if strcmp(wp(k).method, 'cubic_spline'), s = 3; else, s = 5; end
        first_skipped = s + 2 - double(k > 1);    % 1 tabanlı indeks
        skipped{k} = first_skipped:numel(wp(k).x);
    end

    fig = figure('Position', [100, 100, 1200, 550]);
    tl = tiledlayout(fig, 1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    for view_k = 1:2
        ax = nexttile(tl); hold(ax, 'on'); box(ax, 'on');
        if view_k == 1
            plot3(ax, px, py, pz, '-', 'Color', path_color, 'LineWidth', 3.5);
        else
            plot(ax, px, py, '-', 'Color', path_color, 'LineWidth', 3.5);
        end
        label_no = 0;
        for k = 1:numel(wp)
            c = wp_colors{mod(k - 1, numel(wp_colors)) + 1};
            if view_k == 1
                scatter3(ax, wp(k).x, wp(k).y, wp(k).z, 100, c, 's', 'filled', 'MarkerEdgeColor', 'k');
            else
                scatter(ax, wp(k).x, wp(k).y, 100, c, 's', 'filled', 'MarkerEdgeColor', 'k');
            end
            for j = skipped{k}
                label_no = label_no + 1;
                if view_k == 1
                    text(ax, wp(k).x(j), wp(k).y(j), wp(k).z(j), sprintf('  %d', label_no), ...
                         'FontName', font_name, 'FontSize', 16, 'FontWeight', 'bold');
                else
                    text(ax, wp(k).x(j), wp(k).y(j), sprintf('  %d', label_no), ...
                         'FontName', font_name, 'FontSize', 16, 'FontWeight', 'bold');
                end
            end
        end
        if view_k == 1
            scatter3(ax, px(1), py(1), pz(1), 300, 'k', 'p', 'filled', 'MarkerEdgeColor', 'k', 'LineWidth', 1.5);
            zlabel(ax, 'z (m)', 'FontWeight', 'bold', 'FontSize', 22, 'FontName', font_name);
            view(ax, 45, 30);
        else
            scatter(ax, px(1), py(1), 300, 'g', 'p', 'filled', 'MarkerEdgeColor', 'k', 'LineWidth', 1.5);
            view(ax, 0, 90);
        end
        xlabel(ax, 'x (m)', 'FontWeight', 'bold', 'FontSize', 22, 'FontName', font_name);
        ylabel(ax, 'y (m)', 'FontWeight', 'bold', 'FontSize', 22, 'FontName', font_name);
        grid(ax, 'on'); axis(ax, 'equal'); axis(ax, 'tight');
        set(ax, 'FontName', font_name, 'FontSize', 20, 'FontWeight', 'bold', 'LineWidth', 2.0);
    end
    saveFigure(fig, out_folder, 'FigS5.svg', output_pdf);
end

function F = formationMetrics(D, r_N)
    % Logları ortak zaman eksenine getirir ve formationCore'u çağırır
    valid = find([D.ok]);
    t = D(valid(1)).t;
    N = numel(D);
    Pa = nan(numel(t), 3, N);
    Pr = Pa;
    for i = valid
        Ti = D(i).T; ti = D(i).t;
        a = [Ti{:, "a_pos.x"}, Ti{:, "a_pos.y"}, Ti{:, "a_pos.z"}];
        r = [Ti{:, "r_pos.x"}, Ti{:, "r_pos.y"}, Ti{:, "r_pos.z"}];
        if numel(ti) ~= numel(t) || any(abs(ti - t) > 1e-9)
            a = interp1(ti, a, t, 'linear', NaN);
            r = interp1(ti, r, t, 'linear', NaN);
        end
        Pa(:, :, i) = a;
        Pr(:, :, i) = r;
    end
    F = formationCore(t, Pa, Pr, r_N);
end

function F = formationCore(t, Pa, Pr, r_N)
    % Pa, Pr: T x 3 x N gerçek / referans konumlar (dünya ekseni, başlangıç ofsetleri dahil)
    N = size(Pa, 3);
    pairs = nchoosek(1:N, 2);
    sq_d = 0; sq_f = 0; cnt = 0;
    pair_Ed = nan(size(pairs, 1), 1); pair_EF = pair_Ed;
    pair_Ed_mean = pair_Ed; pair_EF_xy = pair_Ed; pair_EF_z = pair_Ed;
    for k = 1:size(pairs, 1)
        da = Pa(:, :, pairs(k, 1)) - Pa(:, :, pairs(k, 2));
        dr = Pr(:, :, pairs(k, 1)) - Pr(:, :, pairs(k, 2));
        nb = sqrt(sum(dr.^2, 2)) <= r_N;                       % zamana bağlı komşuluk
        e_d = sqrt(sum(da.^2, 2)) - sqrt(sum(dr.^2, 2));       % bağıl mesafe hatası
        ev  = da - dr;                                         % formasyon hatası vektörü e_ij = e_i - e_j
        e_f = sqrt(sum(ev.^2, 2));                             % formasyon (bağıl konum) hatası
        sq_d = sq_d + nb .* e_d.^2;
        sq_f = sq_f + nb .* e_f.^2;
        cnt  = cnt + nb;
        if any(nb)                                             % çift bazında RMS (komşu oldukları süre)
            pair_Ed(k) = sqrt(mean(e_d(nb).^2));
            pair_EF(k) = sqrt(mean(e_f(nb).^2));
            pair_Ed_mean(k) = mean(e_d(nb));                   % işaret: + uzak, - yakın
            pair_EF_xy(k) = sqrt(mean(sum(ev(nb, 1:2).^2, 2))); % yatay bileşen RMS
            pair_EF_z(k)  = sqrt(mean(ev(nb, 3).^2));           % dikey bileşen RMS
        end
    end
    ok = cnt > 0;
    F.t  = t(:);
    F.Ed = nan(size(cnt)); F.Ed(ok) = sqrt(sq_d(ok) ./ cnt(ok));
    F.EF = nan(size(cnt)); F.EF(ok) = sqrt(sq_f(ok) ./ cnt(ok));
    F.rms_Ed = sqrt(mean(F.Ed(ok).^2)); F.max_Ed = max(F.Ed(ok));
    F.rms_EF = sqrt(mean(F.EF(ok).^2)); F.max_EF = max(F.EF(ok));
    F.pairs = pairs; F.pair_Ed = pair_Ed; F.pair_EF = pair_EF;
    F.pair_Ed_mean = pair_Ed_mean; F.pair_EF_xy = pair_EF_xy; F.pair_EF_z = pair_EF_z;
end

function zones = interactionZones(D, thr, t_max)
    % Downwash etkileşim bölgeleri: her an |f_dist| > thr olan İHA sayısından (bkz. interactionZonesCore)
    valid = find([D.ok]);
    t = D(valid(1)).t;
    n_dist = zeros(numel(t), 1);
    for i = valid
        Ti = D(i).T;
        F = sqrt(Ti{:, "dist_f.x"}.^2 + Ti{:, "dist_f.y"}.^2 + Ti{:, "dist_f.z"}.^2);
        if numel(F) ~= numel(t)
            F = interp1(D(i).t, F, t, 'previous', 0);
        end
        n_dist = n_dist + (F > thr);
    end
    zones = interactionZonesCore(t, n_dist, t_max);
end

function zones = interactionZonesCore(t, n_dist, t_max)
    % yeşil: ilk etkilenme -> en az iki İHA'nın aynı anda etkilendiği son an; sarı: oradan t_max'a
    % (0.1 s'ye yuvarlanır). Hiç etkileşim yoksa boş, iki İHA hiç aynı anda etkilenmiyorsa tek bölge.
    k_on  = find(n_dist >= 1, 1, 'first');
    k_two = find(n_dist >= 2, 1, 'last');
    if isempty(k_on)
        zones = zeros(0, 2);
        return;
    end
    t_on = floor(t(k_on) * 10) / 10;
    if isempty(k_two) || t(k_two) <= t(k_on)
        zones = [t_on t_max];
    else
        t_mid = round(t(k_two) * 10) / 10;
        zones = [t_on t_mid; t_mid t_max];
    end
end

function [tblS2, tblS3] = parameterTables(config_dir, launch_file)
    % Table S2 (simülasyon parametreleri) ve Table S3 (kazançlar), makaledeki sembollerle
    q1 = fileread(fullfile(config_dir, 'quadcopter_params_1.yaml'));   % Q1
    q2 = fileread(fullfile(config_dir, 'quadcopter_params_2.yaml'));   % Q2-Q5
    ln = fileread(launch_file);
    dyn1 = yamlSection(q1, 'state_derivative_solver_node');
    dyn2 = yamlSection(q2, 'state_derivative_solver_node');
    m  = [yamlNumber(dyn1, 'mass'), yamlNumber(dyn2, 'mass')];
    g  = yamlArray(dyn1, 'g');
    I  = yamlArray(dyn1, 'I');
    A  = yamlArray(dyn1, 'A_trans');
    C1 = yamlArray(dyn1, 'Cd_trans');
    C2 = yamlArray(dyn2, 'Cd_trans');
    if isequal(C1, C2)
        cd_txt = sprintf('diag(%s)', numList(C1([1 5 9])));
    else
        cd_txt = sprintf('diag(%s) (Q1); diag(%s) (Q2-Q5)', numList(C1([1 5 9])), numList(C2([1 5 9])));
    end
    if isequal(I, yamlArray(dyn2, 'I')), I_txt = sprintf('diag(%s)', numList(I([1 5 9])));
    else, I_txt = sprintf('diag(%s) (Q1); diag(%s) (Q2-Q5)', numList(I([1 5 9])), numList(yamlArray(dyn2, 'I')));
    end
    if m(1) == m(2), m_txt = numList(m(1)); else, m_txt = sprintf('%g - %g', min(m), max(m)); end
    rows = {
        'm',        m_txt,                                          'kg'
        'g',        sprintf('[%s]^T', numList(g)),                  'm/s^2'
        'rho',      numList(yamlNumber(dyn1, 'rho')),               'kg/m^3'
        'C_d',      cd_txt,                                         '-'
        'A',        sprintf('diag(%s)', numList(A([1 5 9]))),       'm^2'
        'C_coup',   numList(launchParam(ln, 'c_coup')),             'm^2'
        'kappa_lat', numList(launchParam(ln, 'r_lat')),             '-'
        'z_0',      numList(launchParam(ln, 'z0')),                 'm'
        'sigma',    numList(launchParam(ln, 'sigma')),              'm'
        'lambda',   numList(launchParam(ln, 'turb_intensity')),     '-'
        'I',        I_txt,                                          'kg m^2'
        'T_m',      numList(yamlNumber(dyn1, 'tau_thrust')),        's'
        'T_tau',    numList(yamlNumber(dyn1, 'tau_torque')),        's'
        'beta',     numList(launchArg(ln, 'psi_filter_beta')),      '-'
        };
    tblS2 = cell2table(rows, 'VariableNames', {'Symbol', 'Value', 'Unit'});

    % Kazançlar (Q1 ve Q2-Q5 aynı olmalı; farklıysa ikisi de yazılır)
    sec = {'mid_level_controller', 'low_level_controller', 'se3_mid_level_controller', 'se3_low_level_controller'};
    names = {'PID position (x, y, z)', 'PID attitude (phi, theta, psi)', ...
             'SE(3) position k_x, k_v (x, y, z)', 'SE(3) attitude k_R, k_Omega (phi, theta, psi)'};
    keys = {{'thrust_x', 'thrust_y', 'thrust_z'}, {'torque1', 'torque2', 'torque3'}, ...
            {'thrust_x', 'thrust_y', 'thrust_z'}, {'torque1', 'torque2', 'torque3'}};
    rows = cell(0, 2);
    for s = 1:numel(sec)
        g1 = gainText(yamlSection(q1, sec{s}), keys{s});
        g2 = gainText(yamlSection(q2, sec{s}), keys{s});
        if ~strcmp(g1, g2), g1 = sprintf('%s (Q1); %s (Q2-Q5)', g1, g2); end
        rows(end+1, :) = {names{s}, g1}; %#ok<AGROW>
    end
    mid1 = yamlSection(q1, 'mid_level_controller');
    lv = [yamlNumber(mid1, 'lv_x', 2.0), yamlNumber(mid1, 'lv_y', 2.0), yamlNumber(mid1, 'lv_z', 2.5)];
    rows(end+1, :) = {'NDOB observer gain L = diag(l_x, l_y, l_z) (1/s)', sprintf('diag(%s)', numList(lv))};
    rows(end+1, :) = {'Update rates trajectory / position / attitude / dynamics (Hz)', ...
                      sprintf('%g / %g / %g / %g', 20, 1 / yamlNumber(mid1, 'dt'), ...
                              1 / yamlNumber(yamlSection(q1, 'low_level_controller'), 'dt'), ...
                              1 / yamlNumber(dyn1, 'dt'))};
    tblS3 = cell2table(rows, 'VariableNames', {'Gain', 'Value_kp_ki_kd'});
end

function txt = gainText(sec_txt, axis_keys)
    % "kp/ki/kd" her eksen için, eksenler "; " ile
    parts = cell(1, numel(axis_keys));
    for a = 1:numel(axis_keys)
        parts{a} = sprintf('%g/%g/%g', yamlNumber(sec_txt, ['kp_' axis_keys{a}]), ...
                           yamlNumber(sec_txt, ['ki_' axis_keys{a}]), yamlNumber(sec_txt, ['kd_' axis_keys{a}]));
    end
    txt = strjoin(parts, '; ');
end

function sec_txt = yamlSection(txt, name)
    % Sütun 0'daki "name:" satırından bir sonraki üst seviye anahtara kadar olan blok
    lines_ = regexp(txt, '\r?\n', 'split');
    k0 = find(strcmp(strtrim(lines_), [name ':']) & startsWith(lines_, name), 1);
    if isempty(k0), error('yaml bölümü bulunamadı: %s', name); end
    k1 = numel(lines_);
    for k = k0 + 1:numel(lines_)
        L = lines_{k};
        if ~isempty(L) && ~isspace(L(1)) && L(1) ~= '#'
            k1 = k - 1;
            break;
        end
    end
    sec_txt = strjoin(lines_(k0 + 1:k1), newline);
end

function v = yamlNumber(txt, key, default)
    tok = regexp(txt, ['(?m)^\s*' key ':\s*([-+0-9.eE]+)'], 'tokens', 'once');
    if isempty(tok)
        if nargin > 2, v = default; return; end
        error('yaml anahtarı bulunamadı: %s', key);
    end
    v = str2double(tok{1});
end

function v = launchParam(txt, name)
    tok = regexp(txt, ['<param name="' name '" value="([^"]+)"'], 'tokens', 'once');
    if isempty(tok), error('launch parametresi bulunamadı: %s', name); end
    v = str2double(tok{1});
end

function v = launchArg(txt, name)
    tok = regexp(txt, ['<arg name="' name '" default="([^"]+)"'], 'tokens', 'once');
    if isempty(tok), error('launch argümanı bulunamadı: %s', name); end
    v = str2double(tok{1});
end

function s = numList(v)
    s = strjoin(arrayfun(@(x) sprintf('%g', x), v, 'UniformOutput', false), ', ');
end

function lim = padLimits(vals, frac)
    vals = vals(isfinite(vals));
    if isempty(vals), lim = [-1 1]; return; end
    lo = min(vals); hi = max(vals);
    span = hi - lo;
    if span < 1e-9, span = max(abs(hi), 1) * 0.2; end
    lim = [lo - frac * span, hi + frac * span];
end

function addZones(ax, zones, cols)
    yl = ylim(ax);
    p = gobjects(size(zones, 1), 1);
    for z = 1:size(zones, 1)
        p(z) = patch(ax, [zones(z, 1) zones(z, 2) zones(z, 2) zones(z, 1)], [yl(1) yl(1) yl(2) yl(2)], ...
                     cols(z, :), 'EdgeColor', 'none', 'HandleVisibility', 'off');
    end
    uistack(p, 'bottom');
    ylim(ax, yl);
end

function styleAxes(ax, S)
    grid(ax, 'off');
    xlim(ax, [0 S.t_max]);
    set(ax, 'FontName', S.font_name, 'FontSize', S.font_size, 'FontWeight', 'bold', ...
        'LineWidth', S.line_w, 'Box', 'on', 'Layer', 'top');
end

function addTopLegend(handles, position, S)
    lgd = legend(handles, 'Orientation', 'horizontal', 'FontSize', S.font_size, 'FontWeight', 'bold', ...
                 'Box', 'off', 'FontName', S.font_name);
    set(lgd, 'Position', position);
end

function saveFigure(fig, out_folder, out_name, output_pdf)
    % Figürü SVG (vektör) olarak kaydeder; exportgraphics SVG yazamadığı için print kullanılır.
    [~, base] = fileparts(out_name);
    svg_file = fullfile(out_folder, [base '.svg']);
    set(fig, 'PaperPositionMode', 'auto');                % ekrandaki boyut korunsun
    try
        print(fig, svg_file, '-dsvg', '-vector');         % R2022b ve sonrası
    catch
        print(fig, svg_file, '-dsvg', '-painters');       % eski sürümler
    end
    exportgraphics(fig, output_pdf, 'Append', true);
    close(fig);
end
