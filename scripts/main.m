%% MAKALE GRAFİKLERİ - Disturbance-Aware Hybrid Trajectory Generation and Control for Multirotor Swarms
% Sıralı simülasyonun (launch/sequential_sim.launch) loglarından makaledeki ve ek materyaldeki
% grafikleri ve tabloları üretir:
%
%   Fig. 1  Pozisyon (x, y, z) + z zoom                    -> log_dir
%   Fig. 2  Anlık konum hatası: önerilen vs SE(3)          -> log_dir + geo_log_dir
%   Fig. 3  Hız                                            -> log_dir
%   Fig. 4  İvme                                           -> log_dir
%   Fig. 5  Euler açıları                                  -> log_dir
%   Fig. 6  Bozucu kuvvetler                               -> log_dir
%   Fig. 7  Thrust                                         -> log_dir
%   NDOB    NDOB bozucu kestirimi (controller:=ndob koşusu) -> log_dir (ndob_f sütunları varsa)
%   Fig. 8  Ablasyon: feedforward aktif vs pasif           -> log_dir + ablation_log_dir
%   Fig. S5 Drone 1 hibrit yörünge ve waypointler          -> log_dir + config/*.yaml
%   Fig. S6 Drone 2 yönelim hatası, beta karşılaştırması   -> log_dir + beta_log_dir
%   Table 1 RMSE/ISE: önerilen vs SE(3)                    -> log_dir + geo_log_dir
%   Table 2 RMSE/ISE: feedforward aktif vs pasif           -> log_dir + ablation_log_dir
%   Fig. F1 Komşu İHA'lar arası bağıl mesafe hatası ve formasyon hatası -> log_dir (+ ablation_log_dir, geo_log_dir)
%   Table 3 İHA çiftleri matrisi: üst üçgen formasyon hatası, alt üçgen bağıl mesafe hatası (RMS)
%   (Fig. S2 için: fig_S2_downwash_field.m)
%
% Gerekli simülasyon koşularının hepsi tek komutla çalıştırılabilir (her biri ayrı log klasörüne yazar):
%   rosrun multi_quadcopter_control run_paper_sims.sh
% veya tek tek:
%   1) Önerilen kontrolcü (controller:=pid_ff, varsayılan; beta = 0.05) -> log/
%      roslaunch multi_quadcopter_control sequential_sim.launch rviz:=false real_time_factor:=0 shutdown_on_finish:=true
%   2) Ablasyon (controller:=pid, feedforward yok) -> log_no_feedforward/
%      roslaunch multi_quadcopter_control sequential_sim.launch rviz:=false real_time_factor:=0 shutdown_on_finish:=true controller:=pid log_dir:=$(rospack find multi_quadcopter_control)/log_no_feedforward
%   3) Yaw filtresi beta = 0.1 -> log_beta_0.1/
%      roslaunch multi_quadcopter_control sequential_sim.launch rviz:=false real_time_factor:=0 shutdown_on_finish:=true psi_filter_beta:=0.1 log_dir:=$(rospack find multi_quadcopter_control)/log_beta_0.1
%   4) Geometrik SE(3) kontrolcü -> log_se3/
%      roslaunch multi_quadcopter_control sequential_sim.launch rviz:=false real_time_factor:=0 shutdown_on_finish:=true controller:=se3_ff log_dir:=$(rospack find multi_quadcopter_control)/log_se3
% Eksik klasörlere bağlı grafikler uyarı verilerek atlanır.

clear; clc; close all;
warning('off', 'MATLAB:print:ContentTypeImageSuggested');   % vektör EPS isteniyor

%% === AYARLAR ===
num_drones = 5;

script_dir = fileparts(mfilename('fullpath'));
if isempty(script_dir), script_dir = pwd; end
pkg_dir    = fullfile(script_dir, '..');

log_dir          = fullfile(pkg_dir, 'log');                  % önerilen kontrolcü (ana koşu)
ablation_log_dir = fullfile(pkg_dir, 'log_no_feedforward');   % Fig. 8 / Table 2
beta_log_dir     = fullfile(pkg_dir, 'log_beta_0.1');         % Fig. S6
geo_log_dir      = fullfile(pkg_dir, 'log_se3');              % Fig. 2 / Table 1: SE(3) kontrolcü (controller:=se3_ff)
config_dir       = fullfile(pkg_dir, 'config');               % Fig. S5 waypointleri

beta_main   = 0.05;   % log_dir koşusundaki psi_filter_beta
beta_alt    = 0.1;    % beta_log_dir koşusundaki psi_filter_beta
s6_drone    = 2;      % Fig. S6 drone'u
fig8_drones = [2 3];  % Fig. 8 drone'ları
s5_drone    = 1;      % Fig. S5 drone'u

% Formasyon ölçütleri (Fig. F1, Table 3)
% Komşuluk zamana bağlıdır: referans (istenen) mesafesi neighbor_radius'tan küçük olan çiftler
% o anda komşudur (formasyon uçuş boyunca değiştiği için sabit bir graf kullanılmaz).
neighbor_radius = 3.0;   % m

output_pdf = fullfile(script_dir, 'all_drones_plots_fixed.pdf');
eps_folder = fullfile(script_dir, 'EPS_Results');

% Zaman ekseni (makaledeki gibi 0-60 s)
t_axis_max = 60;

% Yörünge geçiş anları (s). Sıralı simülasyonda deterministiktir:
% cubic spline -> min snap (8.05 s) -> min snap (21.05 s) -> min jerk (32.55 s), bitiş 54.55 s.
t_switch = [8.05 21.05 32.55];

% Downwash etkileşim bölgeleri (s): yeşil = Q4/Q5 -> Q2/Q3, sarı = Q4 ve Q5 -> Q2
t_interact_green  = [23.5 33.5];
t_interact_yellow = 33.5;           % -> t_axis_max

% Fig. 1 z zoom pencereleri (s) ve z aralığı (m)
zoom_windows = [23 25; 34 36];
zoom_colors  = {'r', 'b'};
zoom_zlim    = [1.9 3.0];

% Çubuk grafikleri (Fig. 2, 8, S6): her sparse_factor örnekten biri (500 Hz log -> 0.6 s)
sparse_factor = 300;

% Çizgi grafikleri her plot_step örnekten birini çizer (500 Hz log -> 100 Hz). Vektör EPS dosyalarını
% küçültür ve hızlandırır; metrikler (RMSE, ISE) ve çubuk grafikleri her zaman tam veriyi kullanır.
plot_step = 5;

% Grafik stili
font_name = 'Times New Roman';
font_size = 20;
line_w    = 2;
colors    = lines(num_drones);
dark_gray = [0.2 0.2 0.2];

c_phase  = [1 0.88 0.88; 0.85 1 1; 0.85 1 0.85; 1 1 0.9];   % pembe, mavi, yeşil, sarı
c_green  = [0.85 1 0.85];
c_yellow = [1 1 0.9];

%% === HAZIRLIK ===
if isfile(output_pdf), delete(output_pdf); end
if ~exist(eps_folder, 'dir'), mkdir(eps_folder); end

phase_zones    = [[0 t_switch]' [t_switch t_axis_max]'];
interact_zones = [t_interact_green; t_interact_yellow t_axis_max];
interact_cols  = [c_green; c_yellow];
axis_names     = ["x", "y", "z"];

S = struct('font_name', font_name, 'font_size', font_size, 'line_w', line_w, 't_max', t_axis_max);

%% === VERİ OKUMA (her dosya bir kez) ===
D = readLogs(log_dir, num_drones, plot_step);
valid = find([D.ok]);
if isempty(valid)
    error('Hiç log dosyası okunamadı. log_dir = %s', log_dir);
end
fprintf('Ana koşu: %d drone logu okundu (%s)\n', numel(valid), log_dir);

D_abl  = readLogs(ablation_log_dir, num_drones, plot_step);
D_beta = readLogs(beta_log_dir, num_drones, plot_step);
D_geo  = readLogs(geo_log_dir, num_drones, plot_step);

%% === Fig. 1 - POZİSYON + Z ZOOM ===
fig = figure;
lgd_handles = gobjects(0);
for a = 1:3
    ax = subplot(3, 1, a); hold(ax, 'on');
    if a == 1
        lgd_handles(end+1) = plot(ax, NaN, NaN, ':k', 'LineWidth', line_w, 'DisplayName', 'Desired'); %#ok<SAGROW>
    end
    for i = valid
        h = plot(ax, D(i).tp, D(i).Tp{:, "a_pos." + axis_names(a)}, '-', 'Color', colors(i,:), ...
                 'LineWidth', line_w, 'DisplayName', sprintf('Q %d', i));
        if a == 1, lgd_handles(end+1) = h; else, h.HandleVisibility = 'off'; end %#ok<SAGROW>
        plot(ax, D(i).tp, D(i).Tp{:, "r_pos." + axis_names(a)}, ':k', 'LineWidth', line_w, ...
             'HandleVisibility', 'off');
    end
    yl = ylim(ax);
    if a < 3
        ylim(ax, [yl(1) - 1, yl(2) + 1]);
    else
        ylim(ax, [yl(1) - 0.4, yl(2) + 0.2]);   % altta zoom inset'leri için boşluk
        xlabel(ax, 't (s)', 'FontWeight', 'bold', 'FontSize', font_size, 'FontName', font_name);
    end
    addZones(ax, phase_zones, c_phase);
    styleAxes(ax, S);
    ylabel(ax, sprintf('%s (m)', axis_names(a)), 'FontWeight', 'bold', 'FontSize', font_size, ...
           'FontName', font_name);
end
hZ = ax;
for w = 1:size(zoom_windows, 1)
    for tw = zoom_windows(w, :)
        line(hZ, [tw tw], zoom_zlim, 'Color', zoom_colors{w}, 'LineStyle', '--', ...
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
        plot(ax_zoom, D(i).tp, D(i).Tp{:, "r_pos.z"}, ':k', 'LineWidth', line_w);
    end
    xlim(ax_zoom, zoom_windows(w, :));
    ylim(ax_zoom, zoom_zlim);
    set(ax_zoom, 'FontSize', 9, 'FontName', font_name, 'XTick', [], 'YTick', [], 'LineWidth', 1.5);
end
addTopLegend(lgd_handles, [0.1 0.94 0.8 0.05], S);
saveFigure(fig, eps_folder, 'Fig1.eps', output_pdf);

%% === Fig. 2 - ANLIK KONUM HATASI: ÖNERİLEN vs SE(3) ===
if any([D_geo.ok])
    fig = figure('Position', [100, 100, 1200, 800]);
    tl = tiledlayout(fig, 3, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    for i = 1:num_drones
        ax = nexttile(tl); hold(ax, 'on'); box(ax, 'on');
        [hs, names] = errorBars(ax, {D(i), D_geo(i)}, {colors(i,:), dark_gray}, ...
                                {'Proposed Controller', 'SE(3) Controller'}, sparse_factor);
        finishBarAxes(ax, phase_zones, c_phase, S, 16);
        barLegend(ax, hs, names, sprintf('Q %d', i), font_name);
    end
    xlabel(tl, 't (s)', 'FontWeight', 'bold', 'FontSize', font_size, 'FontName', font_name);
    ylabel(tl, '||e_p|| (m)', 'FontWeight', 'bold', 'FontSize', font_size, 'FontName', font_name);
    saveFigure(fig, eps_folder, 'Fig2.eps', output_pdf);
else
    warning('Fig. 2 / Table 1 atlandı: SE(3) logları bulunamadı (geo_log_dir = ''%s'').', geo_log_dir);
end

%% === Fig. 3 - HIZ ===
plotStateFigure(D, valid, "a_vel.", "r_vel.", ["v_x (m/s)", "v_y (m/s)", "v_z (m/s)"], ...
                phase_zones, c_phase, colors, S, eps_folder, 'Fig3.eps', output_pdf);

%% === Fig. 4 - İVME ===
% Not: a_z gerçekten "a_acc.z" sütunundan okunur (eski script bu panele z konumunu çiziyordu).
plotStateFigure(D, valid, "a_acc.", "r_acc.", ["a_x (m/s^2)", "a_y (m/s^2)", "a_z (m/s^2)"], ...
                phase_zones, c_phase, colors, S, eps_folder, 'Fig4.eps', output_pdf);

%% === Fig. 5 - EULER AÇILARI ===
plotStateFigure(D, valid, "a_eul_ang.", "r_eul_ang.", ["\phi (rad)", "\theta (rad)", "\psi (rad)"], ...
                phase_zones, c_phase, colors, S, eps_folder, 'Fig5.eps', output_pdf);

%% === Fig. 6 - BOZUCU KUVVETLER ===
fig = figure;
lgd_handles = gobjects(0);
for a = 1:3
    ax = subplot(3, 1, a); hold(ax, 'on');
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
    if a == 3
        xlabel(ax, 't (s)', 'FontWeight', 'bold', 'FontSize', font_size, 'FontName', font_name);
    end
end
addTopLegend(lgd_handles, [0.1 0.94 0.8 0.05], S);
saveFigure(fig, eps_folder, 'Fig6.eps', output_pdf);

%% === NDOB KESTİRİMİ (yalnızca controller:=ndob koşusunda, ndob_f sütunları varsa) ===
if ismember("ndob_f.x", D(valid(1)).T.Properties.VariableNames)
    fig = figure;
    lgd_handles = gobjects(0);
    for a = 1:3
        ax = subplot(3, 1, a); hold(ax, 'on');
        all_vals = [];
        for i = valid
            v = D(i).Tp{:, "ndob_f." + axis_names(a)};
            all_vals = [all_vals; v]; %#ok<AGROW>
            h = plot(ax, D(i).tp, v, 'LineWidth', line_w, 'Color', colors(i,:), 'DisplayName', sprintf('Q %d', i));
            if a == 1, lgd_handles(end+1) = h; else, h.HandleVisibility = 'off'; end %#ok<SAGROW>
        end
        ylim(ax, padLimits(all_vals, 0.1));
        addZones(ax, interact_zones, interact_cols);
        styleAxes(ax, S);
        ylabel(ax, sprintf('\\Delta f_{%s} (N)', axis_names(a)), 'FontWeight', 'bold', ...
               'FontSize', font_size, 'FontName', font_name);
        if a == 3
            xlabel(ax, 't (s)', 'FontWeight', 'bold', 'FontSize', font_size, 'FontName', font_name);
        end
    end
    addTopLegend(lgd_handles, [0.1 0.94 0.8 0.05], S);
    saveFigure(fig, eps_folder, 'Fig_NDOB_estimate.eps', output_pdf);
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
saveFigure(fig, eps_folder, 'Fig7.eps', output_pdf);

%% === Fig. 8 - ABLASYON: FEEDFORWARD AKTİF vs PASİF ===
if any([D_abl.ok])
    fig = figure('Position', [100, 100, 1200, 500]);
    tl = tiledlayout(fig, 1, numel(fig8_drones), 'TileSpacing', 'compact', 'Padding', 'compact');
    for i = fig8_drones
        ax = nexttile(tl); hold(ax, 'on'); box(ax, 'on');
        [hs, names] = errorBars(ax, {D_abl(i), D(i)}, {colors(i,:), dark_gray}, ...
                                {'Feedforward Inactive', 'Feedforward Active'}, sparse_factor);
        finishBarAxes(ax, phase_zones, c_phase, S, font_size);
        barLegend(ax, hs, names, sprintf('Q %d', i), font_name);
    end
    xlabel(tl, 't (s)', 'FontWeight', 'bold', 'FontSize', 24, 'FontName', font_name);
    ylabel(tl, '||e_p|| (m)', 'FontWeight', 'bold', 'FontSize', 24, 'FontName', font_name);
    saveFigure(fig, eps_folder, 'Fig8.eps', output_pdf);
else
    warning('Fig. 8 / Table 2 atlandı: ablasyon logları bulunamadı (%s).', ablation_log_dir);
end

%% === Fig. S5 - DRONE 1 HİBRİT YÖRÜNGE VE WAYPOINTLER ===
wp = [];
try
    wp = loadWaypointSets(config_dir, s5_drone);
catch err
    warning('Fig. S5 atlandı: waypointler okunamadı (%s).', err.message);
end
if ~isempty(wp) && D(s5_drone).ok
    plotFigS5(D(s5_drone), wp, colors(s5_drone,:), font_name, eps_folder, output_pdf);
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
    saveFigure(fig, eps_folder, 'FigS6.eps', output_pdf);
else
    warning('Fig. S6 atlandı: beta logları bulunamadı (%s).', beta_log_dir);
end

%% === FORMASYON ÖLÇÜTLERİ (Fig. F1, Table 3) ===
% Komşu çift (i,j) için (komşu: istenen mesafe <= neighbor_radius):
%   bağıl mesafe hatası  e_d,ij = ||p_i - p_j|| - ||p_i^ref - p_j^ref||
%   formasyon hatası     e_ij   = (p_i - p_j) - (p_i^ref - p_j^ref)
% Her an komşu çiftler üzerinden RMS alınır: E_d(t), E_F(t).
runs_F   = {D, D_abl, D_geo};
labels_F = {'Proposed (PID + FF)', 'Without feedforward', 'SE(3) Controller'};
cols_F   = {[0 0.4470 0.7410], [0.8500 0.3250 0.0980], dark_gray};
F = cell(1, 3);
for r = 1:3
    if all([runs_F{r}.ok])
        F{r} = formationMetrics(runs_F{r}, neighbor_radius);
    end
end
avail_F = find(~cellfun(@isempty, F));

fig = figure;
fields_F  = {'Ed', 'EF'};
ylabels_F = {'E_d (m)', 'E_F (m)'};
hs = gobjects(0);
for a = 1:2
    ax = subplot(2, 1, a); hold(ax, 'on');
    for r = avail_F
        idx = 1:plot_step:numel(F{r}.t);
        h = plot(ax, F{r}.t(idx), F{r}.(fields_F{a})(idx), 'LineWidth', line_w, 'Color', cols_F{r}, ...
                 'DisplayName', labels_F{r});
        if a == 1, hs(end+1) = h; end %#ok<SAGROW>
    end
    yl = ylim(ax); ylim(ax, [0 yl(2) * 1.05]);
    addZones(ax, phase_zones, c_phase);
    styleAxes(ax, S);
    ylabel(ax, ylabels_F{a}, 'FontWeight', 'bold', 'FontSize', font_size, 'FontName', font_name);
    if a == 2
        xlabel(ax, 't (s)', 'FontWeight', 'bold', 'FontSize', font_size, 'FontName', font_name);
    end
end
addTopLegend(hs, [0.1 0.94 0.8 0.05], S);
saveFigure(fig, eps_folder, 'FigF1_formation_error.eps', output_pdf);

% Table 3: İHA çiftleri matrisi (her kontrolcü için). Satır/sütun Q1..Q5;
%   köşegenin üstü: formasyon hatası RMS ||e_ij|| (m), köşegenin altı: bağıl mesafe hatası RMS e_d,ij (m).
% Değerler çiftin komşu olduğu süre boyunca hesaplanır; hiç komşu olmayan çiftler '-' ile gösterilir.
names_Q = "Q" + string(1:num_drones);
file_tag = {'proposed', 'without_ff', 'se3'};
for r = avail_F
    M = repmat("-", num_drones, num_drones);
    for k = 1:size(F{r}.pairs, 1)
        i1 = F{r}.pairs(k, 1); j1 = F{r}.pairs(k, 2);
        if ~isnan(F{r}.pair_EF(k))
            M(i1, j1) = sprintf('%.3f', F{r}.pair_EF(k));   % üst üçgen: formasyon hatası
            M(j1, i1) = sprintf('%.3f', F{r}.pair_Ed(k));   % alt üçgen: bağıl mesafe hatası
        end
    end
    tbl3 = array2table(M, 'RowNames', cellstr(names_Q), 'VariableNames', cellstr(names_Q));
    fprintf('\nTable 3 (%s): üst üçgen = formasyon hatası, alt üçgen = bağıl mesafe hatası (RMS, m)\n', labels_F{r});
    disp(tbl3);
    writetable(tbl3, fullfile(eps_folder, sprintf('Table3_formation_%s.csv', file_tag{r})), 'WriteRowNames', true);
end

%% === Table 1 / Table 2 - PERFORMANS METRİKLERİ ===
M = metrics(D);
fprintf('\n=====================================================================\n');
fprintf('   ÖNERİLEN KONTROLCÜ (RMSE: ortalama, ISE: toplam, MAX: en büyük)\n');
fprintf('=====================================================================\n');
for i = 1:num_drones
    if D(i).ok
        fprintf('Q %d -> RMSE: %.5f m | ISE: %.5f m^2s | MAX: %.5f m\n', i, M.rmse(i), M.ise(i), M.max(i));
    else
        fprintf('Q %d: Dosya bulunamadı.\n', i);
    end
end

if any([D_geo.ok])
    tbl1 = comparisonTable(M, metrics(D_geo), 'HierarchicalPID', 'Geometric');
    fprintf('\nTable 1: Hiyerarşik PID vs Geometrik (SE(3)) kontrolcü\n');
    disp(tbl1);
    writetable(tbl1, fullfile(eps_folder, 'Table1.csv'));
end
if any([D_abl.ok])
    tbl2 = comparisonTable(M, metrics(D_abl), 'Proposed', 'NoFeedforward');
    fprintf('\nTable 2: Ablasyon - feedforward aktif vs pasif (f_dist = 0)\n');
    disp(tbl2);
    writetable(tbl2, fullfile(eps_folder, 'Table2.csv'));
end
fprintf('=====================================================================\n');
fprintf('Grafikler "%s" klasörüne ve "%s" dosyasına kaydedildi.\n', eps_folder, output_pdf);


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

function e = positionError(T)
    e = sqrt((T{:, "a_pos.x"} - T{:, "r_pos.x"}).^2 + ...
             (T{:, "a_pos.y"} - T{:, "r_pos.y"}).^2 + ...
             (T{:, "a_pos.z"} - T{:, "r_pos.z"}).^2);
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

function tbl = comparisonTable(A, B, nameA, nameB)
    % Makaledeki gibi: iyileşme (%) = (B - A) / B * 100
    n = numel(A.rmse);
    Quadcopter = "Q " + string((1:n)');
    tbl = table(Quadcopter, A.rmse, A.ise, B.rmse, B.ise, ...
                (B.rmse - A.rmse) ./ B.rmse * 100, (B.ise - A.ise) ./ B.ise * 100, ...
                'VariableNames', {'Quadcopter', [nameA '_RMSE_m'], [nameA '_ISE'], ...
                                  [nameB '_RMSE_m'], [nameB '_ISE'], 'RMSE_Improvement_pct', 'ISE_Improvement_pct'});
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
    widths = [1.0 0.5];
    alphas = [0.6 0.9];
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
                         eps_folder, eps_name, output_pdf)
    % 3 eksenli gerçek (renkli) + istenen (siyah noktalı) zaman grafiği
    fig = figure;
    axis_names = ["x", "y", "z"];
    lgd_handles = gobjects(0);
    for a = 1:3
        ax = subplot(3, 1, a); hold(ax, 'on');
        if a == 1
            lgd_handles(end+1) = plot(ax, NaN, NaN, ':k', 'LineWidth', S.line_w, 'DisplayName', 'Desired'); %#ok<AGROW>
        end
        all_vals = [];
        for i = valid
            v = D(i).Tp{:, act_prefix + axis_names(a)};
            r = D(i).Tp{:, ref_prefix + axis_names(a)};
            if act_prefix == "a_eul_ang." && a == 3
                % yaw: gerçek açı +/-pi'ye sarılı, referans sürekli -> ikisini de sürekli çiz
                v = unwrap(v); r = unwrap(r);
                k0 = round((r(1) - v(1)) / (2 * pi)); v = v + 2 * pi * k0;
            end
            all_vals = [all_vals; v; r]; %#ok<AGROW>
            h = plot(ax, D(i).tp, v, '-', 'Color', colors(i,:), 'LineWidth', S.line_w, ...
                     'DisplayName', sprintf('Q %d', i));
            if a == 1, lgd_handles(end+1) = h; else, h.HandleVisibility = 'off'; end %#ok<AGROW>
            plot(ax, D(i).tp, r, ':k', 'LineWidth', S.line_w, 'HandleVisibility', 'off');
        end
        ylim(ax, padLimits(all_vals, 0.1));
        addZones(ax, zones, zone_cols);
        styleAxes(ax, S);
        ylabel(ax, ylabels(a), 'FontWeight', 'bold', 'FontSize', S.font_size, 'FontName', S.font_name);
        if a == 3
            xlabel(ax, 't (s)', 'FontWeight', 'bold', 'FontSize', S.font_size, 'FontName', S.font_name);
        end
    end
    addTopLegend(lgd_handles, [0.1 0.94 0.8 0.05], S);
    saveFigure(fig, eps_folder, eps_name, output_pdf);
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

function plotFigS5(Dd, wp, path_color, font_name, eps_folder, output_pdf)
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
            plot3(ax, px, py, pz, '-', 'Color', path_color, 'LineWidth', 2.5);
        else
            plot(ax, px, py, '-', 'Color', path_color, 'LineWidth', 2.5);
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
    saveFigure(fig, eps_folder, 'FigS5.eps', output_pdf);
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
    for k = 1:size(pairs, 1)
        da = Pa(:, :, pairs(k, 1)) - Pa(:, :, pairs(k, 2));
        dr = Pr(:, :, pairs(k, 1)) - Pr(:, :, pairs(k, 2));
        nb = sqrt(sum(dr.^2, 2)) <= r_N;                       % zamana bağlı komşuluk
        e_d = sqrt(sum(da.^2, 2)) - sqrt(sum(dr.^2, 2));       % bağıl mesafe hatası
        e_f = sqrt(sum((da - dr).^2, 2));                      % formasyon (bağıl konum) hatası
        sq_d = sq_d + nb .* e_d.^2;
        sq_f = sq_f + nb .* e_f.^2;
        cnt  = cnt + nb;
        if any(nb)                                             % çift bazında RMS (komşu oldukları süre)
            pair_Ed(k) = sqrt(mean(e_d(nb).^2));
            pair_EF(k) = sqrt(mean(e_f(nb).^2));
        end
    end
    ok = cnt > 0;
    F.t  = t(:);
    F.Ed = nan(size(cnt)); F.Ed(ok) = sqrt(sq_d(ok) ./ cnt(ok));
    F.EF = nan(size(cnt)); F.EF(ok) = sqrt(sq_f(ok) ./ cnt(ok));
    F.rms_Ed = sqrt(mean(F.Ed(ok).^2)); F.max_Ed = max(F.Ed(ok));
    F.rms_EF = sqrt(mean(F.EF(ok).^2)); F.max_EF = max(F.EF(ok));
    F.pairs = pairs; F.pair_Ed = pair_Ed; F.pair_EF = pair_EF;
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

function saveFigure(fig, eps_folder, eps_name, output_pdf)
    exportgraphics(fig, fullfile(eps_folder, eps_name), 'ContentType', 'vector');
    exportgraphics(fig, output_pdf, 'Append', true);
    close(fig);
end
