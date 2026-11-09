%% MULTI-QUADCOPTER SİMÜLASYON SONUÇLARI
% sequential_sim.launch ile üretilen log/droneN_log.txt dosyalarını okur,
% makale grafiklerini EPS (EPS_Results/) ve tek bir PDF olarak kaydeder,
% performans metriklerini (RMSE, ISE, maksimum hata) yazdırır.
%
% Kullanılmayan eski grafik bölümleri scripts/legacy_plots.m dosyasındadır.

clear; clc; close all;

%% === AYARLAR ===
num_drones = 5;

% Log klasörü: bu dosyanın bulunduğu klasöre göre ../log (paket içindeki log klasörü)
script_dir = fileparts(mfilename('fullpath'));
if isempty(script_dir), script_dir = pwd; end
log_dir    = fullfile(script_dir, '..', 'log');

output_pdf = fullfile(script_dir, 'all_drones_plots_fixed.pdf');
eps_folder = fullfile(script_dir, 'EPS_Results');

% Yörünge geçiş anları (s). Sıralı simülasyonda deterministiktir:
% trajectory_random -> _1 (8.05 s) -> _2 (21.05 s) -> _3 (32.55 s), bitiş 54.55 s.
t_switch = [8.05 21.05 32.55];

% Downwash etkileşim bölgeleri (s): yeşil = Q2 ve Q3 etkileşimi, sarı = yalnızca Q2
t_interact_green  = [23.5 33.5];
t_interact_yellow = 33.5;           % -> simülasyon sonuna kadar

% z ekseni zoom pencereleri (s) ve zoom z aralığı (m)
zoom_windows = [23 25; 34 36];
zoom_colors  = {'r', 'b'};
zoom_zlim    = [1.9 3.0];

% Grafik stili
font_name = 'Times New Roman';
font_size = 20;
line_w    = 2;
colors    = lines(num_drones);

c_phase = [1 0.88 0.88; 0.85 1 1; 0.85 1 0.85; 1 1 0.9];   % pembe, mavi, yeşil, sarı
c_green  = [0.85 1 0.85];
c_yellow = [1 1 0.9];

%% === HAZIRLIK ===
if isfile(output_pdf), delete(output_pdf); end
if ~exist(eps_folder, 'dir'), mkdir(eps_folder); end

%% === VERİ OKUMA (her dosya bir kez okunur) ===
D = repmat(struct('ok', false, 't', [], 'T', []), num_drones, 1);
for i = 1:num_drones
    filename = fullfile(log_dir, sprintf('drone%d_log.txt', i));
    if ~isfile(filename)
        warning('Log dosyası bulunamadı: %s', filename);
        continue;
    end
    T = readtable(filename, 'VariableNamingRule', 'preserve');
    D(i).ok = true;
    D(i).T  = T;
    D(i).t  = T{:, "time"} - T{1, "time"};
end
valid = find([D.ok]);
if isempty(valid)
    error('Hiç log dosyası okunamadı. log_dir = %s', log_dir);
end
t_end = max(arrayfun(@(k) D(k).t(end), valid));
fprintf('%d drone logu okundu, simülasyon süresi %.2f s\n', numel(valid), t_end);

% Zaman bölgeleri: [başlangıç bitiş] satırları
phase_zones    = [[0 t_switch]' [t_switch t_end]'];
interact_zones = [t_interact_green; t_interact_yellow t_end];
interact_cols  = [c_green; c_yellow];

%% === 1. TÜM DRONE'LAR - THRUST (Fig14) ===
fig = figure;
ax = axes(fig); hold(ax, 'on');
lgd_handles = gobjects(0);
all_vals = [];
for i = valid
    v = D(i).T{:, "r_thr"};
    all_vals = [all_vals; v]; %#ok<AGROW>
    lgd_handles(end+1) = plot(ax, D(i).t, v, 'LineWidth', line_w, 'Color', colors(i,:), ...
                              'DisplayName', sprintf('Q %d', i)); %#ok<SAGROW>
end
ylim(ax, padLimits(all_vals, 0.1));
addZones(ax, interact_zones, interact_cols);
styleAxes(ax, t_end, font_name, font_size, line_w);
xlabel(ax, 't (s)', 'FontWeight', 'bold', 'FontSize', font_size, 'FontName', font_name);
ylabel(ax, '||f_{thrust}|| (N)', 'FontWeight', 'bold', 'FontSize', font_size, 'FontName', font_name);
addTopLegend(lgd_handles, [0.15 0.94 0.7 0.05], font_name, font_size);
saveFigure(fig, eps_folder, 'Fig14.eps', output_pdf);

%% === 2. TÜM DRONE'LAR - DISTURBANCE FORCES (Fig13) ===
fig = figure;
axis_names = ["x", "y", "z"];
lgd_handles = gobjects(0);
for a = 1:3
    ax = subplot(3, 1, a); hold(ax, 'on');
    col = "dist_f." + axis_names(a);
    all_vals = [];
    for i = valid
        v = D(i).T{:, col};
        all_vals = [all_vals; v]; %#ok<AGROW>
        h = plot(ax, D(i).t, v, 'LineWidth', line_w, 'Color', colors(i,:), ...
                 'DisplayName', sprintf('Q %d', i));
        if a == 1, lgd_handles(end+1) = h; else, h.HandleVisibility = 'off'; end %#ok<SAGROW>
    end
    ylim(ax, padLimits(all_vals, 0.1));
    addZones(ax, interact_zones, interact_cols);
    styleAxes(ax, t_end, font_name, font_size, line_w);
    ylabel(ax, sprintf('f_{dist,%s} (N)', axis_names(a)), 'FontWeight', 'bold', ...
           'FontSize', font_size, 'FontName', font_name);
    if a == 3
        xlabel(ax, 't (s)', 'FontWeight', 'bold', 'FontSize', font_size, 'FontName', font_name);
    end
end
addTopLegend(lgd_handles, [0.1 0.94 0.8 0.05], font_name, font_size);
saveFigure(fig, eps_folder, 'Fig13.eps', output_pdf);

%% === 3. TÜM DRONE'LAR - POZİSYON + Z ZOOM (Fig6a) ===
fig = figure;
lgd_handles = gobjects(0);
for a = 1:3
    ax = subplot(3, 1, a); hold(ax, 'on');
    if a == 1
        lgd_handles(end+1) = plot(ax, NaN, NaN, ':k', 'LineWidth', line_w, 'DisplayName', 'Desired');
    end
    for i = valid
        h = plot(ax, D(i).t, D(i).T{:, "a_pos." + axis_names(a)}, '-', 'Color', colors(i,:), ...
                 'LineWidth', line_w, 'DisplayName', sprintf('Q %d', i));
        if a == 1, lgd_handles(end+1) = h; else, h.HandleVisibility = 'off'; end %#ok<SAGROW>
        plot(ax, D(i).t, D(i).T{:, "r_pos." + axis_names(a)}, ':k', 'LineWidth', line_w, ...
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
    styleAxes(ax, t_end, font_name, font_size, line_w);
    ylabel(ax, sprintf('%s (m)', axis_names(a)), 'FontWeight', 'bold', 'FontSize', font_size, ...
           'FontName', font_name);
end
hZ = ax;

% Zoom pencerelerini ana z grafiğinde kesikli çizgilerle işaretle
for w = 1:size(zoom_windows, 1)
    for tw = zoom_windows(w, :)
        line(hZ, [tw tw], zoom_zlim, 'Color', zoom_colors{w}, 'LineStyle', '--', ...
             'LineWidth', line_w, 'HandleVisibility', 'off');
    end
end

% Zoom inset'leri (ana z grafiğinin altına hizalı)
main_pos     = get(hZ, 'Position');
inset_height = main_pos(4) * 0.26;
inset_y      = main_pos(2) + 0.03;
inset_x      = [0.05 0.35];
for w = 1:size(zoom_windows, 1)
    ax_zoom = axes(fig, 'Position', [main_pos(1) + inset_x(w), inset_y, main_pos(3) * 0.25, inset_height]);
    box(ax_zoom, 'on'); hold(ax_zoom, 'on');
    for i = valid
        plot(ax_zoom, D(i).t, D(i).T{:, "a_pos.z"}, 'Color', colors(i,:), 'LineWidth', line_w);
        plot(ax_zoom, D(i).t, D(i).T{:, "r_pos.z"}, ':k', 'LineWidth', line_w);
    end
    xlim(ax_zoom, zoom_windows(w, :));
    ylim(ax_zoom, zoom_zlim);
    set(ax_zoom, 'FontSize', 9, 'FontName', font_name, 'XTick', [], 'YTick', [], 'LineWidth', 1.5);
end

addTopLegend(lgd_handles, [0.1 0.94 0.8 0.05], font_name, font_size);
saveFigure(fig, eps_folder, 'Fig6a.eps', output_pdf);

%% === 4. TÜM DRONE'LAR - 3D POZİSYON HATASI ===
fig = figure;
ax = axes(fig); hold(ax, 'on');
lgd_handles = gobjects(0);
pos_err = cell(num_drones, 1);
for i = valid
    T = D(i).T;
    e = [T{:, "a_pos.x"} - T{:, "r_pos.x"}, T{:, "a_pos.y"} - T{:, "r_pos.y"}, T{:, "a_pos.z"} - T{:, "r_pos.z"}];
    pos_err{i} = e;
    lgd_handles(end+1) = plot(ax, D(i).t, vecnorm(e, 2, 2), 'LineWidth', line_w, 'Color', colors(i,:), ...
                              'DisplayName', sprintf('Q %d', i)); %#ok<SAGROW>
end
styleAxes(ax, t_end, font_name, font_size, line_w);
xlabel(ax, 't (s)', 'FontWeight', 'bold', 'FontSize', font_size, 'FontName', font_name);
ylabel(ax, '||e_p|| (m)', 'FontWeight', 'bold', 'FontSize', font_size, 'FontName', font_name);
addTopLegend(lgd_handles, [0.1 0.92 0.8 0.05], font_name, font_size);
saveFigure(fig, eps_folder, '08_all_drones_position_error.eps', output_pdf);

%% === 5. PERFORMANS METRİKLERİ ===
fprintf('\n=====================================================================\n');
fprintf('   PERFORMANS METRİKLERİ (RMSE: ortalama, ISE: toplam, MAX: en büyük)\n');
fprintf('=====================================================================\n');
for i = 1:num_drones
    if ~D(i).ok
        fprintf('Q %d: Dosya bulunamadı.\n', i);
        continue;
    end
    sq_error = sum(pos_err{i}.^2, 2);
    dt = [0; diff(D(i).t)];
    rmse_3d = sqrt(mean(sq_error));
    ise_3d  = sum(sq_error .* dt);
    max_3d  = sqrt(max(sq_error));
    fprintf('Q %d -> RMSE: %.5f m | ISE: %.5f m^2s | MAX: %.5f m\n', i, rmse_3d, ise_3d, max_3d);
end
fprintf('=====================================================================\n');
fprintf('Grafikler "%s" klasörüne ve "%s" dosyasına kaydedildi.\n', eps_folder, output_pdf);

%% === YARDIMCI FONKSİYONLAR ===
function lim = padLimits(vals, frac)
    % Verinin [min max] aralığını her iki yönde frac oranında genişletir
    vals = vals(isfinite(vals));
    if isempty(vals), lim = [-1 1]; return; end
    lo = min(vals); hi = max(vals);
    span = hi - lo;
    if span < 1e-9, span = max(abs(hi), 1) * 0.2; end
    lim = [lo - frac * span, hi + frac * span];
end

function addZones(ax, zones, cols)
    % Zaman bölgelerini mevcut y sınırlarında arka plana boyar
    yl = ylim(ax);
    p = gobjects(size(zones, 1), 1);
    for z = 1:size(zones, 1)
        p(z) = patch(ax, [zones(z, 1) zones(z, 2) zones(z, 2) zones(z, 1)], [yl(1) yl(1) yl(2) yl(2)], ...
                     cols(z, :), 'EdgeColor', 'none', 'HandleVisibility', 'off');
    end
    uistack(p, 'bottom');
    ylim(ax, yl);
end

function styleAxes(ax, t_end, font_name, font_size, line_w)
    grid(ax, 'off');
    xlim(ax, [0 t_end]);
    set(ax, 'FontName', font_name, 'FontSize', font_size, 'FontWeight', 'bold', ...
        'LineWidth', line_w, 'Box', 'on', 'Layer', 'top');
end

function addTopLegend(handles, position, font_name, font_size)
    lgd = legend(handles, 'Orientation', 'horizontal', 'FontSize', font_size, 'FontWeight', 'bold', ...
                 'Box', 'off', 'FontName', font_name);
    set(lgd, 'Position', position);
end

function saveFigure(fig, eps_folder, eps_name, output_pdf)
    exportgraphics(fig, fullfile(eps_folder, eps_name), 'ContentType', 'vector');
    exportgraphics(fig, output_pdf, 'Append', true);
    close(fig);
end
