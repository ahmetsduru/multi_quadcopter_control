%% Fig. S2 - DOWNWASH BOZUCU MODELİ: KUVVET VE HIZ ALANI (gövde x-z düzlemi)
% Simülasyondaki modelle aynı (src/sim/downwash_model.cpp), TEK pervane için (şekil açıklamasındaki gibi;
% simülasyonda drone başına toplam kuvvet bunun 4 katıdır):
%   f_static = K * n / (dz^2 + z0^2) * exp(-r^2 / (2 sigma^2))
%   f_total  = f_static + |f_static| .* (turb_intensity * pink_noise)
%   v        = sqrt(|f_total| / (rho * C_coup * 4))
% Parametreler launch/sequential_sim.launch varsayılanlarıyla aynıdır; değiştirdiyseniz burada da güncelleyin.

clc; clear; close all;

%% === AYARLAR ===
m_drone        = 0.382;          % kaynak (üstteki) drone kütlesi (kg)
g              = 9.81;
C_coup         = 0.065;          % c_coup (Tablo S2)
r_lat          = 0.05;           % r_lat
z0             = sqrt(0.1);      % z0 (z0^2 = 0.1, Denk. 8)
sigma          = 0.15;           % sigma (m)
turb_intensity = 0.30;           % lambda (Tablo S2)
rho_air        = 1.225;
noise_seed     = 42;             % gürültü haritası tekrarlanabilir olsun

script_dir = fileparts(mfilename('fullpath'));
if isempty(script_dir), script_dir = pwd; end
eps_folder = fullfile(script_dir, 'EPS_Results');
if ~exist(eps_folder, 'dir'), mkdir(eps_folder); end

font_name = 'Times New Roman';

%% === MODEL ===
thrust_per_rotor = m_drone * g / 4;
K_z = thrust_per_rotor * C_coup;
K   = diag([K_z * r_lat, K_z * r_lat, K_z]);

x_range = -1.5 : 0.02 : 1.5;
z_range = -2.5 : 0.02 : -0.1;
[X, Z]  = meshgrid(x_range, z_range);

rng(noise_seed);
[rows, cols] = size(X);
pink = {pinkNoiseField(rows, cols), pinkNoiseField(rows, cols), pinkNoiseField(rows, cols)};

F_mag = zeros(size(X));
V_mag = zeros(size(X));
for idx = 1:numel(X)
    p_rel = [X(idx); 0; Z(idx)];          % hedef - kaynak (kaynak orijinde)
    dist  = norm(p_rel);
    if dist < 0.01 || p_rel(3) >= 0, continue; end
    n_vec = p_rel / dist;
    d_z   = -p_rel(3);
    r2    = p_rel(1)^2 + p_rel(2)^2;

    decay    = 1 / (d_z^2 + z0^2) * exp(-r2 / (2 * sigma^2));
    f_static = K * n_vec * decay;
    noise    = [pink{1}(idx); pink{2}(idx); pink{3}(idx)];
    f_total  = f_static + abs(f_static) .* (turb_intensity * noise);   % tek pervane

    F_mag(idx) = norm(f_total);
    V_mag(idx) = sqrt(F_mag(idx) / (rho_air * C_coup * 4 + eps));
end

%% === GÖRSELLEŞTİRME ===
fig = figure('Color', 'w', 'Position', [100, 100, 1200, 600]);
tl  = tiledlayout(fig, 1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
fields = {F_mag, V_mag};
titles = {'Force Field [N]', 'Velocity Field [m/s]'};
for k = 1:2
    ax = nexttile(tl);
    contourf(ax, X, Z, fields{k}, 100, 'LineStyle', 'none');
    hold(ax, 'on');
    plot(ax, 0, 0, 'kp', 'MarkerSize', 14, 'MarkerFaceColor', 'r');
    text(ax, 0.1, 0.1, 'Source', 'Color', 'k', 'FontWeight', 'bold', 'FontSize', 14, 'FontName', font_name);
    colormap(ax, jet);
    clim_max = max(fields{k}(:));
    if clim_max > 0, caxis(ax, [0 clim_max]); end
    cb = colorbar(ax);
    set(cb, 'LineWidth', 2, 'FontSize', 14, 'FontWeight', 'bold', 'FontName', font_name);
    title(ax, titles{k}, 'FontSize', 14, 'FontWeight', 'bold', 'FontName', font_name);
    xlabel(ax, 'x (m)', 'FontSize', 14, 'FontWeight', 'bold', 'FontName', font_name);
    ylabel(ax, 'z (m)', 'FontSize', 14, 'FontWeight', 'bold', 'FontName', font_name);
    axis(ax, 'equal'); xlim(ax, [-0.7 0.7]); ylim(ax, [-2.5 0.5]);
    set(ax, 'LineWidth', 2, 'FontSize', 14, 'FontWeight', 'bold', 'Box', 'on', 'FontName', font_name);
end

exportgraphics(fig, fullfile(eps_folder, 'FigS2.eps'), 'ContentType', 'vector');
fprintf('Fig. S2 kaydedildi: %s\n', fullfile(eps_folder, 'FigS2.eps'));

%% === PINK NOISE ALANI (1/f, spektral sentez) ===
function field = pinkNoiseField(rows, cols)
    F = fftshift(fft2(randn(rows, cols)));
    [cx, cy] = meshgrid(-(cols/2):(cols/2 - 1), -(rows/2):(rows/2 - 1));
    radius = sqrt(cx.^2 + cy.^2);
    radius(radius == 0) = 1;
    field = real(ifft2(ifftshift(F ./ radius)));
    field = field / max(abs(field(:)));
end
