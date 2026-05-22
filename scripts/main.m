clear all;
clc;
num_drones = 5;
% Dizin yolunu kendi bilgisayarına göre tekrar kontrol et
log_dir = '/home/asd/catkin_ws/src/multi_quadcopter_control/log/';

% Çıktı dosyası ve klasörü ayarları
output_pdf = 'all_drones_plots_fixed.pdf'; 
eps_folder = 'EPS_Results'; % EPS dosyalarının kaydedileceği klasör

% PDF varsa sil (temiz başlangıç)
if isfile(output_pdf)
    delete(output_pdf);
end

% EPS klasörü yoksa oluştur
if ~exist(eps_folder, 'dir')
    mkdir(eps_folder);
end

colors = lines(num_drones);

% === VERİ SAKLAMA HÜCRELERİ ===
thrust_all = cell(num_drones, 1);
time_all = cell(num_drones, 1);

% İvme verilerini zoom grafiğinde tekrar okumamak için hafızaya alıyoruz
acc_x_all = cell(num_drones, 1);
acc_y_all = cell(num_drones, 1);
acc_z_all = cell(num_drones, 1);
ref_acc_x = cell(num_drones, 1);
ref_acc_y = cell(num_drones, 1);
ref_acc_z = cell(num_drones, 1);


%% === VERİ OKUMA DÖNGÜSÜ ===
for i = 1:num_drones
    filename = fullfile(log_dir, sprintf('drone%d_log.txt', i));
    if ~isfile(filename)
        warning('File not found: %s', filename);
        continue;
    end

    data = readtable(filename, 'VariableNamingRule', 'preserve');
    time = data{:, "time"} - data{1, "time"};
    time_all{i} = time;

    % Thrust verilerini sakla
    thrust_all{i} = data{:, "r_thr"};
    
    % İvme verilerini sakla
    acc_x_all{i} = data{:, "a_acc.x"};
    acc_y_all{i} = data{:, "a_acc.y"};
    acc_z_all{i} = data{:, "a_acc.z"};
    
    ref_acc_x{i} = data{:, "r_acc.x"};
    ref_acc_y{i} = data{:, "r_acc.y"};
    ref_acc_z{i} = data{:, "r_acc.z"};
end

% === RENK VE ZAMAN TANIMLARI ===
t_green = [23.5 33.5 33.5 23.5];       color_green = [0.85 1 0.85];
t_yellow = [33.5 60 60 33.5];      color_yellow = [1 1 0.9];

t_zone1 = [0 8 8 0];      c_zone1 = [1 0.88 0.88];    % Pembe
t_zone2 = [8 21 21 8];    c_zone2 = [0.85 1 1];       % Mavi
t_zone3 = [21 33 33 21];  c_zone3 = [0.85 1 0.85];    % Yeşil
t_zone4 = [33 60 60 33];  c_zone4 = [1 1 0.9];        % Sarı


%% === 1. TÜM DRONE'LAR - THRUST ===
fig = figure;
hold on;
lgd_handles = [];
for i = 1:num_drones
    if ~isempty(thrust_all{i})
        h = plot(time_all{i}, thrust_all{i}, 'LineWidth', 2, 'Color', colors(i,:), 'DisplayName', sprintf('Q %d', i));
        lgd_handles = [lgd_handles, h];
    end
end
yl = ylim; yl = [yl(1)*0.9, yl(2)*1.1]; ylim(yl); 
p1 = patch(t_green, [yl(1) yl(1) yl(2) yl(2)], color_green, 'EdgeColor', 'none', 'HandleVisibility', 'off');
p2 = patch(t_yellow, [yl(1) yl(1) yl(2) yl(2)], color_yellow, 'EdgeColor', 'none', 'HandleVisibility', 'off');
uistack(p1, 'bottom'); uistack(p2, 'bottom');
grid off; xlim([0 60]); ylim([3 6.8]);
set(gca, 'FontName', 'Times New Roman', 'FontSize', 20, 'FontWeight', 'bold', 'LineWidth', 2, 'Box', 'on', 'Layer', 'top');
xlabel('t (s)', 'FontWeight', 'bold', 'FontSize', 20, 'FontName', 'Times New Roman');
ylabel('||f_{thrust}|| (N)', 'FontWeight', 'bold', 'FontSize', 20, 'FontName', 'Times New Roman');

lgd = legend(lgd_handles, 'Orientation', 'horizontal', 'FontSize', 20, 'FontWeight', 'bold', 'Box', 'off', 'FontName', 'Times New Roman');
set(lgd, 'Position', [0.15 0.94 0.7 0.05]); 

% KAYDETME
eps_name = fullfile(eps_folder, 'Fig14.eps');
exportgraphics(fig, eps_name, 'ContentType', 'vector');
exportgraphics(fig, output_pdf, 'Append', true);
close(fig);


%% === 13. YENİ EKLENEN: TÜM DRONE'LAR - NDOB ESTIMATED DISTURBANCES ===
fig = figure;

% --- Delta Fx ---
subplot(3,1,1); hold on;
lgd_handles = [];
for i = 1:num_drones
    filename = fullfile(log_dir, sprintf('drone%d_log.txt', i));
    if isfile(filename)
        data = readtable(filename, 'VariableNamingRule', 'preserve');
        time = data{:, "time"} - data{1, "time"};
        
        % Sütun adındaki nokta/alt çizgi kontrolü
        if ismember("ndob_f.x", data.Properties.VariableNames)
            ndob_val = data{:, "ndob_f.x"};
        elseif ismember("ndob_f_x", data.Properties.VariableNames)
            ndob_val = data{:, "ndob_f_x"};
        else
            ndob_val = zeros(size(time));
        end
        
        h = plot(time, ndob_val, 'LineWidth', 2, 'Color', colors(i,:), 'DisplayName', sprintf('Q %d', i));
        lgd_handles = [lgd_handles, h];
    end
end
yl = ylim; yl = [yl(1)*1.1, yl(2)*1.1]; if yl(1)==yl(2), yl=[-0.5, 0.5]; end, ylim(yl);
p1 = patch(t_green, [yl(1) yl(1) yl(2) yl(2)], color_green, 'EdgeColor', 'none', 'HandleVisibility', 'off');
p2 = patch(t_yellow, [yl(1) yl(1) yl(2) yl(2)], color_yellow, 'EdgeColor', 'none', 'HandleVisibility', 'off');
uistack(p1, 'bottom'); uistack(p2, 'bottom');
ylabel('\Delta\hat{f}_x (N)', 'FontWeight', 'bold', 'FontSize', 20, 'FontName', 'Times New Roman'); 
grid off; xlim([0 60]); set(gca, 'FontName', 'Times New Roman', 'FontSize', 20, 'FontWeight', 'bold', 'LineWidth', 2, 'Box', 'on', 'Layer', 'top');

% --- Delta Fy ---
subplot(3,1,2); hold on;
for i = 1:num_drones
    filename = fullfile(log_dir, sprintf('drone%d_log.txt', i));
    if isfile(filename)
        data = readtable(filename, 'VariableNamingRule', 'preserve');
        time = data{:, "time"} - data{1, "time"};
        
        if ismember("ndob_f.y", data.Properties.VariableNames)
            ndob_val = data{:, "ndob_f.y"};
        elseif ismember("ndob_f_y", data.Properties.VariableNames)
            ndob_val = data{:, "ndob_f_y"};
        else
            ndob_val = zeros(size(time));
        end
        
        plot(time, ndob_val, 'LineWidth', 2, 'Color', colors(i,:), 'HandleVisibility', 'off');
    end
end
yl = ylim; yl = [yl(1)*1.1, yl(2)*1.1]; if yl(1)==yl(2), yl=[-0.5, 0.5]; end, ylim(yl);
p1 = patch(t_green, [yl(1) yl(1) yl(2) yl(2)], color_green, 'EdgeColor', 'none', 'HandleVisibility', 'off');
p2 = patch(t_yellow, [yl(1) yl(1) yl(2) yl(2)], color_yellow, 'EdgeColor', 'none', 'HandleVisibility', 'off');
uistack(p1, 'bottom'); uistack(p2, 'bottom');
ylabel('\Delta\hat{f}_y (N)', 'FontWeight', 'bold', 'FontSize', 20, 'FontName', 'Times New Roman'); 
grid off; xlim([0 60]); set(gca, 'FontName', 'Times New Roman', 'FontSize', 20, 'FontWeight', 'bold', 'LineWidth', 2, 'Box', 'on', 'Layer', 'top');

% --- Delta Fz ---
subplot(3,1,3); hold on;
for i = 1:num_drones
    filename = fullfile(log_dir, sprintf('drone%d_log.txt', i));
    if isfile(filename)
        data = readtable(filename, 'VariableNamingRule', 'preserve');
        time = data{:, "time"} - data{1, "time"};
        
        if ismember("ndob_f.z", data.Properties.VariableNames)
            ndob_val = data{:, "ndob_f.z"};
        elseif ismember("ndob_f_z", data.Properties.VariableNames)
            ndob_val = data{:, "ndob_f_z"};
        else
            ndob_val = zeros(size(time));
        end
        
        plot(time, ndob_val, 'LineWidth', 2, 'Color', colors(i,:), 'HandleVisibility', 'off');
    end
end

% Kıyaslamayı kusursuzlaştırmak için Bölüm 2 rüzgar grafiğindeki Z limitlerinin aynısı kilitlendi
ylim([-2.5, 1.0]);
yl = ylim;         
p1 = patch(t_green, [yl(1) yl(1) yl(2) yl(2)], color_green, 'EdgeColor', 'none', 'HandleVisibility', 'off');
p2 = patch(t_yellow, [yl(1) yl(1) yl(2) yl(2)], color_yellow, 'EdgeColor', 'none', 'HandleVisibility', 'off');
uistack(p1, 'bottom'); uistack(p2, 'bottom');
ylabel('\Delta\hat{f}_z (N)', 'FontWeight', 'bold', 'FontSize', 20, 'FontName', 'Times New Roman'); 
xlabel('t (s)', 'FontWeight', 'bold', 'FontSize', 20, 'FontName', 'Times New Roman'); 
grid off; xlim([0 60]); set(gca, 'FontName', 'Times New Roman', 'FontSize', 20, 'FontWeight', 'bold', 'LineWidth', 2, 'Box', 'on', 'Layer', 'top');

% Üst taraftaki yatay ortak legend yerleşimi
lgd = legend(lgd_handles, 'Orientation', 'horizontal', 'FontSize', 20, 'FontWeight', 'bold', 'Box', 'off', 'FontName', 'Times New Roman');
set(lgd, 'Position', [0.1 0.94 0.8 0.05]); 

% KAYDETME
eps_name = fullfile(eps_folder, '13_all_drones_ndob_estimation.eps');
exportgraphics(fig, eps_name, 'ContentType', 'vector');
exportgraphics(fig, output_pdf, 'Append', true);
close(fig);
fprintf('NDOB kuvvet tahmin grafiği, rüzgar grafiğinin ikizi formatında (3 eksen üst üste) başarıyla kaydedildi.\n');

%% === 2. TÜM DRONE'LAR - DISTURBANCE FORCES ===
fig = figure;

% --- Fx ---
subplot(3,1,1); hold on;
lgd_handles = [];
for i = 1:num_drones
    filename = fullfile(log_dir, sprintf('drone%d_log.txt', i));
    if isfile(filename)
        data = readtable(filename, 'VariableNamingRule', 'preserve');
        h = plot(data{:, "time"} - data{1, "time"}, data{:, "dist_f.x"}, 'LineWidth', 2, 'Color', colors(i,:), 'DisplayName', sprintf('Q %d', i));
        lgd_handles = [lgd_handles, h];
    end
end
yl = ylim; yl = [yl(1)*1.1, yl(2)*1.1]; ylim(yl);
p1 = patch(t_green, [yl(1) yl(1) yl(2) yl(2)], color_green, 'EdgeColor', 'none', 'HandleVisibility', 'off');
p2 = patch(t_yellow, [yl(1) yl(1) yl(2) yl(2)], color_yellow, 'EdgeColor', 'none', 'HandleVisibility', 'off');
uistack(p1, 'bottom'); uistack(p2, 'bottom');
ylabel('f_{dist,x} (N)', 'FontWeight', 'bold', 'FontSize', 20, 'FontName', 'Times New Roman'); 
grid off; xlim([0 60]); set(gca, 'FontName', 'Times New Roman', 'FontSize', 20, 'FontWeight', 'bold', 'LineWidth', 2, 'Box', 'on', 'Layer', 'top');

% --- Fy ---
subplot(3,1,2); hold on;
for i = 1:num_drones
    filename = fullfile(log_dir, sprintf('drone%d_log.txt', i));
    if isfile(filename)
        data = readtable(filename, 'VariableNamingRule', 'preserve');
        plot(data{:, "time"} - data{1, "time"}, data{:, "dist_f.y"}, 'LineWidth', 2, 'Color', colors(i,:), 'HandleVisibility', 'off');
    end
end
yl = ylim; yl = [yl(1)*1.1, yl(2)*1.1]; ylim(yl);
p1 = patch(t_green, [yl(1) yl(1) yl(2) yl(2)], color_green, 'EdgeColor', 'none', 'HandleVisibility', 'off');
p2 = patch(t_yellow, [yl(1) yl(1) yl(2) yl(2)], color_yellow, 'EdgeColor', 'none', 'HandleVisibility', 'off');
uistack(p1, 'bottom'); uistack(p2, 'bottom');
ylabel('f_{dist,y} (N)', 'FontWeight', 'bold', 'FontSize', 20, 'FontName', 'Times New Roman'); 
grid off; xlim([0 60]); set(gca, 'FontName', 'Times New Roman', 'FontSize', 20, 'FontWeight', 'bold', 'LineWidth', 2, 'Box', 'on', 'Layer', 'top');

% --- Fz (GÜNCELLENDİ) ---
subplot(3,1,3); hold on;
for i = 1:num_drones
    filename = fullfile(log_dir, sprintf('drone%d_log.txt', i));
    if isfile(filename)
        data = readtable(filename, 'VariableNamingRule', 'preserve');
        plot(data{:, "time"} - data{1, "time"}, data{:, "dist_f.z"}, 'LineWidth', 2, 'Color', colors(i,:), 'HandleVisibility', 'off');
    end
end

% --- AYAR BURADA YAPILDI ---
ylim([-2.5, 0.2]); % Alt sınır -1.5, üst sınır 0.1 olarak ayarlandı
yl = ylim;         % Patch çizimi için yeni limitleri hafızaya al

p1 = patch(t_green, [yl(1) yl(1) yl(2) yl(2)], color_green, 'EdgeColor', 'none', 'HandleVisibility', 'off');
p2 = patch(t_yellow, [yl(1) yl(1) yl(2) yl(2)], color_yellow, 'EdgeColor', 'none', 'HandleVisibility', 'off');
uistack(p1, 'bottom'); uistack(p2, 'bottom');
ylabel('f_{dist,z} (N)', 'FontWeight', 'bold', 'FontSize', 20, 'FontName', 'Times New Roman'); 
xlabel('t (s)', 'FontWeight', 'bold', 'FontSize', 20, 'FontName', 'Times New Roman'); 
grid off; xlim([0 60]); set(gca, 'FontName', 'Times New Roman', 'FontSize', 20, 'FontWeight', 'bold', 'LineWidth', 2, 'Box', 'on', 'Layer', 'top');

lgd = legend(lgd_handles, 'Orientation', 'horizontal', 'FontSize', 20, 'FontWeight', 'bold', 'Box', 'off', 'FontName', 'Times New Roman');
set(lgd, 'Position', [0.1 0.94 0.8 0.05]); 

% KAYDETME
eps_name = fullfile(eps_folder, 'Fig13.eps');
exportgraphics(fig, eps_name, 'ContentType', 'vector');
exportgraphics(fig, output_pdf, 'Append', true);
close(fig);

%% === 3. TÜM DRONE'LAR - POZİSYON (Z-AXIS DOUBLE ZOOM) ===
fig = figure;

%% ===================== X =====================
subplot(3,1,1); hold on;
lgd_handles = [];
h_des = plot(NaN, NaN, ':k', 'LineWidth', 2, 'DisplayName', 'Desired');
lgd_handles = [lgd_handles, h_des];

for i = 1:num_drones
    filename = fullfile(log_dir, sprintf('drone%d_log.txt', i));
    if isfile(filename)
        data = readtable(filename,'VariableNamingRule','preserve');
        time = data{:, "time"} - data{1,"time"};
        h = plot(time, data{:, "a_pos.x"}, '-', 'Color', colors(i,:), ...
                 'LineWidth',2,'DisplayName',sprintf('Q %d',i));
        lgd_handles = [lgd_handles, h];
        plot(time, data{:, "r_pos.x"}, ':k', 'LineWidth',2,'HandleVisibility','off');
    end
end

yl = ylim; yl = [yl(1)-1 yl(2)+1]; ylim(yl);
p1 = patch(t_zone1,[yl(1) yl(1) yl(2) yl(2)],c_zone1,'EdgeColor','none');
p2 = patch(t_zone2,[yl(1) yl(1) yl(2) yl(2)],c_zone2,'EdgeColor','none');
p3 = patch(t_zone3,[yl(1) yl(1) yl(2) yl(2)],c_zone3,'EdgeColor','none');
p4 = patch(t_zone4,[yl(1) yl(1) yl(2) yl(2)],c_zone4,'EdgeColor','none');
uistack([p1 p2 p3 p4],'bottom');

ylabel('x (m)','FontWeight','bold','FontSize',20,'FontName','Times New Roman');
grid off; xlim([0 60]);
set(gca,'FontName','Times New Roman','FontSize',20,'FontWeight','bold',...
    'LineWidth',2,'Box','on','Layer','top');

%% ===================== Y =====================
subplot(3,1,2); hold on;
for i = 1:num_drones
    filename = fullfile(log_dir, sprintf('drone%d_log.txt', i));
    if isfile(filename)
        data = readtable(filename,'VariableNamingRule','preserve');
        time = data{:, "time"} - data{1,"time"};
        plot(time, data{:, "a_pos.y"}, '-', 'Color', colors(i,:), 'LineWidth',2);
        plot(time, data{:, "r_pos.y"}, ':k', 'LineWidth',2);
    end
end

yl = ylim; yl = [yl(1)-1 yl(2)+1]; ylim(yl);
p1 = patch(t_zone1,[yl(1) yl(1) yl(2) yl(2)],c_zone1,'EdgeColor','none');
p2 = patch(t_zone2,[yl(1) yl(1) yl(2) yl(2)],c_zone2,'EdgeColor','none');
p3 = patch(t_zone3,[yl(1) yl(1) yl(2) yl(2)],c_zone3,'EdgeColor','none');
p4 = patch(t_zone4,[yl(1) yl(1) yl(2) yl(2)],c_zone4,'EdgeColor','none');
uistack([p1 p2 p3 p4],'bottom');

ylabel('y (m)','FontWeight','bold','FontSize',20,'FontName','Times New Roman');
grid off; xlim([0 60]);
set(gca,'FontName','Times New Roman','FontSize',20,'FontWeight','bold',...
    'LineWidth',2,'Box','on','Layer','top');

%% ===================== Z =====================
hZ = subplot(3,1,3); hold on;

for i = 1:num_drones
    filename = fullfile(log_dir, sprintf('drone%d_log.txt', i));
    if isfile(filename)
        data = readtable(filename,'VariableNamingRule','preserve');
        time = data{:, "time"} - data{1,"time"};

        plot(time, data{:, "a_pos.z"}, '-', ...
             'Color', colors(i,:), 'LineWidth',2);
        plot(time, data{:, "r_pos.z"}, ':k', ...
             'LineWidth',2,'HandleVisibility','off');

        % inset için sakla
        time_all{i}  = time;
        acc_z_all{i} = data{:, "a_pos.z"};
        ref_acc_z{i} = data{:, "r_pos.z"};
    end
end

%% === Eksen ayarları ===
yl = ylim;
yl = [yl(1)-0.4 yl(2)+0.2];   % altta inset için boşluk
ylim(yl);
xlim([0 60]);

ylabel('z (m)','FontWeight','bold','FontSize',20,'FontName','Times New Roman');
xlabel('t (s)','FontWeight','bold','FontSize',20,'FontName','Times New Roman');

set(gca,'FontName','Times New Roman','FontSize',20,'FontWeight','bold',...
        'LineWidth',2,'Box','on','Layer','top');
grid off;

%% === Arka plan zonları ===
p1 = patch(t_zone1,[yl(1) yl(1) yl(2) yl(2)],c_zone1,'EdgeColor','none');
p2 = patch(t_zone2,[yl(1) yl(1) yl(2) yl(2)],c_zone2,'EdgeColor','none');
p3 = patch(t_zone3,[yl(1) yl(1) yl(2) yl(2)],c_zone3,'EdgeColor','none');
p4 = patch(t_zone4,[yl(1) yl(1) yl(2) yl(2)],c_zone4,'EdgeColor','none');
uistack([p1 p2 p3 p4],'bottom');

%% === KISA KESİKLİ ZOOM İŞARETLERİ ===
z_mark_min = 1.9;
z_mark_max = 3.0;

% 23–25 s (kırmızı)
line([23 23],[z_mark_min z_mark_max], ...
     'Color','r','LineStyle','--','LineWidth',2);
line([25 25],[z_mark_min z_mark_max], ...
     'Color','r','LineStyle','--','LineWidth',2);

% 34–36 s (mavi)
line([34 34],[z_mark_min z_mark_max], ...
     'Color','b','LineStyle','--','LineWidth',2);
line([36 36],[z_mark_min z_mark_max], ...
     'Color','b','LineStyle','--','LineWidth',2);

%% ===================== ZOOM INSET'LER (ALT HİZALI) =====================
main_pos = get(hZ,'Position'); % Ana Z grafiğinin konumu

inset_height = main_pos(4) * 0.26; % Yükseklik
inset_y      = main_pos(2) + 0.03; % Y konumu (biraz yukarı kaldır)

%% --- Zoom 1: 23–25 s (SOL TARAFA) ---
ax_zoom1 = axes('Position', ...
    [ main_pos(1) + 0.05, ...  % X konumu
      inset_y, ...
      main_pos(3) * 0.25, ...  % Genişlik
      inset_height ]);

box on; hold on;
for i = 1:num_drones
    plot(time_all{i}, acc_z_all{i}, 'Color', colors(i,:), 'LineWidth', 2);
    plot(time_all{i}, ref_acc_z{i}, ':k', 'LineWidth', 2);
end

xlim([23 25]);
ylim([z_mark_min z_mark_max]);

% DEĞİŞİKLİK BURADA: Title silindi, XTick ve YTick boş bırakıldı
set(gca, 'FontSize', 9, 'FontName', 'Times New Roman', ...
         'XTick', [], 'YTick', [], 'LineWidth', 1.5);

%% --- Zoom 2: 34–36 s (SAĞ TARAFA) ---
ax_zoom2 = axes('Position', ...
    [ main_pos(1) + 0.35, ...  % X konumu
      inset_y, ...
      main_pos(3) * 0.25, ...  % Genişlik
      inset_height ]);

box on; hold on;
for i = 1:num_drones
    plot(time_all{i}, acc_z_all{i}, 'Color', colors(i,:), 'LineWidth', 2);
    plot(time_all{i}, ref_acc_z{i}, ':k', 'LineWidth', 2);
end

xlim([34 36]);
ylim([z_mark_min z_mark_max]);

% DEĞİŞİKLİK BURADA: Title silindi, XTick ve YTick boş bırakıldı
set(gca, 'FontSize', 9, 'FontName', 'Times New Roman', ...
         'XTick', [], 'YTick', [], 'LineWidth', 1.5);

%% ... (Önceki kodlarınız: subplotlar ve zoom kutuları bittikten hemen sonra) ...

% ===================== LEGEND EKLEME =====================
% Legend handles zaten 1. subplotta toplanmıştı (lgd_handles)
lgd = legend(lgd_handles, 'Orientation', 'horizontal', ...
    'FontSize', 20, 'FontWeight', 'bold', ...
    'Box', 'off', 'FontName', 'Times New Roman');

% Legend'ı sayfanın en tepesine ortalayarak yerleştir
set(lgd, 'Position', [0.1 0.94 0.8 0.05]); 

%% ===================== KAYDETME =====================
%eps_name = fullfile(eps_folder, 'Fig6a.eps'); % Dosya ismini kontrol edin
%exportgraphics(fig, eps_name, 'ContentType', 'vector');
%exportgraphics(fig, output_pdf, 'Append', true);
%close(fig);


%%% === 5. TÜM DRONE'LAR - VELOCITY ===
%fig = figure;
%
%% Vx
%subplot(3,1,1); hold on;
%lgd_handles = [];
%h_des = plot(NaN, NaN, ':k', 'LineWidth', 2, 'DisplayName', 'Desired');
%lgd_handles = [lgd_handles, h_des];
%
%for i = 1:num_drones
%    filename = fullfile(log_dir, sprintf('drone%d_log.txt', i));
%    if isfile(filename)
%        data = readtable(filename, 'VariableNamingRule', 'preserve');
%        time = data{:, "time"} - data{1, "time"};
%        h = plot(time, data{:, "a_vel.x"}, '-', 'Color', colors(i,:), 'LineWidth', 2, 'DisplayName', sprintf('Q %d', i));
%        lgd_handles = [lgd_handles, h];
%        plot(time, data{:, "r_vel.x"}, ':k', 'LineWidth', 2, 'HandleVisibility', 'off');
%    end
%end
%yl = ylim; yl = [yl(1)-0.5, yl(2)+0.5]; ylim(yl);
%p1 = patch(t_zone1, [yl(1) yl(1) yl(2) yl(2)], c_zone1, 'EdgeColor', 'none', 'HandleVisibility', 'off');
%p2 = patch(t_zone2, [yl(1) yl(1) yl(2) yl(2)], c_zone2, 'EdgeColor', 'none', 'HandleVisibility', 'off');
%p3 = patch(t_zone3, [yl(1) yl(1) yl(2) yl(2)], c_zone3, 'EdgeColor', 'none', 'HandleVisibility', 'off');
%p4 = patch(t_zone4, [yl(1) yl(1) yl(2) yl(2)], c_zone4, 'EdgeColor', 'none', 'HandleVisibility', 'off');
%uistack(p1,'bottom'); uistack(p2,'bottom'); uistack(p3,'bottom'); uistack(p4,'bottom');
%ylabel('v_x (m/s)', 'FontWeight', 'bold', 'FontSize', 20, 'FontName', 'Times New Roman'); 
%grid off; xlim([0 60]); set(gca, 'FontName', 'Times New Roman', 'FontSize', 20, 'FontWeight', 'bold', 'LineWidth', 2, 'Box', 'on', 'Layer', 'top');
%
%% Vy
%subplot(3,1,2); hold on;
%for i = 1:num_drones
%    filename = fullfile(log_dir, sprintf('drone%d_log.txt', i));
%    if isfile(filename)
%        data = readtable(filename, 'VariableNamingRule', 'preserve');
%        time = data{:, "time"} - data{1, "time"};
%        plot(time, data{:, "a_vel.y"}, '-', 'Color', colors(i,:), 'LineWidth', 2);
%        plot(time, data{:, "r_vel.y"}, ':k', 'LineWidth', 2);
%    end
%end
%yl = ylim; yl = [yl(1)-0.5, yl(2)+0.5]; ylim(yl);
%p1 = patch(t_zone1, [yl(1) yl(1) yl(2) yl(2)], c_zone1, 'EdgeColor', 'none', 'HandleVisibility', 'off');
%p2 = patch(t_zone2, [yl(1) yl(1) yl(2) yl(2)], c_zone2, 'EdgeColor', 'none', 'HandleVisibility', 'off');
%p3 = patch(t_zone3, [yl(1) yl(1) yl(2) yl(2)], c_zone3, 'EdgeColor', 'none', 'HandleVisibility', 'off');
%p4 = patch(t_zone4, [yl(1) yl(1) yl(2) yl(2)], c_zone4, 'EdgeColor', 'none', 'HandleVisibility', 'off');
%uistack(p1,'bottom'); uistack(p2,'bottom'); uistack(p3,'bottom'); uistack(p4,'bottom');
%ylabel('v_y (m/s)', 'FontWeight', 'bold', 'FontSize', 20, 'FontName', 'Times New Roman'); 
%grid off; xlim([0 60]); set(gca, 'FontName', 'Times New Roman', 'FontSize', 20, 'FontWeight', 'bold', 'LineWidth', 2, 'Box', 'on', 'Layer', 'top');
%
%% Vz
%subplot(3,1,3); hold on;
%for i = 1:num_drones
%    filename = fullfile(log_dir, sprintf('drone%d_log.txt', i));
%    if isfile(filename)
%        data = readtable(filename, 'VariableNamingRule', 'preserve');
%        time = data{:, "time"} - data{1, "time"};
%        plot(time, data{:, "a_vel.z"}, '-', 'Color', colors(i,:), 'LineWidth', 2);
%        plot(time, data{:, "r_vel.z"}, ':k', 'LineWidth', 2);
%    end
%end
%yl = ylim; yl = [yl(1)-0.5, yl(2)+0.5]; ylim(yl);
%p1 = patch(t_zone1, [yl(1) yl(1) yl(2) yl(2)], c_zone1, 'EdgeColor', 'none', 'HandleVisibility', 'off');
%p2 = patch(t_zone2, [yl(1) yl(1) yl(2) yl(2)], c_zone2, 'EdgeColor', 'none', 'HandleVisibility', 'off');
%p3 = patch(t_zone3, [yl(1) yl(1) yl(2) yl(2)], c_zone3, 'EdgeColor', 'none', 'HandleVisibility', 'off');
%p4 = patch(t_zone4, [yl(1) yl(1) yl(2) yl(2)], c_zone4, 'EdgeColor', 'none', 'HandleVisibility', 'off');
%uistack(p1,'bottom'); uistack(p2,'bottom'); uistack(p3,'bottom'); uistack(p4,'bottom');
%ylabel('v_z (m/s)', 'FontWeight', 'bold', 'FontSize', 20, 'FontName', 'Times New Roman'); 
%xlabel('t (s)', 'FontWeight', 'bold', 'FontSize', 20, 'FontName', 'Times New Roman'); 
%grid off; xlim([0 60]); set(gca, 'FontName', 'Times New Roman', 'FontSize', 20, 'FontWeight', 'bold', 'LineWidth', 2, 'Box', 'on', 'Layer', 'top');
%
%lgd = legend(lgd_handles, 'Orientation', 'horizontal', 'FontSize', 20, 'FontWeight', 'bold', 'Box', 'off', 'FontName', 'Times New Roman');
%set(lgd, 'Position', [0.1 0.94 0.8 0.05]);
%
%% KAYDETME
%eps_name = fullfile(eps_folder, 'Fig7.eps');
%exportgraphics(fig, eps_name, 'ContentType', 'vector');
%exportgraphics(fig, output_pdf, 'Append', true);
%close(fig);
%
%
%%% === 6. TÜM DRONE'LAR - ACCELERATION (ZOOMSUZ) ===
%fig = figure;
%
%% --- AX ---
%h1 = subplot(3,1,1); hold on;
%lgd_handles = [];
%h_des = plot(NaN, NaN, ':k', 'LineWidth', 2, 'DisplayName', 'Desired');
%lgd_handles = [lgd_handles, h_des];
%
%for i = 1:num_drones
%    if ~isempty(acc_x_all{i})
%        h = plot(time_all{i}, acc_x_all{i}, '-', 'Color', colors(i,:), 'LineWidth', 2, 'DisplayName', sprintf('Q %d', i));
%        lgd_handles = [lgd_handles, h];
%        plot(time_all{i}, ref_acc_x{i}, ':k', 'LineWidth', 2, 'HandleVisibility', 'off');
%    end
%end
%% Arka Plan
%yl = ylim; yl = [yl(1)-0.5, yl(2)+0.5]; ylim(yl);
%p1 = patch(t_zone1, [yl(1) yl(1) yl(2) yl(2)], c_zone1, 'EdgeColor', 'none', 'HandleVisibility', 'off');
%p2 = patch(t_zone2, [yl(1) yl(1) yl(2) yl(2)], c_zone2, 'EdgeColor', 'none', 'HandleVisibility', 'off');
%p3 = patch(t_zone3, [yl(1) yl(1) yl(2) yl(2)], c_zone3, 'EdgeColor', 'none', 'HandleVisibility', 'off');
%p4 = patch(t_zone4, [yl(1) yl(1) yl(2) yl(2)], c_zone4, 'EdgeColor', 'none', 'HandleVisibility', 'off');
%uistack(p1,'bottom'); uistack(p2,'bottom'); uistack(p3,'bottom'); uistack(p4,'bottom');
%
%ylabel('a_x (m/s^2)', 'FontWeight', 'bold', 'FontSize', 20, 'FontName', 'Times New Roman'); 
%grid off; xlim([0 60]); 
%set(gca, 'FontName', 'Times New Roman', 'FontSize', 20, 'FontWeight', 'bold', 'LineWidth', 2, 'Box', 'on', 'Layer', 'top');
%
%
%% --- AY ---
%h2 = subplot(3,1,2); hold on;
%for i = 1:num_drones
%    if ~isempty(acc_y_all{i})
%        plot(time_all{i}, acc_y_all{i}, '-', 'Color', colors(i,:), 'LineWidth', 2);
%        plot(time_all{i}, ref_acc_y{i}, ':k', 'LineWidth', 2);
%    end
%end
%% Arka Plan
%yl = ylim; yl = [yl(1)-0.5, yl(2)+0.5]; ylim(yl);
%p1 = patch(t_zone1, [yl(1) yl(1) yl(2) yl(2)], c_zone1, 'EdgeColor', 'none', 'HandleVisibility', 'off');
%p2 = patch(t_zone2, [yl(1) yl(1) yl(2) yl(2)], c_zone2, 'EdgeColor', 'none', 'HandleVisibility', 'off');
%p3 = patch(t_zone3, [yl(1) yl(1) yl(2) yl(2)], c_zone3, 'EdgeColor', 'none', 'HandleVisibility', 'off');
%p4 = patch(t_zone4, [yl(1) yl(1) yl(2) yl(2)], c_zone4, 'EdgeColor', 'none', 'HandleVisibility', 'off');
%uistack(p1,'bottom'); uistack(p2,'bottom'); uistack(p3,'bottom'); uistack(p4,'bottom');
%
%ylabel('a_y (m/s^2)', 'FontWeight', 'bold', 'FontSize', 20, 'FontName', 'Times New Roman'); 
%grid off; xlim([0 60]); 
%set(gca, 'FontName', 'Times New Roman', 'FontSize', 20, 'FontWeight', 'bold', 'LineWidth', 2, 'Box', 'on', 'Layer', 'top');
%
%
%% --- AZ ---
%h3 = subplot(3,1,3); hold on;
%for i = 1:num_drones
%    if ~isempty(acc_z_all{i})
%        plot(time_all{i}, acc_z_all{i}, '-', 'Color', colors(i,:), 'LineWidth', 2);
%        plot(time_all{i}, ref_acc_z{i}, ':k', 'LineWidth', 2);
%    end
%end
%% Arka Plan
%yl = ylim; yl = [yl(1)-0.5, yl(2)+0.5]; ylim(yl);
%p1 = patch(t_zone1, [yl(1) yl(1) yl(2) yl(2)], c_zone1, 'EdgeColor', 'none', 'HandleVisibility', 'off');
%p2 = patch(t_zone2, [yl(1) yl(1) yl(2) yl(2)], c_zone2, 'EdgeColor', 'none', 'HandleVisibility', 'off');
%p3 = patch(t_zone3, [yl(1) yl(1) yl(2) yl(2)], c_zone3, 'EdgeColor', 'none', 'HandleVisibility', 'off');
%p4 = patch(t_zone4, [yl(1) yl(1) yl(2) yl(2)], c_zone4, 'EdgeColor', 'none', 'HandleVisibility', 'off');
%uistack(p1,'bottom'); uistack(p2,'bottom'); uistack(p3,'bottom'); uistack(p4,'bottom');
%
%ylabel('a_z (m/s^2)', 'FontWeight', 'bold', 'FontSize', 20, 'FontName', 'Times New Roman');
%xlabel('t (s)', 'FontWeight', 'bold', 'FontSize', 20, 'FontName', 'Times New Roman'); 
%grid off; xlim([0 60]); 
%set(gca, 'FontName', 'Times New Roman', 'FontSize', 20, 'FontWeight', 'bold', 'LineWidth', 2, 'Box', 'on', 'Layer', 'top');
%
%% LEGEND
%lgd = legend(lgd_handles, 'Orientation', 'horizontal', 'FontSize', 20, 'FontWeight', 'bold', 'Box', 'off', 'FontName', 'Times New Roman');
%set(lgd, 'Position', [0.1 0.94 0.8 0.05]);
%
%% KAYDETME
%eps_name = fullfile(eps_folder, 'Fig11.eps');
%exportgraphics(fig, eps_name, 'ContentType', 'vector');
%exportgraphics(fig, output_pdf, 'Append', true);
%close(fig);
%
%
%%% === 7. TÜM DRONE'LAR - EULER AÇILARI ===
%fig = figure;
%
%% --- Roll (Phi) ---
%subplot(3,1,1); hold on;
%lgd_handles = [];
%h_des = plot(NaN, NaN, ':k', 'LineWidth', 2, 'DisplayName', 'Desired');
%lgd_handles = [lgd_handles, h_des];
%for i = 1:num_drones
%    filename = fullfile(log_dir, sprintf('drone%d_log.txt', i));
%    if isfile(filename)
%        data = readtable(filename, 'VariableNamingRule', 'preserve');
%        time = data{:, "time"} - data{1, "time"};
%        
%        h = plot(time, data{:, "a_eul_ang.x"}, '-', 'Color', colors(i,:), 'LineWidth', 2, 'DisplayName', sprintf('Q %d', i));
%        lgd_handles = [lgd_handles, h];
%        plot(time, data{:, "r_eul_ang.x"}, ':k', 'LineWidth', 2, 'HandleVisibility', 'off');
%    end
%end
%% Arka Plan
%yl = ylim; yl = [yl(1)-0.1, yl(2)+0.1]; ylim(yl);
%p1 = patch(t_zone1, [yl(1) yl(1) yl(2) yl(2)], c_zone1, 'EdgeColor', 'none', 'HandleVisibility', 'off');
%p2 = patch(t_zone2, [yl(1) yl(1) yl(2) yl(2)], c_zone2, 'EdgeColor', 'none', 'HandleVisibility', 'off');
%p3 = patch(t_zone3, [yl(1) yl(1) yl(2) yl(2)], c_zone3, 'EdgeColor', 'none', 'HandleVisibility', 'off');
%p4 = patch(t_zone4, [yl(1) yl(1) yl(2) yl(2)], c_zone4, 'EdgeColor', 'none', 'HandleVisibility', 'off');
%uistack(p1,'bottom'); uistack(p2,'bottom'); uistack(p3,'bottom'); uistack(p4,'bottom');
%ylabel('\phi (rad)', 'FontWeight', 'bold', 'FontSize', 20, 'FontName', 'Times New Roman'); 
%grid off; xlim([0 60]); set(gca, 'FontName', 'Times New Roman', 'FontSize', 20, 'FontWeight', 'bold', 'LineWidth', 2, 'Box', 'on', 'Layer', 'top');
%
%% --- Pitch (Theta) ---
%subplot(3,1,2); hold on;
%for i = 1:num_drones
%    filename = fullfile(log_dir, sprintf('drone%d_log.txt', i));
%    if isfile(filename)
%        data = readtable(filename, 'VariableNamingRule', 'preserve');
%        time = data{:, "time"} - data{1, "time"};
%        plot(time, data{:, "a_eul_ang.y"}, '-', 'Color', colors(i,:), 'LineWidth', 2);
%        plot(time, data{:, "r_eul_ang.y"}, ':k', 'LineWidth', 2);
%    end
%end
%yl = ylim; yl = [yl(1)-0.1, yl(2)+0.1]; ylim(yl);
%p1 = patch(t_zone1, [yl(1) yl(1) yl(2) yl(2)], c_zone1, 'EdgeColor', 'none', 'HandleVisibility', 'off');
%p2 = patch(t_zone2, [yl(1) yl(1) yl(2) yl(2)], c_zone2, 'EdgeColor', 'none', 'HandleVisibility', 'off');
%p3 = patch(t_zone3, [yl(1) yl(1) yl(2) yl(2)], c_zone3, 'EdgeColor', 'none', 'HandleVisibility', 'off');
%p4 = patch(t_zone4, [yl(1) yl(1) yl(2) yl(2)], c_zone4, 'EdgeColor', 'none', 'HandleVisibility', 'off');
%uistack(p1,'bottom'); uistack(p2,'bottom'); uistack(p3,'bottom'); uistack(p4,'bottom');
%ylabel('\theta (rad)', 'FontWeight', 'bold', 'FontSize', 20, 'FontName', 'Times New Roman'); 
%grid off; xlim([0 60]); set(gca, 'FontName', 'Times New Roman', 'FontSize', 20, 'FontWeight', 'bold', 'LineWidth', 2, 'Box', 'on', 'Layer', 'top');
%
%% --- Yaw (Psi) ---
%subplot(3,1,3); hold on;
%for i = 1:num_drones
%    filename = fullfile(log_dir, sprintf('drone%d_log.txt', i));
%    if isfile(filename)
%        data = readtable(filename, 'VariableNamingRule', 'preserve');
%        time = data{:, "time"} - data{1, "time"};
%        plot(time, data{:, "a_eul_ang.z"}, '-', 'Color', colors(i,:), 'LineWidth', 2);
%        plot(time, data{:, "r_eul_ang.z"}, ':k', 'LineWidth', 2);
%    end
%end
%yl = ylim; yl = [yl(1)-0.1, yl(2)+0.1]; ylim(yl);
%p1 = patch(t_zone1, [yl(1) yl(1) yl(2) yl(2)], c_zone1, 'EdgeColor', 'none', 'HandleVisibility', 'off');
%p2 = patch(t_zone2, [yl(1) yl(1) yl(2) yl(2)], c_zone2, 'EdgeColor', 'none', 'HandleVisibility', 'off');
%p3 = patch(t_zone3, [yl(1) yl(1) yl(2) yl(2)], c_zone3, 'EdgeColor', 'none', 'HandleVisibility', 'off');
%p4 = patch(t_zone4, [yl(1) yl(1) yl(2) yl(2)], c_zone4, 'EdgeColor', 'none', 'HandleVisibility', 'off');
%uistack(p1,'bottom'); uistack(p2,'bottom'); uistack(p3,'bottom'); uistack(p4,'bottom');
%ylabel('\psi (rad)', 'FontWeight', 'bold', 'FontSize', 20, 'FontName', 'Times New Roman');
%xlabel('t (s)', 'FontWeight', 'bold', 'FontSize', 20, 'FontName', 'Times New Roman'); 
%grid off; xlim([0 60]); set(gca, 'FontName', 'Times New Roman', 'FontSize', 20, 'FontWeight', 'bold', 'LineWidth', 2, 'Box', 'on', 'Layer', 'top');
%
%lgd = legend(lgd_handles, 'Orientation', 'horizontal', 'FontSize', 20, 'FontWeight', 'bold', 'Box', 'off', 'FontName', 'Times New Roman');
%set(lgd, 'Position', [0.1 0.94 0.8 0.05]);
%
%% KAYDETME
%eps_name = fullfile(eps_folder, 'Fig9.eps');
%exportgraphics(fig, eps_name, 'ContentType', 'vector');
%exportgraphics(fig, output_pdf, 'Append', true);
%close(fig);
%
%
%% === 8. TÜM DRONE'LAR - 3D POZİSYON HATASI ===
fig = figure;
hold on;
lgd_handles = [];
for i = 1:num_drones
    filename = fullfile(log_dir, sprintf('drone%d_log.txt', i));
    if isfile(filename)
        data = readtable(filename, 'VariableNamingRule', 'preserve');
        time = data{:, "time"} - data{1, "time"};
        x = data{:, "a_pos.x"};  y = data{:, "a_pos.y"};  z = data{:, "a_pos.z"};
        xr = data{:, "r_pos.x"}; yr = data{:, "r_pos.y"}; zr = data{:, "r_pos.z"};
        pos_error = sqrt((x - xr).^2 + (y - yr).^2 + (z - zr).^2);
        h = plot(time, pos_error, 'LineWidth', 2, 'Color', colors(i,:), 'DisplayName', sprintf('Q %d', i));
        lgd_handles = [lgd_handles, h];
    end
end
xlabel('t (s)', 'FontWeight', 'bold', 'FontSize', 20, 'FontName', 'Times New Roman');
ylabel('||e_p|| (m)', 'FontWeight', 'bold', 'FontSize', 20, 'FontName', 'Times New Roman');

grid off; set(gca, 'FontName', 'Times New Roman', 'FontSize', 20, 'FontWeight', 'bold', 'LineWidth', 2, 'Box', 'on');

lgd = legend(lgd_handles, 'Orientation', 'horizontal', 'FontSize', 20, 'FontWeight', 'bold', 'Box','off', 'FontName', 'Times New Roman');
set(lgd, 'Position', [0.1 0.92 0.8 0.05]);

% KAYDETME
eps_name = fullfile(eps_folder, '08_all_drones_position_error.eps');
exportgraphics(fig, eps_name, 'ContentType', 'vector');
exportgraphics(fig, output_pdf, 'Append', true);
close(fig);

% HATA HESAPLAMA EKRAN ÇIKTISI
fprintf('\n=======================================================\n');
fprintf('   PERFORMANS METRİKLERİ (RMSE: Ortalama, ISE: Toplam)   \n');
fprintf('=======================================================\n');

for i = 1:num_drones
    filename = fullfile(log_dir, sprintf('drone%d_log.txt', i));
    if isfile(filename)
        data = readtable(filename, 'VariableNamingRule', 'preserve');
        
        t = data{:, "time"} - data{1, "time"};
        dt = [0; diff(t)]; 
        
        e_x = data{:, "a_pos.x"} - data{:, "r_pos.x"};
        e_y = data{:, "a_pos.y"} - data{:, "r_pos.y"};
        e_z = data{:, "a_pos.z"} - data{:, "r_pos.z"};
        
        sq_error = e_x.^2 + e_y.^2 + e_z.^2;
        rmse_3d = sqrt(mean(sq_error));
        ise_3d = sum(sq_error .* dt); 
        
        fprintf('Q %d -> RMSE (Ortalama): %.5f m | ISE (Toplam): %.5f\n', i, rmse_3d, ise_3d);
    else
        fprintf('Q %d: Dosya bulunamadı.\n', i);
    end
end
fprintf('=======================================================\n');
fprintf('Grafikler EPS formatında "EPS_Results" klasörüne kaydedildi.\n');

%% === 9. HER DRONE İÇİN AYRI ANLIK HATA (SEYRELTİLMİŞ İNCE ÇUBUKLAR) ===
%fig = figure;
%set(fig, 'Position', [100, 100, 1200, 800]); % Büyük pencere
%
%% --- KULLANICI AYARLARI ---
%% 1) SEYREKLİK ORANI:
%% Veriyi ne kadar seyreltmek istediğinizi belirler.
%% Örneğin '50' ise, log dosyasındaki her 50 veriden sadece 1'i çizilir.
%% Çubuklar hala çok sıksa bu sayıyı artırın (örn: 100).
%sparse_factor = 300; 
%
%% 2) ÇUBUK İNCELİĞİ:
%% 0 ile 1 arasında bir değer. 
%% 1 = Çubuklar birbirine değer (kalın). 
%% 0.3 - 0.5 arası = İnce ve aralıklı çubuklar.
%bar_thinness = 1.0; 
%% ---------------------------
%
%fixed_ylim = 'auto'; 
%
%for i = 1:num_drones
%    % 3 satır 2 sütunlu yerleşim
%    subplot(3, 2, i); 
%    hold on;
%    
%    filename = fullfile(log_dir, sprintf('drone%d_log.txt', i));
%    if isfile(filename)
%        data = readtable(filename, 'VariableNamingRule', 'preserve');
%        time = data{:, "time"} - data{1, "time"};
%        
%        % Hata Hesaplama
%        e_x = data{:, "a_pos.x"} - data{:, "r_pos.x"};
%        e_y = data{:, "a_pos.y"} - data{:, "r_pos.y"};
%        e_z = data{:, "a_pos.z"} - data{:, "r_pos.z"};
%        pos_error = sqrt(e_x.^2 + e_y.^2 + e_z.^2);
%        
%        % --- VERİ SEYRELTME İŞLEMİ ---
%        num_points = length(time);
%        % 1'den başlayıp, sparse_factor kadar atlayarak indeksleri seç
%        idx_sparse = 1:sparse_factor:num_points;
%        
%        time_sparse = time(idx_sparse);
%        error_sparse = pos_error(idx_sparse);
%        
%        % --- ÇİZİM: SEYRELTİLMİŞ VE İNCE ÇUBUKLAR ---
%        % 'BarWidth': Çubuk inceliği ayarı.
%        % 'EdgeColor', 'k': Çubuklar artık ayrık olduğu için siyah kenar çizgisi
%        % eklemek görselliği artırır. İstemezseniz 'none' yapabilirsiniz.
%        h_bar = bar(time_sparse, error_sparse, 'FaceColor', colors(i,:), ...
%                    'EdgeColor', 'k', 'LineWidth', 0.5, 'BarWidth', bar_thinness);
%        
%        % Y Limiti Ayarları
%        yl = ylim; 
%        if ~strcmp(fixed_ylim, 'auto')
%             yl = [0 fixed_ylim];
%        else
%             max_err = max(pos_error); if max_err == 0, max_err = 0.1; end
%             yl = [0, max_err * 1.1]; 
%        end
%        ylim(yl);
%        
%        % Zonları ekle (Arka plan renkleri)
%        p1 = patch(t_zone1, [yl(1) yl(1) yl(2) yl(2)], c_zone1, 'EdgeColor', 'none', 'HandleVisibility', 'off');
%        p2 = patch(t_zone2, [yl(1) yl(1) yl(2) yl(2)], c_zone2, 'EdgeColor', 'none', 'HandleVisibility', 'off');
%        p3 = patch(t_zone3, [yl(1) yl(1) yl(2) yl(2)], c_zone3, 'EdgeColor', 'none', 'HandleVisibility', 'off');
%        p4 = patch(t_zone4, [yl(1) yl(1) yl(2) yl(2)], c_zone4, 'EdgeColor', 'none', 'HandleVisibility', 'off');
%        uistack([p1 p2 p3 p4], 'bottom'); % Zonları arkaya at
%        
%        % Etiketler ve Başlıklar
%        % Başlıkta ne kadar seyreltildiğini bilgi olarak ekleyelim
%        title(sprintf('Q %d Error', i), ...
%            'FontWeight', 'bold', 'FontSize', 20, 'FontName', 'Times New Roman');
%        ylabel('||e_p|| (m)', 'FontWeight', 'bold', 'FontSize', 20, 'FontName', 'Times New Roman');
%        grid off; 
%        xlim([0 60]);
%        
%        % Sadece alt sıradakilere X ekseni etiketi
%        if i >= 4
%            xlabel('t (s)', 'FontWeight', 'bold', 'FontSize', 20, 'FontName', 'Times New Roman');
%        end
%        
%        set(gca, 'FontName', 'Times New Roman', 'FontSize', 20, 'FontWeight', 'bold', 'LineWidth', 1.5, 'Box', 'on', 'Layer', 'top');
%    else
%        text(0.5, 0.5, 'No Data', 'HorizontalAlignment', 'center');
%    end
%end
%
%% KAYDETME
%eps_name = fullfile(eps_folder, '09_individual_error_sparse_bar.eps');
%exportgraphics(fig, eps_name, 'ContentType', 'vector');
%exportgraphics(fig, output_pdf, 'Append', true);
%close(fig);
%fprintf('Her drone için seyreltilmiş anlık hata çubuk grafikleri oluşturuldu ve kaydedildi.\n');
%
%
%%% === 10. KARŞILAŞTIRMALI HATA ANALİZİ (MEVCUT vs GEOMETRIC) - TIGHT LAYOUT ===
%fig = figure;
%set(fig, 'Position', [100, 100, 1200, 800]); % Pencere boyutu
%
%% --- DİZİN AYARLARI ---
%log_dir1 = log_dir; 
%log_dir2 = '/home/asd/catkin_ws/src/multi_quadcopter_geometric_control/log/';
%
%% --- GÖRSEL VE YERLEŞİM AYARLARI (TIGHT LAYOUT) ---
%sparse_factor = 300;     
%bar_width_1   = 1.0;     
%bar_width_2   = 0.5;     
%fixed_ylim    = 'auto';  
%
%% Sayfa Yerleşim Parametreleri (0 ile 1 arası oranlar)
%marg_l = 0.08; % Sol kenar boşluğu (Y eksen yazısı için)
%marg_r = 0.02; % Sağ kenar boşluğu
%marg_b = 0.08; % Alt kenar boşluğu (X eksen yazısı için)
%marg_t = 0.05; % Üst kenar boşluğu
%gap_w  = 0.03; % Grafikler arası yatay boşluk (Daraltıldı)
%gap_h  = 0.06; % Grafikler arası dikey boşluk (Daraltıldı)
%
%% Genişlik ve Yükseklik Hesaplaması (Otomatik)
%plot_w = (1 - marg_l - marg_r - gap_w) / 2;      % 2 Sütun olduğu için
%plot_h = (1 - marg_b - marg_t - 2 * gap_h) / 3;  % 3 Satır olduğu için
%
%for i = 1:num_drones
%    % --- KONUM HESAPLAMA (Grid Mantığı) ---
%    row = ceil(i / 2);          % Satır indeksi (1, 1, 2, 2, 3, 3)
%    col = mod(i-1, 2) + 1;      % Sütun indeksi (1, 2, 1, 2, 1, 2)
%    
%    % MATLAB koordinat sistemi sol-alttan başlar, buna göre pozisyon:
%    pos_x = marg_l + (col - 1) * (plot_w + gap_w);
%    % Y ekseni yukarıdan aşağıya insin diye ters mantık kurulur:
%    pos_y = 1 - marg_t - row * plot_h - (row - 1) * gap_h;
%    
%    % Subplot yerine direkt eksen oluşturuyoruz
%    ax = axes('Position', [pos_x, pos_y, plot_w, plot_h]);
%    hold on; box on;
%    
%    local_handles = [];
%    local_names = {};
%    
%    % === 1. VERİ SETİ (PROPOSED) ===
%    file1 = fullfile(log_dir1, sprintf('drone%d_log.txt', i));
%    max_err1 = 0;
%    if isfile(file1)
%        data1 = readtable(file1, 'VariableNamingRule', 'preserve');
%        t1 = data1{:, "time"} - data1{1, "time"};
%        ex1 = data1{:, "a_pos.x"} - data1{:, "r_pos.x"};
%        ey1 = data1{:, "a_pos.y"} - data1{:, "r_pos.y"};
%        ez1 = data1{:, "a_pos.z"} - data1{:, "r_pos.z"};
%        err1 = sqrt(ex1.^2 + ey1.^2 + ez1.^2);
%        max_err1 = max(err1);
%        idx1 = 1:sparse_factor:length(t1);
%        h1 = bar(t1(idx1), err1(idx1), 'FaceColor', colors(i,:), ...
%            'EdgeColor', 'none', 'BarWidth', bar_width_1, 'FaceAlpha', 0.6);
%        local_handles = [local_handles, h1];
%        local_names{end+1} = 'Proposed Controller';
%    end
%    
%    % === 2. VERİ SETİ (SE3) ===
%    file2 = fullfile(log_dir2, sprintf('drone%d_log.txt', i));
%    max_err2 = 0;
%    if isfile(file2)
%        data2 = readtable(file2, 'VariableNamingRule', 'preserve');
%        t2 = data2{:, "time"} - data2{1, "time"};
%        ex2 = data2{:, "a_pos.x"} - data2{:, "r_pos.x"};
%        ey2 = data2{:, "a_pos.y"} - data2{:, "r_pos.y"};
%        ez2 = data2{:, "a_pos.z"} - data2{:, "r_pos.z"};
%        err2 = sqrt(ex2.^2 + ey2.^2 + ez2.^2);
%        max_err2 = max(err2);
%        idx2 = 1:sparse_factor:length(t2);
%        h2 = bar(t2(idx2), err2(idx2), 'FaceColor', [0.2 0.2 0.2], ...
%            'EdgeColor', 'none', 'BarWidth', bar_width_2, 'FaceAlpha', 0.9);
%        local_handles = [local_handles, h2];
%        local_names{end+1} = 'SE(3) Controller';
%    end
%    
%    % === EKSEN LİMİTLERİ VE ZONLAR ===
%    if ~strcmp(fixed_ylim, 'auto')
%         yl = [0 fixed_ylim];
%    else
%         global_max = max(max_err1, max_err2);
%         if global_max == 0, global_max = 0.1; end
%         yl = [0, global_max * 1.1]; 
%    end
%    ylim(yl);
%    
%    p1 = patch(t_zone1, [yl(1) yl(1) yl(2) yl(2)], c_zone1, 'EdgeColor', 'none', 'HandleVisibility', 'off');
%    p2 = patch(t_zone2, [yl(1) yl(1) yl(2) yl(2)], c_zone2, 'EdgeColor', 'none', 'HandleVisibility', 'off');
%    p3 = patch(t_zone3, [yl(1) yl(1) yl(2) yl(2)], c_zone3, 'EdgeColor', 'none', 'HandleVisibility', 'off');
%    p4 = patch(t_zone4, [yl(1) yl(1) yl(2) yl(2)], c_zone4, 'EdgeColor', 'none', 'HandleVisibility', 'off');
%    uistack([p1 p2 p3 p4], 'bottom'); 
%    
%    grid off; xlim([0 60]);
%    
%    % Eksen yazı tipleri
%    set(gca, 'XTick', 0:10:60, ...          
%             'FontName', 'Times New Roman', ...
%             'FontSize', 16, ... % Font biraz küçültüldü ki sığsın
%             'FontWeight', 'bold', ...
%             'LineWidth', 1.5, ...
%             'Layer', 'top');
%    
%    % === LEGEND ===
%    if ~isempty(local_handles)
%        lgd = legend(local_handles, local_names, ...
%            'Location', 'NorthEast', ...
%            'FontSize', 13, ...            
%            'FontWeight', 'bold', ...
%            'FontName', 'Times New Roman', ...
%            'Box', 'on');
%        title(lgd, sprintf('Q %d', i)); 
%        lgd.Title.Visible = 'on';
%        lgd.Title.NodeChildren.FontSize = 10;
%        lgd.Title.NodeChildren.FontWeight = 'bold';
%    end
%end
%
%% --- ORTAK EKSEN YAZILARI ---
%% Tüm figürü kaplayan görünmez bir eksen
%han = axes(fig, 'Position', [0 0 1 1], 'visible', 'off'); 
%
%% Y-Ekseni Yazısı (Sol Kenar Ortası)
%text(han, 0.03, 0.5, '||e_p|| (m)', ...
%    'FontName', 'Times New Roman', ...
%    'FontSize', 22, ...
%    'FontWeight', 'bold', ...
%    'Rotation', 90, ...
%    'HorizontalAlignment', 'center', ...
%    'VerticalAlignment', 'middle');
%
%% X-Ekseni Yazısı (Alt Kenar Ortası)
%text(han, 0.5, 0.03, 't (s)', ...
%    'FontName', 'Times New Roman', ...
%    'FontSize', 22, ...
%    'FontWeight', 'bold', ...
%    'HorizontalAlignment', 'center', ...
%    'VerticalAlignment', 'middle');
%
%% KAYDETME
%eps_name = fullfile(eps_folder, 'Fig12_Tight.eps');
%exportgraphics(fig, eps_name, 'ContentType', 'vector');
%exportgraphics(fig, output_pdf, 'Append', true);
%close(fig);
%fprintf('Grafikler sıkıştırılmış (Tight Layout) formatta oluşturuldu.\n');
%%% === 11. KARŞILAŞTIRMALI HATA ANALİZİ (SADECE DRONE 2 ve 3) - DÜZELTİLMİŞ LAYOUT ===
%fig = figure;
%set(fig, 'Position', [100, 100, 1200, 500]); % Geniş ve alçak pencere
%
%% --- DİZİN AYARLARI ---
%log_dir1 = log_dir; 
%log_dir2 = '/home/asd/catkin_ws/src/multi_quadcopter_geometric_control/log/';
%
%% --- GÖRSEL VE YERLEŞİM AYARLARI ---
%sparse_factor = 300;     
%bar_width_1   = 1.0;     
%bar_width_2   = 0.5;     
%fixed_ylim    = 0.24;    
%
%% --- SAYFA YERLEŞİMİ (DÜZELTİLEN KISIM) ---
%% Sol eksen sayıları ve yazısı için daha fazla yer ayrıldı.
%marg_l = 0.10;  % Sol kenar boşluğu (ARTIRILDI: 0.07 -> 0.12)
%marg_r = 0.02;  % Sağ kenar boşluğu
%marg_b = 0.15;  % Alt kenar boşluğu
%marg_t = 0.10;  % Üst kenar boşluğu (Legend sığsın diye biraz artırıldı)
%
%% Grafikler arası boşluk (Sağdaki sayıların sola binmemesi için)
%gap    = 0.08;  % Ara boşluk (ARTIRILDI: 0.03 -> 0.08)
%
%% Grafik genişlik hesaplaması (Otomatik)
%plot_w = (1 - marg_l - marg_r - gap) / 2; 
%plot_h = 1 - marg_b - marg_t;             
%
%target_drones = [2, 3]; 
%
%for k = 1:length(target_drones)
%    i = target_drones(k); 
%    
%    % --- KONUM HESAPLAMA ---
%    pos_x = marg_l + (k-1) * (plot_w + gap);
%    pos_y = marg_b; 
%    
%    ax = axes('Position', [pos_x, pos_y, plot_w, plot_h]);
%    hold on; box on;
%    
%    local_handles = [];
%    local_names = {};
%    
%    % === 1. VERİ SETİ (PROPOSED) ===
%    file1 = fullfile(log_dir1, sprintf('drone%d_log.txt', i));
%    max_err1 = 0;
%    if isfile(file1)
%        data1 = readtable(file1, 'VariableNamingRule', 'preserve');
%        t1 = data1{:, "time"} - data1{1, "time"};
%        ex1 = data1{:, "a_pos.x"} - data1{:, "r_pos.x"};
%        ey1 = data1{:, "a_pos.y"} - data1{:, "r_pos.y"};
%        ez1 = data1{:, "a_pos.z"} - data1{:, "r_pos.z"};
%        err1 = sqrt(ex1.^2 + ey1.^2 + ez1.^2);
%        max_err1 = max(err1);
%        idx1 = 1:sparse_factor:length(t1);
%        h1 = bar(t1(idx1), err1(idx1), 'FaceColor', colors(i,:), ...
%            'EdgeColor', 'none', 'BarWidth', bar_width_1, 'FaceAlpha', 0.6);
%        local_handles = [local_handles, h1];
%        local_names{end+1} = 'Feedforward Inactive';
%    end
%    
%    % === 2. VERİ SETİ (SE3) ===
%    file2 = fullfile(log_dir2, sprintf('drone%d_log.txt', i));
%    max_err2 = 0;
%    if isfile(file2)
%        data2 = readtable(file2, 'VariableNamingRule', 'preserve');
%        t2 = data2{:, "time"} - data2{1, "time"};
%        ex2 = data2{:, "a_pos.x"} - data2{:, "r_pos.x"};
%        ey2 = data2{:, "a_pos.y"} - data2{:, "r_pos.y"};
%        ez2 = data2{:, "a_pos.z"} - data2{:, "r_pos.z"};
%        err2 = sqrt(ex2.^2 + ey2.^2 + ez2.^2);
%        max_err2 = max(err2);
%        idx2 = 1:sparse_factor:length(t2);
%        h2 = bar(t2(idx2), err2(idx2), 'FaceColor', [0.2 0.2 0.2], ...
%            'EdgeColor', 'none', 'BarWidth', bar_width_2, 'FaceAlpha', 0.9);
%        local_handles = [local_handles, h2];
%        local_names{end+1} = 'Feedforward Active';
%    end
%    
%    % === EKSEN LİMİTLERİ VE ZONLAR ===
%    if ~strcmp(fixed_ylim, 'auto')
%         yl = [0 fixed_ylim];
%    else
%         global_max = max(max_err1, max_err2);
%         if global_max == 0, global_max = 0.1; end
%         yl = [0, global_max * 1.1]; 
%    end
%    ylim(yl);
%    
%    p1 = patch(t_zone1, [yl(1) yl(1) yl(2) yl(2)], c_zone1, 'EdgeColor', 'none', 'HandleVisibility', 'off');
%    p2 = patch(t_zone2, [yl(1) yl(1) yl(2) yl(2)], c_zone2, 'EdgeColor', 'none', 'HandleVisibility', 'off');
%    p3 = patch(t_zone3, [yl(1) yl(1) yl(2) yl(2)], c_zone3, 'EdgeColor', 'none', 'HandleVisibility', 'off');
%    p4 = patch(t_zone4, [yl(1) yl(1) yl(2) yl(2)], c_zone4, 'EdgeColor', 'none', 'HandleVisibility', 'off');
%    uistack([p1 p2 p3 p4], 'bottom'); 
%    
%    grid off; xlim([0 60]);
%    
%    set(gca, 'XTick', 0:10:60, ...
%             'FontName', 'Times New Roman', ...
%             'FontSize', 20, ...
%             'FontWeight', 'bold', ...
%             'LineWidth', 1.5, ...
%             'Layer', 'top');
%    
%    % === LEGEND ===
%    if ~isempty(local_handles)
%        lgd = legend(local_handles, local_names, ...
%            'Location', 'NorthEast', ...
%            'FontSize', 13, ...            
%            'FontWeight', 'bold', ...
%            'FontName', 'Times New Roman', ...
%            'Box', 'on');
%            
%        title(lgd, sprintf('Q %d', i)); 
%        lgd.Title.Visible = 'on';
%        lgd.Title.NodeChildren.FontSize = 12;
%        lgd.Title.NodeChildren.FontWeight = 'bold';
%    end
%end
%
%% --- ORTAK EKSEN YAZILARI (Global Labels) ---
%han = axes(fig, 'Position', [0 0 1 1], 'visible', 'off'); 
%
%% Y-Ekseni Yazısı (Daha sola çekildi: 0.02)
%text(han, 0.02, 0.55, '||e_p|| (m)', ...
%    'FontName', 'Times New Roman', ...
%    'FontSize', 24, ...
%    'FontWeight', 'bold', ...
%    'Rotation', 90, ...
%    'HorizontalAlignment', 'center', ...
%    'VerticalAlignment', 'middle');
%
%% X-Ekseni Yazısı
%text(han, 0.5, 0.04, 't (s)', ...
%    'FontName', 'Times New Roman', ...
%    'FontSize', 24, ...
%    'FontWeight', 'bold', ...
%    'HorizontalAlignment', 'center', ...
%    'VerticalAlignment', 'middle');
%
%% KAYDETME
%eps_name = fullfile(eps_folder, 'Fig12_Drone2and3_Corrected.eps');
%exportgraphics(fig, eps_name, 'ContentType', 'vector');
%exportgraphics(fig, output_pdf, 'Append', true);
%close(fig);
%fprintf('Eksen çakışmaları giderildi.\n');
%
%%% === 12. HER DRONE İÇİN EULER AÇI HATASI (ATTITUDE ERROR - BAR GRAFİĞİ) ===
%fig = figure;
%set(fig, 'Position', [100, 100, 1200, 800]); % Büyük pencere
%
%% --- GÖRSEL AYARLAR ---
%sparse_factor = 300;     % Veri seyreltme (Bar sıklığı)
%bar_thinness  = 0.3;     % Bar kalınlığı
%fixed_ylim    = 'auto';  % Y ekseni limiti ('auto' veya örn: 0.1 rad)
%
%for i = 1:num_drones
%    % 3 satır 2 sütunlu yerleşim
%    subplot(3, 2, i); 
%    hold on; box on;
%    
%    filename = fullfile(log_dir, sprintf('drone%d_log.txt', i));
%    if isfile(filename)
%        data = readtable(filename, 'VariableNamingRule', 'preserve');
%        time = data{:, "time"} - data{1, "time"};
%        
%        % --- EULER AÇI HATALARINI HESAPLA (Radyan) ---
%        % x: Roll (Phi), y: Pitch (Theta), z: Yaw (Psi)
%        e_phi   = data{:, "a_eul_ang.x"} - data{:, "r_eul_ang.x"};
%        e_theta = data{:, "a_eul_ang.y"} - data{:, "r_eul_ang.y"};
%        e_psi   = data{:, "a_eul_ang.z"} - data{:, "r_eul_ang.z"};
%        
%        % Toplam Açı Hatası Normu (Euclidean Norm)
%        att_error = sqrt(e_phi.^2 + e_theta.^2 + e_psi.^2);
%        
%        % --- VERİ SEYRELTME ---
%        idx_sparse = 1:sparse_factor:length(time);
%        time_sparse = time(idx_sparse);
%        error_sparse = att_error(idx_sparse);
%        
%        % --- ÇİZİM ---
%        bar(time_sparse, error_sparse, 'FaceColor', colors(i,:), ...
%            'EdgeColor', 'k', 'LineWidth', 0.5, 'BarWidth', bar_thinness);
%        
%        % Y Limiti Ayarları
%        yl = ylim; 
%        if ~strcmp(fixed_ylim, 'auto')
%             yl = [0 fixed_ylim];
%        else
%             max_err = max(att_error); if max_err == 0, max_err = 0.1; end
%             yl = [0, max_err * 1.1]; 
%        end
%        ylim(yl);
%        
%        % Arka Plan Zonları
%        p1 = patch(t_zone1, [yl(1) yl(1) yl(2) yl(2)], c_zone1, 'EdgeColor', 'none', 'HandleVisibility', 'off');
%        p2 = patch(t_zone2, [yl(1) yl(1) yl(2) yl(2)], c_zone2, 'EdgeColor', 'none', 'HandleVisibility', 'off');
%        p3 = patch(t_zone3, [yl(1) yl(1) yl(2) yl(2)], c_zone3, 'EdgeColor', 'none', 'HandleVisibility', 'off');
%        p4 = patch(t_zone4, [yl(1) yl(1) yl(2) yl(2)], c_zone4, 'EdgeColor', 'none', 'HandleVisibility', 'off');
%        uistack([p1 p2 p3 p4], 'bottom'); 
%        
%        % Etiketler
%        title(sprintf('Q %d Attitude Error', i), 'FontWeight', 'bold', 'FontSize', 20, 'FontName', 'Times New Roman');
%        ylabel('||e_{att}|| (rad)', 'FontWeight', 'bold', 'FontSize', 20, 'FontName', 'Times New Roman');
%        grid off; xlim([0 60]);
%        
%        % Sadece alt sıradakilere X ekseni etiketi
%        if i >= 4
%            xlabel('t (s)', 'FontWeight', 'bold', 'FontSize', 20, 'FontName', 'Times New Roman');
%        end
%        
%        % X Eksenini 10'un katları yap
%        set(gca, 'XTick', 0:10:60, ...
%                 'FontName', 'Times New Roman', ...
%                 'FontSize', 20, ...
%                 'FontWeight', 'bold', ...
%                 'LineWidth', 1.5, ...
%                 'Layer', 'top');
%    else
%        text(0.5, 0.5, 'No Data', 'HorizontalAlignment', 'center');
%    end
%end
%
%% KAYDETME
%eps_name = fullfile(eps_folder, '12_attitude_error_bar.eps');
%exportgraphics(fig, eps_name, 'ContentType', 'vector');
%exportgraphics(fig, output_pdf, 'Append', true);
%close(fig);
%fprintf('Euler açısı hata grafikleri (Attitude Error) oluşturuldu.\n');
%
%%% === 15. DCM (ROTASYON MATRİSİ) TABANLI YÖNELİM HATASI (GEODESIC ERROR) ===
%fig = figure;
%set(fig, 'Position', [100, 100, 800, 500]); % Tek grafik
%
%% --- AYARLAR ---
%target_drone  = 2;      % Sadece Drone 2
%sparse_factor = 300;    % Veri seyreltme
%bar_width_1   = 0.5;
%bar_width_2   = 0.3;
%fixed_ylim    = 1.3;    % <--- DEĞİŞİKLİK BURADA: Y Limiti 1.3 radyan
%
%% Dosya Yolları
%log_dirs = {log_dir, '/home/asd/catkin_ws/src/multi_quadcopter_geometric_control/log/'};
%legend_names = {'\beta = 0.05', '\beta = 0.1'};
%bar_colors = {colors(target_drone,:), [0.2 0.2 0.2]};
%bar_widths = [bar_width_1, bar_width_2];
%alphas = [0.6, 0.9];
%
%hold on; box on;
%local_handles = [];
%
%for k = 1:2
%    filename = fullfile(log_dirs{k}, sprintf('drone%d_log.txt', target_drone));
%    if ~isfile(filename), continue; end
%    
%    data = readtable(filename, 'VariableNamingRule', 'preserve');
%    time = data{:, "time"} - data{1, "time"};
%    
%    % Euler Açılarını Al (Radyan)
%    r_phi = data{:, "r_eul_ang.x"}; r_theta = data{:, "r_eul_ang.y"}; r_psi = data{:, "r_eul_ang.z"};
%    a_phi = data{:, "a_eul_ang.x"}; a_theta = data{:, "a_eul_ang.y"}; a_psi = data{:, "a_eul_ang.z"};
%    
%    num_points = length(time);
%    dcm_error = zeros(num_points, 1);
%    
%    % --- DCM HESAPLAMA DÖNGÜSÜ (Z-Y-X) ---
%    for j = 1:num_points
%        % 1. Referans Rotasyon Matrisi (R_ref)
%        % Z-Y-X: R = Rz(psi) * Ry(theta) * Rx(phi)
%        
%        Rx_r = [1 0 0; 0 cos(r_phi(j)) -sin(r_phi(j)); 0 sin(r_phi(j)) cos(r_phi(j))];
%        Ry_r = [cos(r_theta(j)) 0 sin(r_theta(j)); 0 1 0; -sin(r_theta(j)) 0 cos(r_theta(j))];
%        Rz_r = [cos(r_psi(j)) -sin(r_psi(j)) 0; sin(r_psi(j)) cos(r_psi(j)) 0; 0 0 1];
%        
%        R_ref = Rz_r * Ry_r * Rx_r;
%        
%        % 2. Aktüel Rotasyon Matrisi (R_act)
%        Rx_a = [1 0 0; 0 cos(a_phi(j)) -sin(a_phi(j)); 0 sin(a_phi(j)) cos(a_phi(j))];
%        Ry_a = [cos(a_theta(j)) 0 sin(a_theta(j)); 0 1 0; -sin(a_theta(j)) 0 cos(a_theta(j))];
%        Rz_a = [cos(a_psi(j)) -sin(a_psi(j)) 0; sin(a_psi(j)) cos(a_psi(j)) 0; 0 0 1];
%        
%        R_act = Rz_a * Ry_a * Rx_a;
%        
%        % 3. Hata Matrisi: R_err = R_ref' * R_act
%        R_err = R_ref' * R_act;
%        
%        % 4. Geodesic Error (Trace Hesabı)
%        tr = R_err(1,1) + R_err(2,2) + R_err(3,3);
%        val = (tr - 1) / 2;
%        val = max(min(val, 1), -1); % Güvenlik
%        
%        dcm_error(j) = acos(val); % Radyan
%    end
%    
%    % --- ÇİZİM ---
%    idx = 1:sparse_factor:num_points;
%    h = bar(time(idx), dcm_error(idx), 'FaceColor', bar_colors{k}, ...
%            'EdgeColor', 'none', 'BarWidth', bar_widths(k), 'FaceAlpha', alphas(k));
%    local_handles = [local_handles, h];
%end
%
%% --- GÖRSEL DÜZENLEMELER ---
%% Y Limiti Sabitleme
%if ~strcmp(fixed_ylim, 'auto')
%     yl = [0 fixed_ylim];
%else
%     yl = ylim; yl = [0, yl(2)*1.1]; 
%end
%ylim(yl);
%
%% Arka Plan Zonları
%p1 = patch(t_zone1, [yl(1) yl(1) yl(2) yl(2)], c_zone1, 'EdgeColor', 'none', 'HandleVisibility', 'off');
%p2 = patch(t_zone2, [yl(1) yl(1) yl(2) yl(2)], c_zone2, 'EdgeColor', 'none', 'HandleVisibility', 'off');
%p3 = patch(t_zone3, [yl(1) yl(1) yl(2) yl(2)], c_zone3, 'EdgeColor', 'none', 'HandleVisibility', 'off');
%p4 = patch(t_zone4, [yl(1) yl(1) yl(2) yl(2)], c_zone4, 'EdgeColor', 'none', 'HandleVisibility', 'off');
%uistack([p1 p2 p3 p4], 'bottom'); 
%
%% Eksenler
%ylabel('Orientation Error (rad)', 'FontWeight', 'bold', 'FontSize', 16, 'FontName', 'Times New Roman');
%xlabel('t (s)', 'FontWeight', 'bold', 'FontSize', 16, 'FontName', 'Times New Roman');
%xlim([0 60]);
%
%set(gca, 'XTick', 0:10:60, ...
%         'FontName', 'Times New Roman', 'FontSize', 16, 'FontWeight', 'bold', 'LineWidth', 1.5, 'Layer', 'top');
%
%% Legend
%lgd = legend(local_handles, legend_names, 'Location', 'NorthEast', ...
%    'FontSize', 12, 'FontWeight', 'bold', 'Box', 'on');
%
%% KAYDETME
%eps_name = fullfile(eps_folder, '15_drone2_DCM_error.eps');
%exportgraphics(fig, eps_name, 'ContentType', 'vector');
%exportgraphics(fig, output_pdf, 'Append', true);
%close(fig);
%fprintf('Drone 2 için DCM tabanlı oryantasyon hata grafiği (Y-Lim: 1.3) oluşturuldu.\n');
%
%%% === 16. DRONE 1 - PUBLICATION READY (TIGHT LAYOUT & STAR MARKER) ===
%fig = figure;
%% Makale için genelde genişlik yükseklikten fazla olur, oran ayarlandı
%set(fig, 'Position', [100, 100, 1200, 550]); 
%
%% 'tiledlayout' grafikleri birbirine subplot'tan çok daha fazla yaklaştırır.
%% 'TileSpacing', 'compact' -> Grafikler arası boşluğu azaltır.
%% 'Padding', 'compact' -> Figür kenar boşluklarını azaltır.
%t = tiledlayout(1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
%
%% --- 1. WAYPOINT VERİLERİ ---
%wp_sets{1}.x = [0.0, 0.0, 0.0, 2.0, 2.0, 2.0, 3.0];
%wp_sets{1}.y = [0.0, 0.5, 2.5, 4.0, 7.0, 8.5, 10.0];
%wp_sets{1}.z = [0.0, 1.5, 2.0, 2.0, 3.0, 2.0, 2.0];
%
%wp_sets{2}.x = [-1.0, -6.0, -10.0, -7.0, -4.5, -2.8, -2.0];
%wp_sets{2}.y = [3.0, 0.0, -5.2, -6.3, -7.0, -9.5, -10.0];
%wp_sets{2}.z = [2.5, 2.0, 2.2, 2.2, 2.3, 2.4, 2.7];
%
%wp_sets{3}.x = [-2.5, -1.0, 0.0, 0.0, 0.0, 0.0, 1.0];
%wp_sets{3}.y = [-9.0, -11.0, -12.5, -14.0, -16.0, -18.0, -20.0];
%wp_sets{3}.z = [2.5, 2.0, 2.0, 2.0, 3.0, 3.0, 3.0];
%
%wp_sets{4}.x = [1.0, 0.0, 0.0, -1.0, 1.0, 1.0, 0.0];
%wp_sets{4}.y = [-17.0, -14.0, -11.0, -8.0, -5.0, -2.0, 0.0];
%wp_sets{4}.z = [2.0, 3.0, 3.0, 3.0, 3.0, 2.5, 0.0];
%
%% Renkler (Grafik referansına uygun)
%wp_colors = {[1, 0.2, 0.2], [0.2, 0.6, 1], [0.2, 0.8, 0.2], [0.9, 0.8, 0]};
%
%% --- 2. LOG OKUMA ---
%filename = fullfile(log_dir, 'drone1_log.txt');
%
%if isfile(filename)
%    data = readtable(filename, 'VariableNamingRule', 'preserve');
%    act_x = data{:, "a_pos.x"};
%    act_y = data{:, "a_pos.y"};
%    act_z = data{:, "a_pos.z"};
%    d1_color = colors(1, :); 
%
%    start_x = act_x(1);
%    start_y = act_y(1);
%    start_z = act_z(1);
%
%    % ======================================================
%    % GRAFİK 1: IZOMETRIK GÖRÜNÜM (3D)
%    % ======================================================
%    nexttile; % Subplot yerine nexttile kullanılır
%    hold on; box on;
%    
%    % Gerçek Rota
%    plot3(act_x, act_y, act_z, '-', 'Color', d1_color, 'LineWidth', 2.5);
%    
%    % Waypointler (Sadece Kareler)
%    for k = 1:4
%        scatter3(wp_sets{k}.x, wp_sets{k}.y, wp_sets{k}.z, 100, wp_colors{k}, 's', 'filled', ...
%                 'MarkerEdgeColor', 'k');
%    end
%
%    % BAŞLANGIÇ NOKTASI (YILDIZ)
%    % 'p' -> Pentagram (Yıldız) sembolüdür.
%    scatter3(start_x, start_y, start_z, 300, 'g', 'p', 'filled', 'MarkerEdgeColor', 'k', 'LineWidth', 1.5);
%
%    xlabel('x (m)', 'FontWeight', 'bold', 'FontSize', 22, 'FontName', 'Times New Roman');
%    ylabel('y (m)', 'FontWeight', 'bold', 'FontSize', 22, 'FontName', 'Times New Roman');
%    zlabel('z (m)', 'FontWeight', 'bold', 'FontSize', 22, 'FontName', 'Times New Roman');
%    
%    view(45, 30);
%    grid on; 
%    axis equal; 
%    axis tight; % Grafiği kenarlara yapıştırır (Compact görüntü için şart)
%    
%    % Eksen yazı boyutlarını makale için büyüttük
%    set(gca, 'FontName', 'Times New Roman', 'FontSize', 20, 'FontWeight', 'bold', 'LineWidth', 2.0);
%
%    % ======================================================
%    % GRAFİK 2: XY GÖRÜNÜMÜ (TOP-DOWN)
%    % ======================================================
%    nexttile; 
%    hold on; box on;
%    
%    % Gerçek Rota
%    plot(act_x, act_y, '-', 'Color', d1_color, 'LineWidth', 2.5);
%    
%    % Waypointler
%    for k = 1:4
%        scatter(wp_sets{k}.x, wp_sets{k}.y, 100, wp_colors{k}, 's', 'filled', ...
%                'MarkerEdgeColor', 'k');
%    end
%
%    % BAŞLANGIÇ NOKTASI (YILDIZ)
%    scatter(start_x, start_y, 300, 'g', 'p', 'filled', 'MarkerEdgeColor', 'k', 'LineWidth', 1.5);
%
%    xlabel('x (m)', 'FontWeight', 'bold', 'FontSize', 22, 'FontName', 'Times New Roman');
%    ylabel('y (m)', 'FontWeight', 'bold', 'FontSize', 22, 'FontName', 'Times New Roman');
%    
%    view(0, 90);
%    grid on; 
%    axis equal; 
%    axis tight;
%    
%    set(gca, 'FontName', 'Times New Roman', 'FontSize', 20, 'FontWeight', 'bold', 'LineWidth', 2.0);
%
%    % --- KAYDETME ---
%    eps_name = fullfile(eps_folder, '16_drone1_article_ready.eps');
%    exportgraphics(fig, eps_name, 'ContentType', 'vector');
%    exportgraphics(fig, output_pdf, 'Append', true);
%    fprintf('Makale formatında (Sıkıştırılmış, Yıldızlı) grafik oluşturuldu.\n');
%else
%    fprintf('HATA: Drone 1 log dosyası (%s) bulunamadı.\n', filename);
%end
%close(fig);
%
%