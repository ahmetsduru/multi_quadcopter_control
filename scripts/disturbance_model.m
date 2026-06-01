clc; clear; close all;

%% 1. Model Parametreleri

% --- KULLANICI GİRİŞLERİ ---
m_drone = 0.382;    
g = 9.81;           

% --- FİZİKSEL KATSAYILAR ---
C_coup = 0.075;     
r_lat = 0.05;      
rho_air = 1.225;    

% --- HESAPLAMALAR ---
Thrust_per_rotor = m_drone * g / 4; 

K_z = Thrust_per_rotor * C_coup;  
K_x = K_z * r_lat;            
K_y = K_z * r_lat;            

K_matrix = diag([K_x, K_y, K_z]); 

% --- YENİ MODEL PARAMETRELERİ (NON-LİNEER BİRLEŞME) ---
z0 = 0.05;       
sigma = 0.2;  
turb_intensity = 0.80; % Pink Noise şiddet katsayısı

% İki üst drone arasındaki mesafe
D_drones = 0.6; % metre
Pos_Top1 = [-D_drones/2, 0, 0]; % 1. Drone Konumu
Pos_Top2 = [ D_drones/2, 0, 0]; % 2. Drone Konumu

% Jet birleşme bölgesi (Merging Zone) parametreleri
alpha_rad = deg2rad(12); % Havanın yayılma açısı
z_merge = - (D_drones / 2) / tan(alpha_rad); % Akışların kesiştiği dikey derinlik
p_mid = [0, 0, z_merge]; % Geometrik etkileşim merkezi

C_turb = 8.0;      % Çapraz türbülans büyütme katsayısı (Gamma için)
C_coal = 0.6;      % Ortalama akış birleşme katsayısı (Lambda için)
sigma_int = 0.4;   % Etkileşim bölgesinin genişliği

%% 2. Izgara (Grid) Oluşturma
x_range = -1.5 : 0.02 : 1.5;  
z_range = -2.5 : 0.02 : -0.1;
[X, Z] = meshgrid(x_range, z_range);

%% 3. Pink Noise Alanlarının Oluşturulması
[rows, cols] = size(X);
Pink_Map_X = create_pink_noise_field(rows, cols);
Pink_Map_Y = create_pink_noise_field(rows, cols);
Pink_Map_Z = create_pink_noise_field(rows, cols);

%% 4. Hesaplama Döngüsü
F_mag = zeros(size(X));
V_mag = zeros(size(X)); 

for i = 1:numel(X)
    p_b = [X(i); 0; Z(i)]; 
    
    % --- DRONE 1 ETKİSİ ---
    p_bt1 = Pos_Top1' - p_b; 
    dist1 = norm(p_bt1);
    if dist1 > 0.01
        n_vec1 = p_bt1 / dist1; 
        decay_factor1 = (1 / (abs(p_bt1(3))^2 + z0)) * exp(-(norm([p_bt1(1); p_bt1(2)])^2) / (2*sigma^2));
        f_stat1 = - K_matrix * n_vec1 * decay_factor1;
    else
        f_stat1 = zeros(3,1);
    end
    
    % --- DRONE 2 ETKİSİ ---
    p_bt2 = Pos_Top2' - p_b; 
    dist2 = norm(p_bt2);
    if dist2 > 0.01
        n_vec2 = p_bt2 / dist2; 
        decay_factor2 = (1 / (abs(p_bt2(3))^2 + z0)) * exp(-(norm([p_bt2(1); p_bt2(2)])^2) / (2*sigma^2));
        f_stat2 = - K_matrix * n_vec2 * decay_factor2;
    else
        f_stat2 = zeros(3,1);
    end
    
    % --- PINK NOISE VE TÜRBÜLANS ---
    [r, c] = ind2sub(size(X), i);
    noise_vector = [Pink_Map_X(r,c); Pink_Map_Y(r,c); Pink_Map_Z(r,c)];
    
    f_turb1 = abs(f_stat1) .* (turb_intensity * noise_vector);
    f_turb2 = abs(f_stat2) .* (turb_intensity * noise_vector);
    
    % =================================================================
    % YENİ MODEL: NON-LİNEER SÜPERPOZİSYON (Denklem 13 ve 14)
    % =================================================================
    dist_to_mid = norm(p_b - p_mid');
    
    % Gamma: Türbülans Büyütme Skaleri (Kesişim bölgesinde kuadratik artış)
    Gamma = sqrt(1 + C_turb * exp(-(dist_to_mid^2) / (2 * sigma_int^2)));
    
    % Lambda: Dinamik Birleşme Çarpanı (Ortalama akışın birleşmesi)
    Lambda = 1 + C_coal * exp(-(dist_to_mid^2) / (2 * sigma_int^2));
    
    % Toplam Kuvvet: Sadece toplamak (f1 + f2) yerine Gamma ve Lambda ile modüle edilir
    f_total = 4 * (Lambda * (f_stat1 + f_stat2) + Gamma * (f_turb1 + f_turb2));
    % =================================================================
    
    F_mag(i) = norm(f_total);
    scaling_factor = rho_air * C_coup * 4 + eps; 
    V_mag(i) = sqrt(F_mag(i) / scaling_factor);
end

%% 5. Görselleştirme (Times New Roman - Makale Formatı)

figure('Name', 'Non-Linear Jet Merging & Turbulence Model', 'Color', 'w', 'Position', [100, 100, 1200, 600]);
t = tiledlayout(1, 2, 'TileSpacing', 'compact', 'Padding', 'compact'); 

% --- SOL GRAFİK: KUVVET PROFİLİ ---
nexttile; 
contourf(X, Z, F_mag, 100, 'LineStyle', 'none');
hold on;
plot(Pos_Top1(1), Pos_Top1(3), 'kp', 'MarkerSize', 14, 'MarkerFaceColor', 'r'); 
plot(Pos_Top2(1), Pos_Top2(3), 'kp', 'MarkerSize', 14, 'MarkerFaceColor', 'r'); 
text(Pos_Top1(1)-0.15, Pos_Top1(3)+0.15, 'UAV 1', 'Color', 'k', 'FontWeight', 'bold', 'FontSize', 12, 'FontName', 'Times New Roman');
text(Pos_Top2(1)+0.05, Pos_Top2(3)+0.15, 'UAV 2', 'Color', 'k', 'FontWeight', 'bold', 'FontSize', 12, 'FontName', 'Times New Roman');

cb1 = colorbar; 
set(cb1, 'LineWidth', 2, 'FontSize', 14, 'FontWeight', 'bold', 'FontName', 'Times New Roman');
colormap(jet); caxis([0 max(max(F_mag))]);

title({'Force Field (Non-linear Merging)', ' N'}, 'FontSize', 14, 'FontWeight', 'bold', 'FontName', 'Times New Roman');
xlabel('x (m)', 'FontSize', 14, 'FontWeight', 'bold', 'FontName', 'Times New Roman'); 
ylabel('z (m)', 'FontSize', 14, 'FontWeight', 'bold', 'FontName', 'Times New Roman');
grid off; axis equal; xlim([-1.0 1.0]); ylim([-2.5 0.5]);
set(gca, 'LineWidth', 2, 'FontSize', 14, 'FontWeight', 'bold', 'XColor', 'k', 'YColor', 'k', 'Box', 'on', 'FontName', 'Times New Roman');

% --- SAĞ GRAFİK: HIZ PROFİLİ ---
nexttile; 
contourf(X, Z, V_mag, 100, 'LineStyle', 'none');
hold on;
plot(Pos_Top1(1), Pos_Top1(3), 'kp', 'MarkerSize', 14, 'MarkerFaceColor', 'r'); 
plot(Pos_Top2(1), Pos_Top2(3), 'kp', 'MarkerSize', 14, 'MarkerFaceColor', 'r'); 
text(Pos_Top1(1)-0.15, Pos_Top1(3)+0.15, 'UAV 1', 'Color', 'k', 'FontWeight', 'bold', 'FontSize', 12, 'FontName', 'Times New Roman');
text(Pos_Top2(1)+0.05, Pos_Top2(3)+0.15, 'UAV 2', 'Color', 'k', 'FontWeight', 'bold', 'FontSize', 12, 'FontName', 'Times New Roman');

cb2 = colorbar; 
set(cb2, 'LineWidth', 2, 'FontSize', 14, 'FontWeight', 'bold', 'FontName', 'Times New Roman');
colormap(jet); caxis([0 max(max(V_mag))]);

title({'Velocity Field (Non-linear Merging)', ' m/s'}, 'FontSize', 14, 'FontWeight', 'bold', 'FontName', 'Times New Roman');
xlabel('x (m)', 'FontSize', 14, 'FontWeight', 'bold', 'FontName', 'Times New Roman'); 
ylabel('z (m)', 'FontSize', 14, 'FontWeight', 'bold', 'FontName', 'Times New Roman');
grid off; axis equal; xlim([-1.0 1.0]); ylim([-2.5 0.5]);
set(gca, 'LineWidth', 2, 'FontSize', 14, 'FontWeight', 'bold', 'XColor', 'k', 'YColor', 'k', 'Box', 'on', 'FontName', 'Times New Roman');

disp('Non-Lineer Jet Merging (Denklem 13-14) hesaplandı. İki akış merkezde birleşiyor.');

%% --- FONKSİYON: PINK NOISE ÜRETİCİSİ (1/f) ---
function pink_field = create_pink_noise_field(rows, cols)
    noise_seed = randn(rows, cols);
    fft_vals = fft2(noise_seed);
    fft_shifted = fftshift(fft_vals);
    
    [cx, cy] = meshgrid(-(cols/2):(cols/2-1), -(rows/2):(rows/2-1));
    radius = sqrt(cx.^2 + cy.^2); 
    radius(radius==0) = 1; 
    
    pink_filter = 1 ./ (radius.^1.0); 
    fft_pink = fft_shifted .* pink_filter;
    
    pink_field_raw = ifft2(ifftshift(fft_pink));
    pink_field = real(pink_field_raw);
    pink_field = pink_field / max(abs(pink_field(:))); 
end