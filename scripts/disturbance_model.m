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

% --- DİĞER PARAMETRELER ---
z0 = 0.05;       
sigma = 0.15;  
turb_intensity = 0.40; % Pink Noise şiddet katsayısı
Pos_Top = [0, 0, 0];  

%% 2. Izgara (Grid) Oluşturma
x_range = -1.5 : 0.02 : 1.5;  
z_range = -2.5 : 0.02 : -0.1;
[X, Z] = meshgrid(x_range, z_range);

%% 3. Pink Noise Alanlarının Oluşturulması
% Türbülansın x, y ve z bileşenleri için 3 ayrı Pink Noise haritası üretilir.
% Bu haritalar 1/f karakteristiğindedir (White Noise DEĞİLDİR).

[rows, cols] = size(X);

% 3 Eksen için ayrı ayrı Pink Noise (Colored Noise) üretimi
Pink_Map_X = create_pink_noise_field(rows, cols);
Pink_Map_Y = create_pink_noise_field(rows, cols);
Pink_Map_Z = create_pink_noise_field(rows, cols);

%% 4. Hesaplama Döngüsü
F_mag = zeros(size(X));
V_mag = zeros(size(X)); 

for i = 1:numel(X)
    p_b = [X(i); 0; Z(i)]; 
    p_t = Pos_Top';
    
    p_bt = p_t - p_b; 
    dist = norm(p_bt);
    
    if dist > 0.01
        n_vec = p_bt / dist; 
        d_z = abs(p_bt(3));
        d_xy = norm([p_bt(1); p_bt(2)]);
        
        % Deterministik Downwash Modeli (Sabit Kısım)
        decay_factor = (1 / (d_z^2 + z0)) * exp(-(d_xy^2) / (2*sigma^2));
        f_static = - K_matrix * n_vec * decay_factor;
        
        % --- PINK NOISE ENTEGRASYONU ---
        [r, c] = ind2sub(size(X), i);
        
        % İlgili koordinattaki Pink Noise değerini çek
        noise_vector = [Pink_Map_X(r,c); Pink_Map_Y(r,c); Pink_Map_Z(r,c)];
        
        % Türbülansı statik kuvvet üzerine uygula
        % White noise gibi keskin değil, akışkan bir bozucu etki yaratır.
        f_turbulence = abs(f_static) .* (turb_intensity * noise_vector);
        
        f_total = 4*(f_static + f_turbulence);
        F_mag(i) = norm(f_total);
        
        scaling_factor = rho_air * C_coup * 4 + eps; 
        V_mag(i) = sqrt(F_mag(i) / scaling_factor);
    else
        F_mag(i) = 0;
        V_mag(i) = 0;
    end
end

%% 5. Görselleştirme (Times New Roman - Makale Formatı)

figure('Name', 'Pink Noise Downwash Model', 'Color', 'w', 'Position', [100, 100, 1200, 600]);
t = tiledlayout(1, 2, 'TileSpacing', 'compact', 'Padding', 'compact'); 

% --- SOL GRAFİK: KUVVET PROFİLİ ---
nexttile; 
contourf(X, Z, F_mag, 100, 'LineStyle', 'none');
hold on;
plot(0, 0, 'kp', 'MarkerSize', 14, 'MarkerFaceColor', 'r'); 
text(0.1, 0.1, 'Source', 'Color', 'k', 'FontWeight', 'bold', 'FontSize', 14, 'FontName', 'Times New Roman');

cb1 = colorbar; 
set(cb1, 'LineWidth', 2, 'FontSize', 14, 'FontWeight', 'bold', 'FontName', 'Times New Roman');
colormap(jet); caxis([0 max(max(F_mag))]);

title({'Force Field (Pink Noise Model)', ' N'}, 'FontSize', 14, 'FontWeight', 'bold', 'FontName', 'Times New Roman');
xlabel('x (m)', 'FontSize', 14, 'FontWeight', 'bold', 'FontName', 'Times New Roman'); 
ylabel('z (m)', 'FontSize', 14, 'FontWeight', 'bold', 'FontName', 'Times New Roman');
grid off; axis equal; xlim([-0.7 0.7]); ylim([-2.5 0.5]);
set(gca, 'LineWidth', 2, 'FontSize', 14, 'FontWeight', 'bold', 'XColor', 'k', 'YColor', 'k', 'Box', 'on', 'FontName', 'Times New Roman');

% --- SAĞ GRAFİK: HIZ PROFİLİ ---
nexttile; 
contourf(X, Z, V_mag, 100, 'LineStyle', 'none');
hold on;
plot(0, 0, 'kp', 'MarkerSize', 14, 'MarkerFaceColor', 'r'); 
text(0.1, 0.1, 'Source', 'Color', 'k', 'FontWeight', 'bold', 'FontSize', 14, 'FontName', 'Times New Roman');

cb2 = colorbar; 
set(cb2, 'LineWidth', 2, 'FontSize', 14, 'FontWeight', 'bold', 'FontName', 'Times New Roman');
colormap(jet); caxis([0 max(max(V_mag))]);

title({'Velocity Field (Pink Noise Model)', ' m/s'}, 'FontSize', 14, 'FontWeight', 'bold', 'FontName', 'Times New Roman');
xlabel('x (m)', 'FontSize', 14, 'FontWeight', 'bold', 'FontName', 'Times New Roman'); 
ylabel('z (m)', 'FontSize', 14, 'FontWeight', 'bold', 'FontName', 'Times New Roman');
grid off; axis equal; xlim([-0.7 0.7]); ylim([-2.5 0.5]);
set(gca, 'LineWidth', 2, 'FontSize', 14, 'FontWeight', 'bold', 'XColor', 'k', 'YColor', 'k', 'Box', 'on', 'FontName', 'Times New Roman');

disp(['Model SADECE Pink Noise kullanılarak hesaplandı.']);

%% --- FONKSİYON: PINK NOISE ÜRETİCİSİ (1/f) ---
function pink_field = create_pink_noise_field(rows, cols)
    % Bu fonksiyon spektral sentez yöntemiyle saf Pink Noise üretir.
    % 1/f güç spektrumuna (Power Spectral Density) sahiptir.
    
    % 1. Rastgele Faz Başlangıcı (Seed)
    % Not: Pink noise üretimi için matematiksel bir "tohum" gerekir.
    % Bu aşama sadece frekans uzayını doldurmak içindir, çıktı filtrelenmiştir.
    noise_seed = randn(rows, cols);
    
    % 2. Fourier Dönüşümü (Frekans Uzayına Geçiş)
    fft_vals = fft2(noise_seed);
    fft_shifted = fftshift(fft_vals);
    
    % 3. 1/f Filtre Matrisi Oluşturma
    [cx, cy] = meshgrid(-(cols/2):(cols/2-1), -(rows/2):(rows/2-1));
    radius = sqrt(cx.^2 + cy.^2); 
    radius(radius==0) = 1; % DC bileşenini koru (sıfıra bölünmeyi önle)
    
    % Pink Noise Filtresi: Genlik 1/f ile orantılıdır (Güç 1/f^2)
    % Bu işlem yüksek frekanslı (White Noise) bileşenleri bastırır.
    pink_filter = 1 ./ (radius.^1.0); 
    
    % 4. Filtreyi Uygula
    fft_pink = fft_shifted .* pink_filter;
    
    % 5. Uzay Domenine Geri Dönüş (Inverse FFT)
    pink_field_raw = ifft2(ifftshift(fft_pink));
    
    % 6. Reel Bileşeni Al ve Normalize Et (-1 ile +1 arasına)
    pink_field = real(pink_field_raw);
    pink_field = pink_field / max(abs(pink_field(:))); 
end