clear all;
clc;

% Parameters
n = 2.0;
k_matrix = diag([0.04, 0.04, 0.1]);  % Disturbance force gain matrix (diagonal)

% Create meshgrid for distance and angle
distances = linspace(0.2, 1, 40);          
angles_deg = linspace(-90, 90, 40);        
[D, A] = meshgrid(distances, angles_deg);   
A_rad = deg2rad(A);                         

% Initialize force magnitude matrix
force_magnitude = zeros(size(D));

% Compute force magnitude at each point in the grid
for i = 1:size(D, 1)
    for j = 1:size(D, 2)
        d = D(i, j);                  
        theta = A_rad(i, j);          

        % Direction vector in the yz-plane (unit vector)
        unit_rij = [0; sin(theta); cos(theta)];

        % Disturbance force calculation
        force_vec = (-cos(theta) / d^n) * k_matrix * unit_rij;
        force_magnitude(i, j) = norm(force_vec);  
    end
end

% Enhanced 3D surface plot
figure;
s = surf(D, A, force_magnitude);  
s.EdgeColor = 'k';                % Black grid lines
s.LineStyle = '-';                % Solid lines
s.LineWidth = 0.05;                % Thin grid lines
s.FaceAlpha = 0.8;               % Almost opaque
s.FaceLighting = 'gouraud';       % Smooth lighting
s.AmbientStrength = 0.3;

xlabel('$\mathrm{Distance~(m)}$', 'FontSize', 12, 'FontWeight', 'bold', 'Interpreter', 'latex');
ylabel('$\mathrm{Angle~(^{\circ})}$', 'FontSize', 12, 'FontWeight', 'bold', 'Interpreter', 'latex');
zlabel('$\|\vec{F}\|~\mathrm{(N)}$', 'FontSize', 12, 'FontWeight', 'bold', 'Interpreter', 'latex');
title('$\mathrm{Force~vs.~Distance~and~Angle}$', 'FontSize', 13, 'FontWeight', 'bold', 'Interpreter', 'latex');

cbar = colorbar;
cbar.TickLabelInterpreter = 'latex';
cbar.Label.Interpreter = 'latex';
cbar.Label.FontSize = 12;
cbar.Label.FontWeight = 'bold';

% Custom pastel + vivid colormap with clearer contrast
better_contrast_map = [
    0.4  0.6  1.00;   % canlı mavi (daha koyu)
    0.6  0.85 0.6;    % açık yeşil (maviyle kontrast)
    0.95 0.95 0.4;    % limon sarısı
    1.00 0.6  0.2;    % turuncu
    1.00 0.4  0.6     % pembe-menekşe (canlı ama pastel)
];

% Interpolate to smooth 256-color map
contrast_colormap = interp1(linspace(0,1,size(better_contrast_map,1)), better_contrast_map, linspace(0,1,256));

% Apply it in your plot
colormap(contrast_colormap);


grid on;
shading faceted;

view(45, 30);          
material shiny;
%exportgraphics(gcf, 'force_surface_plot.eps', 'ContentType', 'vector');