clear all;
clc;

%% Parameters
n = 2.0;
k_matrix = diag([0.04, 0.04, 0.1]);  % Disturbance force gain matrix (diagonal)

%% Create meshgrid for distance and gamma (angle)
distances_positive = linspace(0.2, 1, 40);    
distances_negative = linspace(-1, -0.2, 40);  
distances = [distances_negative, distances_positive];  % Merge negative and positive distances

angles_deg = linspace(-90, 90, 40);        
[D, A] = meshgrid(distances, angles_deg);   
A_rad = deg2rad(A);

%% Initialize force magnitude matrix
force_magnitude = zeros(size(D));

%% Compute force magnitude at each point
for i = 1:size(D, 1)
    for j = 1:size(D, 2)
        d = D(i, j);                  
        gamma = A_rad(i, j);           % GAMMA in radians

        % Direction vector in the yz-plane (unit vector)
        unit_rij = [0; sin(gamma); cos(gamma)];

        % Disturbance force calculation
        if abs(d) >= 0.2
            force_vec = (-cos(gamma) / d^n) * k_matrix * unit_rij;
            force_magnitude(i, j) = norm(force_vec);
        else
            force_magnitude(i, j) = NaN;  % Collision region (assign NaN for clarity)
        end
    end
end

%% Enhanced 3D surface plot
figure;
s = surf(D, A, force_magnitude);  
s.EdgeColor = 'k';                
s.LineStyle = '-';                
s.LineWidth = 0.05;                
s.FaceAlpha = 0.8;               
s.FaceLighting = 'gouraud';       
s.AmbientStrength = 0.3;
shading faceted;
hold on;

% Highlight Collision Region (-0.2m < d < 0.2m)
collision_x = [-0.2, 0.2, 0.2, -0.2];
collision_y = [-90, -90, 90, 90];
collision_z_top = ones(1,4) * max(force_magnitude(:), [], 'omitnan') * 1.1;
collision_z_bottom = zeros(1,4);

fill3(collision_x, collision_y, collision_z_top, 'r', 'FaceAlpha', 0.15, 'EdgeColor', 'none');
fill3(collision_x, collision_y, collision_z_bottom, 'r', 'FaceAlpha', 0.15, 'EdgeColor', 'none');

% Label the collision zone
text(0, 0, max(force_magnitude(:), [], 'omitnan')*1.15, ...
    '\textbf{Collision Region}', 'HorizontalAlignment', 'center', ...
    'Interpreter', 'latex', 'FontSize', 14, 'FontWeight', 'bold', 'Color', 'r');

%% Labels and Titles
xlabel('$d_{bt}~(m)$', 'FontSize', 14, 'FontWeight', 'bold', 'Interpreter', 'latex');
ylabel('$\gamma~(^{\circ})$', 'FontSize', 14, 'FontWeight', 'bold', 'Interpreter', 'latex');
zlabel('$\|\textbf{f}_{dist}\|~\mathrm{(N)}$', 'FontSize', 14, 'FontWeight', 'bold', 'Interpreter', 'latex');

cbar = colorbar;
cbar.TickLabelInterpreter = 'latex';
cbar.Label.Interpreter = 'latex';
cbar.Label.FontSize = 14;
cbar.Label.FontWeight = 'bold';

%% Colormap
better_contrast_map = [
    0.4  0.6  1.00;   
    0.6  0.85 0.6;    
    0.95 0.95 0.4;    
    1.00 0.6  0.2;    
    1.00 0.4  0.6     
];
contrast_colormap = interp1(linspace(0,1,size(better_contrast_map,1)), better_contrast_map, linspace(0,1,256));
colormap(contrast_colormap);

grid on;
view(30, 20);          
material shiny;

%% Optional: Save the figure
% exportgraphics(gcf, 'force_surface_plot_with_collision.eps', 'ContentType', 'vector');

hold off;
