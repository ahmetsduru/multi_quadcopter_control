% Drone positions
pos_bottom = [0.2, 0.2, 0.2];    % Bottom drone (i)
pos_top = [0.3, 0.6, 0.7];   % Top drone (j)

% Compute relative vector and distance
r_ij = pos_top - pos_bottom;
d = norm(r_ij);

% Compute unit vector
unit_rij = r_ij / d;

% Compute angle with respect to vertical (z-axis)
theta_rad = acos(unit_rij(3));   
theta_deg = rad2deg(theta_rad);  

% Plot
figure;
hold on;
grid on;
axis equal;
xlabel('$x$ (m)', 'Interpreter', 'latex', 'FontSize', 14);
ylabel('$y$ (m)', 'Interpreter', 'latex', 'FontSize', 14);
zlabel('$z$ (m)', 'Interpreter', 'latex', 'FontSize', 14);

% Plot drone positions
scatter3(pos_bottom(1), pos_bottom(2), pos_bottom(3), 100, 'filled', 'b'); % Bottom drone
scatter3(pos_top(1), pos_top(2), pos_top(3), 100, 'filled', 'r');           % Top drone

% Plot r_ij vector (full 3D vector)
quiver3(pos_bottom(1), pos_bottom(2), pos_bottom(3), ...
        r_ij(1), r_ij(2), r_ij(3), ...
        0, 'LineWidth', 2, 'Color', 'k', 'MaxHeadSize', 0.25);

% === NEW: Plot the projection of r_ij onto the XY plane ===
r_ij_xy = [r_ij(1), r_ij(2), 0];  % Z bileşeni sıfırlandı
quiver3(pos_bottom(1), pos_bottom(2), pos_bottom(3), ...
        r_ij_xy(1), r_ij_xy(2), r_ij_xy(3), ...
        0, 'LineWidth', 2, 'Color', [0.5 0.5 0.5], 'LineStyle', '--', 'MaxHeadSize', 0.25);

legend('Bottom UAV', 'Top UAV', 'Interpreter', 'latex');
view(45, 30);
