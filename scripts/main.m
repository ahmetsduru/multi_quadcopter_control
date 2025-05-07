clear all;
clc;
num_drones = 5;
log_dir = '/home/asd/catkin_ws/src/multi_quadcopter_control/log/';
output_pdf = 'all_drones_plots.pdf';

if isfile(output_pdf)
    delete(output_pdf);
end
colors = lines(num_drones);
% Tüm drone'ların thrust ve zaman verileri için hücre dizileri
thrust_all = cell(num_drones, 1);
time_all = cell(num_drones, 1);

for i = 1:num_drones
    filename = fullfile(log_dir, sprintf('drone%d_log.txt', i));
    if ~isfile(filename)
        warning('File not found: %s', filename);
        continue;
    end

    data = readtable(filename, 'VariableNamingRule', 'preserve');
    time = data{:, "time"} - data{1, "time"};

    % === Actual & Reference Veriler ===
    x  = data{:, "a_pos.x"};    y  = data{:, "a_pos.y"};    z  = data{:, "a_pos.z"};
    xr = data{:, "r_pos.x"};    yr = data{:, "r_pos.y"};    zr = data{:, "r_pos.z"};

    vx  = data{:, "a_vel.x"};   vy  = data{:, "a_vel.y"};   vz = data{:, "a_vel.z"};
    vxr = data{:, "r_vel.x"};   vyr = data{:, "r_vel.y"};   vzr = data{:, "r_vel.z"};

    ax  = data{:, "a_acc.x"};   ay  = data{:, "a_acc.y"};   az = data{:, "a_acc.z"};
    axr = data{:, "r_acc.x"};   ayr = data{:, "r_acc.y"};   azr = data{:, "r_acc.z"};

    roll_a = data{:, "a_eul_ang.x"};  pitch_a = data{:, "a_eul_ang.y"};  yaw_a = data{:, "a_eul_ang.z"};
    roll_r = data{:, "r_eul_ang.x"};  pitch_r = data{:, "r_eul_ang.y"};  yaw_r = data{:, "r_eul_ang.z"};

    fx = data{:, "dist_f.x"};  fy = data{:, "dist_f.y"};  fz = data{:, "dist_f.z"};
    thrust = data{:, "r_thr"};

    tx = data{:, "r_torq.x"};  ty = data{:, "r_torq.y"};  tz = data{:, "r_torq.z"};

    % Thrust verilerini sakla
    thrust_all{i} = thrust;
    time_all{i} = time;

    % === POZISYON ===
    fig = figure;
    sgtitle(sprintf('Drone %d - Position vs Time', i), 'Interpreter','latex');
    subplot(3,1,1); plot(time, x, 'b', time, xr, 'k:', 'LineWidth', 1.5); ylabel('$X$ (m)', 'Interpreter','latex'); grid on;
    legend('Actual','Reference','Location','northeast','FontSize',4,'Box','off');
    subplot(3,1,2); plot(time, y, 'r', time, yr, 'k:', 'LineWidth', 1.5); ylabel('$Y$ (m)', 'Interpreter','latex'); grid on;
    legend('Actual','Reference','Location','northeast','FontSize',4,'Box','off');
    subplot(3,1,3); plot(time, z, 'g', time, zr, 'k:', 'LineWidth', 1.5); ylabel('$Z$ (m)', 'Interpreter','latex'); xlabel('$t$ (s)', 'Interpreter','latex'); grid on;
    legend('Actual','Reference','Location','northeast','FontSize',4,'Box','off');
    %exportgraphics(fig, sprintf('drone%d_position.eps', i), 'ContentType', 'vector');    
    exportgraphics(fig, output_pdf, 'Append', true); close(fig);

    % === 3D Position Error ===
    pos_error = sqrt((x - xr).^2 + (y - yr).^2 + (z - zr).^2);
    fig = figure;
    sgtitle(sprintf('Drone %d - 3D Position Error vs Time', i), 'Interpreter','latex');
    plot(time, pos_error, 'r', 'LineWidth', 1.5);
    xlabel('$t$ (s)', 'Interpreter','latex');
    ylabel('$e_{pos}$ (m)', 'Interpreter','latex');
    grid on;
    %exportgraphics(fig, sprintf('drone%d_pos_error.eps', i), 'ContentType', 'vector');    
    exportgraphics(fig, output_pdf, 'Append', true); close(fig);

    % === HIZ ===
    fig = figure;
    sgtitle(sprintf('Drone %d - Velocity vs Time', i), 'Interpreter','latex');
    subplot(3,1,1); plot(time, vx, 'b', time, vxr, 'k:', 'LineWidth', 1.5); ylabel('$X$ (m/s)', 'Interpreter','latex'); grid on;
    legend('Actual','Reference','Location','northeast','FontSize',4,'Box','off');
    subplot(3,1,2); plot(time, vy, 'r', time, vyr, 'k:', 'LineWidth', 1.5); ylabel('$Y$ (m/s)', 'Interpreter','latex'); grid on;
    legend('Actual','Reference','Location','northeast','FontSize',4,'Box','off');
    subplot(3,1,3); plot(time, vz, 'g', time, vzr, 'k:', 'LineWidth', 1.5); ylabel('$Z$ (m/s)', 'Interpreter','latex'); xlabel('$t$ (s)', 'Interpreter','latex'); grid on;
    legend('Actual','Reference','Location','northeast','FontSize',4,'Box','off');
    %exportgraphics(fig, sprintf('drone%d_velocity.eps', i), 'ContentType', 'vector');    
    exportgraphics(fig, output_pdf, 'Append', true); close(fig);

    % === IVME ===
    fig = figure;
    sgtitle(sprintf('Drone %d - Acceleration vs Time', i), 'Interpreter','latex');
    subplot(3,1,1); plot(time, ax, 'b', time, axr, 'k:', 'LineWidth', 1.5); ylabel('$X$ (m/s$^2$)', 'Interpreter','latex'); grid on;
    legend('Actual','Reference','Location','northeast','FontSize',4,'Box','off');
    subplot(3,1,2); plot(time, ay, 'r', time, ayr, 'k:', 'LineWidth', 1.5); ylabel('$Y$ (m/s$^2$)', 'Interpreter','latex'); grid on;
    legend('Actual','Reference','Location','northeast','FontSize',4,'Box','off');
    subplot(3,1,3); plot(time, az, 'g', time, azr, 'k:', 'LineWidth', 1.5); ylabel('$Z$ (m/s$^2$)', 'Interpreter','latex'); xlabel('$t$ (s)', 'Interpreter','latex'); grid on;
    legend('Actual','Reference','Location','northeast','FontSize',4,'Box','off');
    %exportgraphics(fig, sprintf('drone%d_acceleration.eps', i), 'ContentType', 'vector');    
    exportgraphics(fig, output_pdf, 'Append', true); close(fig);

    % === EULER AÇILARI ===
    fig = figure;
    sgtitle(sprintf('Drone %d - Euler Angles vs Time', i), 'Interpreter','latex');
    subplot(3,1,1); plot(time, roll_a, 'b', time, roll_r, 'k:', 'LineWidth', 1.5); ylabel('$\phi$ (rad)', 'Interpreter','latex'); grid on;
    legend('Actual','Reference','Location','northeast','FontSize',4,'Box','off');
    subplot(3,1,2); plot(time, pitch_a, 'r', time, pitch_r, 'k:', 'LineWidth', 1.5); ylabel('$\theta$ (rad)', 'Interpreter','latex'); grid on;
    legend('Actual','Reference','Location','northeast','FontSize',4,'Box','off');
    subplot(3,1,3); plot(time, yaw_a, 'g', time, yaw_r, 'k:', 'LineWidth', 1.5); ylabel('$\psi$ (rad)', 'Interpreter','latex'); xlabel('$t$ (s)', 'Interpreter','latex'); grid on;
    legend('Actual','Reference','Location','northeast','FontSize',4,'Box','off');
    %exportgraphics(fig, sprintf('drone%d_euler.eps', i), 'ContentType', 'vector');    
    exportgraphics(fig, output_pdf, 'Append', true); close(fig);
end

% === TÜM DRONE'LAR - THRUST ===
fig = figure;
hold on;
for i = 1:num_drones
    if ~isempty(thrust_all{i})
        plot(time_all{i}, thrust_all{i}, 'LineWidth', 1.5, 'Color', colors(i,:), 'DisplayName', sprintf('Quadcopter %d', i));
    end
end
hold off;
grid on;
xlabel('$t$ (s)', 'Interpreter','latex');
ylabel('$\left\|\textbf{f}_{\mathrm{thrust}}\right\|$ (N)', 'Interpreter', 'latex');
legend('show', 'Location','northeast', 'FontSize', 6, 'Box', 'off');
sgtitle('All Drones - Thrust vs Time', 'Interpreter','latex');
exportgraphics(fig, sprintf('drone_thrust.eps'), 'ContentType', 'vector');
exportgraphics(fig, output_pdf, 'Append', true);
close(fig);


%% === TÜM DRONE'LAR - DISTURBANCE FORCES ===
fig = figure;
sgtitle('All Drones - Disturbance Forces vs Time', 'Interpreter','latex');

% Fx
subplot(3,1,1); hold on;
for i = 1:num_drones
    filename = fullfile(log_dir, sprintf('drone%d_log.txt', i));
    if isfile(filename)
        data = readtable(filename, 'VariableNamingRule', 'preserve');
        time = data{:, "time"} - data{1, "time"};
        fx = data{:, "dist_f.x"};
        plot(time, fx, 'LineWidth', 1.2, 'Color', colors(i,:), 'DisplayName', sprintf('Quadcopter %d', i));
    end
end
ylabel('$f_{dist,x}$ (N)', 'Interpreter','latex'); grid on; legend('show','FontSize',6,'Box','off');

% Fy
subplot(3,1,2); hold on;
for i = 1:num_drones
    filename = fullfile(log_dir, sprintf('drone%d_log.txt', i));
    if isfile(filename)
        data = readtable(filename, 'VariableNamingRule', 'preserve');
        time = data{:, "time"} - data{1, "time"};
        fy = data{:, "dist_f.y"};
        plot(time, fy, 'LineWidth', 1.2, 'Color', colors(i,:), 'DisplayName', sprintf('Quadcopter %d', i));
    end
end
ylabel('$f_{dist,y}$ (N)', 'Interpreter','latex'); grid on; legend('show','FontSize',6,'Box','off');

% Fz
subplot(3,1,3); hold on;
for i = 1:num_drones
    filename = fullfile(log_dir, sprintf('drone%d_log.txt', i));
    if isfile(filename)
        data = readtable(filename, 'VariableNamingRule', 'preserve');
        time = data{:, "time"} - data{1, "time"};
        fz = data{:, "dist_f.z"};
        plot(time, fz, 'LineWidth', 1.2, 'Color', colors(i,:), 'DisplayName', sprintf('Quadcopter %d', i));
    end
end
ylabel('$f_{dist,z}$ (N)', 'Interpreter','latex'); xlabel('$t$ (s)', 'Interpreter','latex'); grid on; legend('show','FontSize',6,'Box','off');
exportgraphics(fig, sprintf('drone_disturbance.eps'), 'ContentType', 'vector');
exportgraphics(fig, output_pdf, 'Append', true);
close(fig);

%%
fig = figure;
sgtitle('All Drones - Position (Actual vs Reference) vs Time', 'Interpreter','latex');

% === X ===
subplot(3,1,1); hold on;
for i = 1:num_drones
    filename = fullfile(log_dir, sprintf('drone%d_log.txt', i));
    if isfile(filename)
        data = readtable(filename, 'VariableNamingRule', 'preserve');
        time = data{:, "time"} - data{1, "time"};
        x = data{:, "a_pos.x"};
        xr = data{:, "r_pos.x"};
        plot(time, x, '-',  'Color', colors(i,:), 'LineWidth', 1.2, ...
            'DisplayName', sprintf('Quadcopter %d', i));
        if i == 1  % sadece bir kez göster
            plot(time, xr, ':k', 'LineWidth', 1.0, 'DisplayName', 'Reference');
        else
            plot(time, xr, ':k', 'LineWidth', 1.0, 'HandleVisibility', 'off');
        end
    end
end
ylabel('$x$ (m)', 'Interpreter','latex'); grid on;
legend('show', 'FontSize', 5, 'Location', 'northeast', 'Box', 'off');

% === Y ===
subplot(3,1,2); hold on;
for i = 1:num_drones
    filename = fullfile(log_dir, sprintf('drone%d_log.txt', i));
    if isfile(filename)
        data = readtable(filename, 'VariableNamingRule', 'preserve');
        time = data{:, "time"} - data{1, "time"};
        y = data{:, "a_pos.y"};
        yr = data{:, "r_pos.y"};
        plot(time, y, '-',  'Color', colors(i,:), 'LineWidth', 1.2, ...
            'DisplayName', sprintf('Quadcopter %d', i));
        if i == 1
            plot(time, yr, ':k', 'LineWidth', 1.0, 'DisplayName', 'Reference');
        else
            plot(time, yr, ':k', 'LineWidth', 1.0, 'HandleVisibility', 'off');
        end
    end
end
ylabel('$y$ (m)', 'Interpreter','latex'); grid on;
legend('show', 'FontSize', 5, 'Location', 'northeast', 'Box', 'off');

% === Z ===
subplot(3,1,3); hold on;
for i = 1:num_drones
    filename = fullfile(log_dir, sprintf('drone%d_log.txt', i));
    if isfile(filename)
        data = readtable(filename, 'VariableNamingRule', 'preserve');
        time = data{:, "time"} - data{1, "time"};
        z = data{:, "a_pos.z"};
        zr = data{:, "r_pos.z"};
        plot(time, z, '-',  'Color', colors(i,:), 'LineWidth', 1.2, ...
            'DisplayName', sprintf('Quadcopter %d', i));
        if i == 1
            plot(time, zr, ':k', 'LineWidth', 1.0, 'DisplayName', 'Reference');
        else
            plot(time, zr, ':k', 'LineWidth', 1.0, 'HandleVisibility', 'off');
        end
    end
end
ylabel('$z$ (m)', 'Interpreter','latex');
xlabel('$t$ (s)', 'Interpreter','latex'); grid on;
legend('show', 'FontSize', 5, 'Location', 'northeast', 'Box', 'off');

% Export and close
exportgraphics(fig, 'drone_pos.eps', 'ContentType', 'vector');
exportgraphics(gcf, output_pdf, 'Append', true);  
close(fig);


%% === HER DRONE İÇİN 3D KONUM GRAFİĞİ ===
%for i = 1:num_drones
%    filename = fullfile(log_dir, sprintf('drone%d_log.txt', i));
%    if ~isfile(filename)
%        warning('File not found: %s', filename);
%        continue;
%    end
%
%    data = readtable(filename, 'VariableNamingRule', 'preserve');
%    x = data{:, "a_pos.x"};
%    y = data{:, "a_pos.y"};
%    z = data{:, "a_pos.z"};
%    xr = data{:, "r_pos.x"};
%    yr = data{:, "r_pos.y"};
%    zr = data{:, "r_pos.z"};
%
%    fig = figure;
%    hold on;
%    grid on;
%    axis equal;
%    plot3(x, y, z, '-', 'Color', colors(i,:), 'LineWidth', 1.5, 'DisplayName', 'Actual');
%    plot3(xr, yr, zr, ':k', 'LineWidth', 1.2, 'DisplayName', 'Reference');
%    xlabel('$X$ (m)', 'Interpreter','latex');
%    ylabel('$Y$ (m)', 'Interpreter','latex');
%    zlabel('$Z$ (m)', 'Interpreter','latex');
%    title(sprintf('Drone %d - 3D Position', i), 'Interpreter','latex');
%    legend('show', 'FontSize', 6, 'Box', 'off', 'Location', 'best');
%    view(45, 25); % açıyı isteğe göre ayarlayabilirsin
%
%    %exportgraphics(fig, sprintf('drone%d_3d_position.eps', i), 'ContentType', 'vector');
%    exportgraphics(gcf, output_pdf, 'Append', true);
%    close(fig);
%end

%% === TÜM DRONE'LAR - 3D KONUM (ACTUAL + REFERENCE) ===
%fig = figure;
%hold on;
%grid on;
%axis equal;
%
%for i = 1:num_drones
%    filename = fullfile(log_dir, sprintf('drone%d_log.txt', i));
%    if ~isfile(filename)
%        warning('File not found: %s', filename);
%        continue;
%    end
%
%    data = readtable(filename, 'VariableNamingRule', 'preserve');
%    x  = data{:, "a_pos.x"};  y  = data{:, "a_pos.y"};  z  = data{:, "a_pos.z"};
%    xr = data{:, "r_pos.x"};  yr = data{:, "r_pos.y"};  zr = data{:, "r_pos.z"};
%
%    % Gerçek ve referans yolları çiz
%    plot3(x, y, z, '-', 'Color', colors(i,:), 'LineWidth', 1.5, ...
%        'DisplayName', sprintf('Drone %d Actual', i));
%    plot3(xr, yr, zr, ':', 'Color', colors(i,:), 'LineWidth', 1.2, ...
%        'HandleVisibility', 'off');
%end
%
%xlabel('$X$ (m)', 'Interpreter','latex');
%ylabel('$Y$ (m)', 'Interpreter','latex');
%zlabel('$Z$ (m)', 'Interpreter','latex');
%title('All Drones - 3D Position (Actual Trajectories)', 'Interpreter','latex');
%legend('show', 'Location','bestoutside', 'FontSize', 6, 'Box','off');
%view(45, 25);  % Görüntüleme açısı
%
%% Kaydet
%%exportgraphics(fig, 'all_drones_3d_position.eps', 'ContentType', 'vector');
%exportgraphics(gcf, output_pdf, 'Append', true);
%close(fig);

%% === HER DRONE İÇİN 3D KONUM GRAFİĞİ ===
for i = 1:num_drones
    filename = fullfile(log_dir, sprintf('drone%d_log.txt', i));
    if ~isfile(filename)
        warning('File not found: %s', filename);
        continue;
    end

    data = readtable(filename, 'VariableNamingRule', 'preserve');
    x = data{:, "a_pos.x"};
    y = data{:, "a_pos.y"};
    z = data{:, "a_pos.z"};
    xr = data{:, "r_pos.x"};
    yr = data{:, "r_pos.y"};
    zr = data{:, "r_pos.z"};

    fig = figure;
    hold on;
    grid on;
    axis equal;
    plot3(x, y, z, '-', 'Color', colors(i,:), 'LineWidth', 1.5, 'DisplayName', 'Actual');
    plot3(xr, yr, zr, ':k', 'LineWidth', 1.2, 'DisplayName', 'Reference');

    % === SADECE 1. DRONE İÇİN WAYPOINTLERİ EKLE ===
    if i == 1
        % 4 farklı waypoint seti
        waypoint_sets = {
            [0.0, 0.0, 0.0, 2.0, 2.0, 2.0, 3.0], ...
            [0.0, 0.5, 2.5, 4.0, 7.0, 8.5, 10.0], ...
            [0.0, 1.5, 2.0, 2.0, 3.0, 2.0, 2.0];

            [-1.0, -4.0, -6.5, -12.0, -10.5, -3.0, -2.0], ...
            [3.0, 0.0, -5.2, -6.3, -7.0, -9.5, -10.0], ...
            [2.5, 2.0, 2.2, 2.2, 2.3, 2.4, 2.7];

            [-2.5, -1.0, 0.0, 0.0, 0.0, 0.0, 1.0], ...
            [-9.0, -11.0, -12.5, -14.0, -16.0, -18.0, -20.0], ...
            [2.5, 2.0, 2.0, 2.0, 3.0, 3.0, 3.0];

            [1.0, 0.0, 0.0, -1.0, 1.0, 1.0, 0.0], ...
            [-17.0, -14.0, -11.0, -8.0, -5.0, -2.0, 0.0], ...
            [2.0, 3.0, 3.0, 3.0, 3.0, 2.5, 0.0];
        };

        waypoint_colors = lines(4);  % 4 farklı renk

        for t = 1:4
            px = waypoint_sets{t,1};
            py = waypoint_sets{t,2};
            pz = waypoint_sets{t,3};
            scatter3(px, py, pz, 30, waypoint_colors(t,:), 'filled', ...
                     'HandleVisibility', 'off');  % legend'e eklenmez
        end
    end

    xlabel('$X$ (m)', 'Interpreter','latex');
    ylabel('$Y$ (m)', 'Interpreter','latex');
    zlabel('$Z$ (m)', 'Interpreter','latex');
    title(sprintf('Drone %d - 3D Position + Waypoints', i), 'Interpreter','latex');
    legend('show', 'FontSize', 6, 'Box', 'off', 'Location', 'best');
    view(45, 25);

    % === Grafik çıktıları ===
    %if i == 1
        savefig(fig, sprintf('drone%d_3d_position_with_waypoints.fig', i));
    %end

    exportgraphics(gcf, output_pdf, 'Append', true);
    close(fig);
end


%% === TÜM DRONE'LAR - VELOCITY (Actual vs Reference) ===
fig = figure;
sgtitle('All Drones - Velocity (Actual vs Reference) vs Time', 'Interpreter','latex');

% === Vx ===
subplot(3,1,1); hold on;
for i = 1:num_drones
    filename = fullfile(log_dir, sprintf('drone%d_log.txt', i));
    if isfile(filename)
        data = readtable(filename, 'VariableNamingRule', 'preserve');
        time = data{:, "time"} - data{1, "time"};
        vx = data{:, "a_vel.x"};
        vxr = data{:, "r_vel.x"};
        plot(time, vx, '-', 'Color', colors(i,:), 'LineWidth', 1.2, ...
             'DisplayName', sprintf('Quadcopter %d', i));
        if i == 1
            plot(time, vxr, ':k', 'LineWidth', 1.0, 'DisplayName', 'Reference');
        else
            plot(time, vxr, ':k', 'LineWidth', 1.0, 'HandleVisibility', 'off');
        end
    end
end
ylabel('$v_x$ (m/s)', 'Interpreter','latex'); grid on;
legend('show', 'FontSize', 5, 'Location', 'northeast', 'Box', 'off');

% === Vy ===
subplot(3,1,2); hold on;
for i = 1:num_drones
    filename = fullfile(log_dir, sprintf('drone%d_log.txt', i));
    if isfile(filename)
        data = readtable(filename, 'VariableNamingRule', 'preserve');
        time = data{:, "time"} - data{1, "time"};
        vy = data{:, "a_vel.y"};
        vyr = data{:, "r_vel.y"};
        plot(time, vy, '-', 'Color', colors(i,:), 'LineWidth', 1.2, ...
             'DisplayName', sprintf('Quadcopter %d', i));
        if i == 1
            plot(time, vyr, ':k', 'LineWidth', 1.0, 'DisplayName', 'Reference');
        else
            plot(time, vyr, ':k', 'LineWidth', 1.0, 'HandleVisibility', 'off');
        end
    end
end
ylabel('$v_y$ (m/s)', 'Interpreter','latex'); grid on;
legend('show', 'FontSize', 5, 'Location', 'northeast', 'Box', 'off');

% === Vz ===
subplot(3,1,3); hold on;
for i = 1:num_drones
    filename = fullfile(log_dir, sprintf('drone%d_log.txt', i));
    if isfile(filename)
        data = readtable(filename, 'VariableNamingRule', 'preserve');
        time = data{:, "time"} - data{1, "time"};
        vz = data{:, "a_vel.z"};
        vzr = data{:, "r_vel.z"};
        plot(time, vz, '-', 'Color', colors(i,:), 'LineWidth', 1.2, ...
             'DisplayName', sprintf('Quadcopter %d', i));
        if i == 1
            plot(time, vzr, ':k', 'LineWidth', 1.0, 'DisplayName', 'Reference');
        else
            plot(time, vzr, ':k', 'LineWidth', 1.0, 'HandleVisibility', 'off');
        end
    end
end
ylabel('$v_z$ (m/s)', 'Interpreter','latex'); xlabel('$t$ (s)', 'Interpreter','latex'); grid on;
legend('show', 'FontSize', 5, 'Location', 'northeast', 'Box', 'off');

% === Grafik çıktısı ===
exportgraphics(fig, 'all_drones_velocity.eps', 'ContentType', 'vector');
exportgraphics(fig, output_pdf, 'Append', true);
close(fig);


%% === TÜM DRONE'LAR - ACCELERATION (Actual vs Reference) ===
fig = figure;
sgtitle('All Drones - Acceleration (Actual vs Reference) vs Time', 'Interpreter','latex');

% === ax ===
subplot(3,1,1); hold on;
for i = 1:num_drones
    filename = fullfile(log_dir, sprintf('drone%d_log.txt', i));
    if isfile(filename)
        data = readtable(filename, 'VariableNamingRule', 'preserve');
        time = data{:, "time"} - data{1, "time"};
        ax = data{:, "a_acc.x"};
        axr = data{:, "r_acc.x"};
        plot(time, ax, '-', 'Color', colors(i,:), 'LineWidth', 1.2, ...
             'DisplayName', sprintf('Quadcopter %d', i));
        if i == 1
            plot(time, axr, ':k', 'LineWidth', 1.0, 'DisplayName', 'Reference');
        else
            plot(time, axr, ':k', 'LineWidth', 1.0, 'HandleVisibility', 'off');
        end
    end
end
ylabel('$a_x$ (m/s$^2$)', 'Interpreter','latex'); grid on;
legend('show', 'FontSize', 5, 'Location', 'northeast', 'Box', 'off');

% === ay ===
subplot(3,1,2); hold on;
for i = 1:num_drones
    filename = fullfile(log_dir, sprintf('drone%d_log.txt', i));
    if isfile(filename)
        data = readtable(filename, 'VariableNamingRule', 'preserve');
        time = data{:, "time"} - data{1, "time"};
        ay = data{:, "a_acc.y"};
        ayr = data{:, "r_acc.y"};
        plot(time, ay, '-', 'Color', colors(i,:), 'LineWidth', 1.2, ...
             'DisplayName', sprintf('Quadcopter %d', i));
        if i == 1
            plot(time, ayr, ':k', 'LineWidth', 1.0, 'DisplayName', 'Reference');
        else
            plot(time, ayr, ':k', 'LineWidth', 1.0, 'HandleVisibility', 'off');
        end
    end
end
ylabel('$a_y$ (m/s$^2$)', 'Interpreter','latex'); grid on;
legend('show', 'FontSize', 5, 'Location', 'northeast', 'Box', 'off');

% === az ===
subplot(3,1,3); hold on;
for i = 1:num_drones
    filename = fullfile(log_dir, sprintf('drone%d_log.txt', i));
    if isfile(filename)
        data = readtable(filename, 'VariableNamingRule', 'preserve');
        time = data{:, "time"} - data{1, "time"};
        az = data{:, "a_acc.z"};
        azr = data{:, "r_acc.z"};
        plot(time, az, '-', 'Color', colors(i,:), 'LineWidth', 1.2, ...
             'DisplayName', sprintf('Quadcopter %d', i));
        if i == 1
            plot(time, azr, ':k', 'LineWidth', 1.0, 'DisplayName', 'Reference');
        else
            plot(time, azr, ':k', 'LineWidth', 1.0, 'HandleVisibility', 'off');
        end
    end
end
ylabel('$a_z$ (m/s$^2$)', 'Interpreter','latex');
xlabel('$t$ (s)', 'Interpreter','latex'); grid on;
legend('show', 'FontSize', 5, 'Location', 'northeast', 'Box', 'off');

% === Çıktı ===
exportgraphics(fig, 'all_drones_acceleration.eps', 'ContentType', 'vector');
exportgraphics(fig, output_pdf, 'Append', true);
close(fig);


%% === TÜM DRONE'LAR - EULER AÇILARI (Actual vs Reference) ===
fig = figure;
sgtitle('All Drones - Euler Angles (Actual vs Reference) vs Time', 'Interpreter','latex');

% === Roll (phi) ===
subplot(3,1,1); hold on;
for i = 1:num_drones
    filename = fullfile(log_dir, sprintf('drone%d_log.txt', i));
    if isfile(filename)
        data = readtable(filename, 'VariableNamingRule', 'preserve');
        time = data{:, "time"} - data{1, "time"};
        roll_a = data{:, "a_eul_ang.x"};
        roll_r = data{:, "r_eul_ang.x"};
        plot(time, roll_a, '-', 'Color', colors(i,:), 'LineWidth', 1.2, ...
             'DisplayName', sprintf('Quadcopter %d', i));
        if i == 1
            plot(time, roll_r, ':k', 'LineWidth', 1.0, 'DisplayName', 'Reference');
        else
            plot(time, roll_r, ':k', 'LineWidth', 1.0, 'HandleVisibility', 'off');
        end
    end
end
ylabel('$\phi$ (rad)', 'Interpreter','latex'); grid on;
legend('show', 'FontSize', 5, 'Location', 'northeast', 'Box', 'off');

% === Pitch (theta) ===
subplot(3,1,2); hold on;
for i = 1:num_drones
    filename = fullfile(log_dir, sprintf('drone%d_log.txt', i));
    if isfile(filename)
        data = readtable(filename, 'VariableNamingRule', 'preserve');
        time = data{:, "time"} - data{1, "time"};
        pitch_a = data{:, "a_eul_ang.y"};
        pitch_r = data{:, "r_eul_ang.y"};
        plot(time, pitch_a, '-', 'Color', colors(i,:), 'LineWidth', 1.2, ...
             'DisplayName', sprintf('Quadcopter %d', i));
        if i == 1
            plot(time, pitch_r, ':k', 'LineWidth', 1.0, 'DisplayName', 'Reference');
        else
            plot(time, pitch_r, ':k', 'LineWidth', 1.0, 'HandleVisibility', 'off');
        end
    end
end
ylabel('$\theta$ (rad)', 'Interpreter','latex'); grid on;
legend('show', 'FontSize', 5, 'Location', 'northeast', 'Box', 'off');

% === Yaw (psi) ===
subplot(3,1,3); hold on;
for i = 1:num_drones
    filename = fullfile(log_dir, sprintf('drone%d_log.txt', i));
    if isfile(filename)
        data = readtable(filename, 'VariableNamingRule', 'preserve');
        time = data{:, "time"} - data{1, "time"};
        yaw_a = data{:, "a_eul_ang.z"};
        yaw_r = data{:, "r_eul_ang.z"};
        plot(time, yaw_a, '-', 'Color', colors(i,:), 'LineWidth', 1.2, ...
             'DisplayName', sprintf('Quadcopter %d', i));
        if i == 1
            plot(time, yaw_r, ':k', 'LineWidth', 1.0, 'DisplayName', 'Reference');
        else
            plot(time, yaw_r, ':k', 'LineWidth', 1.0, 'HandleVisibility', 'off');
        end
    end
end
ylabel('$\psi$ (rad)', 'Interpreter','latex');
xlabel('$t$ (s)', 'Interpreter','latex'); grid on;
legend('show', 'FontSize', 5, 'Location', 'northeast', 'Box', 'off');

exportgraphics(fig, 'all_drones_euler.eps', 'ContentType', 'vector');
exportgraphics(fig, output_pdf, 'Append', true);
close(fig);


%% === TÜM DRONE'LAR - 3D POZİSYON HATASI ===
fig = figure;
hold on;
sgtitle('All Drones - 3D Position Error vs Time', 'Interpreter','latex');

for i = 1:num_drones
    filename = fullfile(log_dir, sprintf('drone%d_log.txt', i));
    if isfile(filename)
        data = readtable(filename, 'VariableNamingRule', 'preserve');
        time = data{:, "time"} - data{1, "time"};
        x = data{:, "a_pos.x"};  y = data{:, "a_pos.y"};  z = data{:, "a_pos.z"};
        xr = data{:, "r_pos.x"}; yr = data{:, "r_pos.y"}; zr = data{:, "r_pos.z"};
        pos_error = sqrt((x - xr).^2 + (y - yr).^2 + (z - zr).^2);
        plot(time, pos_error, 'LineWidth', 1.5, 'Color', colors(i,:), ...
            'DisplayName', sprintf('Quadcopter %d', i));
    end
end

xlabel('$t$ (s)', 'Interpreter','latex');
ylabel('$\left\|\textbf{e}_{p}\right\|$ (m)', 'Interpreter', 'latex');
legend('show', 'Location', 'northeast', 'FontSize', 6, 'Box','off');
grid on;


% Grafik çıktısı
exportgraphics(fig, 'all_drones_position_error.eps', 'ContentType', 'vector');
exportgraphics(fig, output_pdf, 'Append', true);
close(fig);
