num_drones = 6;
log_dir = '/home/asd/catkin_ws/src/multi_quadcopter_control/log/';
output_pdf = 'all_drones_plots.pdf';

if isfile(output_pdf)
    delete(output_pdf);
end

for i = 1:num_drones
    filename = fullfile(log_dir, sprintf('drone%d_log.txt', i));
    if ~isfile(filename)
        warning('Dosya bulunamadı: %s', filename);
        continue;
    end

    data = readtable(filename, 'VariableNamingRule', 'preserve');
    time = data{:, "time"} - data{1, "time"};

    % === Actual & Reference Veriler ===
    x  = data{:, "a_pos.x"};    y  = data{:, "a_pos.y"};    z  = data{:, "a_pos.z"};
    xr = data{:, "r_pos.x"};    yr = data{:, "r_pos.y"};    zr = data{:, "r_pos.z"};

    vx  = data{:, "a_vel.x"};   vy  = data{:, "a_vel.y"};   vz  = data{:, "a_vel.z"};
    vxr = data{:, "r_vel.x"};   vyr = data{:, "r_vel.y"};   vzr = data{:, "r_vel.z"};

    ax  = data{:, "a_acc.x"};   ay  = data{:, "a_acc.y"};   az  = data{:, "a_acc.z"};
    axr = data{:, "r_acc.x"};   ayr = data{:, "r_acc.y"};   azr = data{:, "r_acc.z"};

    roll_a = data{:, "a_eul_ang.x"};  pitch_a = data{:, "a_eul_ang.y"};  yaw_a = data{:, "a_eul_ang.z"};
    roll_r = data{:, "r_eul_ang.x"};  pitch_r = data{:, "r_eul_ang.y"};  yaw_r = data{:, "r_eul_ang.z"};

    fx = data{:, "dist_f.x"};  fy = data{:, "dist_f.y"};  fz = data{:, "dist_f.z"};
    thrust = data{:, "r_thr"};

    % === POZISYON ===
    fig = figure;
    sgtitle(sprintf('Drone %d - Position vs Time', i));
    subplot(3,1,1); plot(time, x, 'b', time, xr, 'k:', 'LineWidth', 1.5); ylabel('X (m)'); grid on;
    legend('Actual','Reference','Location','northeast','FontSize',4,'Box','off');
    subplot(3,1,2); plot(time, y, 'r', time, yr, 'k:', 'LineWidth', 1.5); ylabel('Y (m)'); grid on;
    legend('Actual','Reference','Location','northeast','FontSize',4,'Box','off');
    subplot(3,1,3); plot(time, z, 'g', time, zr, 'k:', 'LineWidth', 1.5); ylabel('Z (m)'); xlabel('Time (s)'); grid on;
    legend('Actual','Reference','Location','northeast','FontSize',4,'Box','off');
    exportgraphics(fig, output_pdf, 'Append', true); close(fig);

    % === HIZ ===
    fig = figure;
    sgtitle(sprintf('Drone %d - Velocity vs Time', i));
    subplot(3,1,1); plot(time, vx, 'b', time, vxr, 'k:', 'LineWidth', 1.5); ylabel('X (m/s)'); grid on;
    legend('Actual','Reference','Location','northeast','FontSize',4,'Box','off');
    subplot(3,1,2); plot(time, vy, 'r', time, vyr, 'k:', 'LineWidth', 1.5); ylabel('Y (m/s)'); grid on;
    legend('Actual','Reference','Location','northeast','FontSize',4,'Box','off');
    subplot(3,1,3); plot(time, vz, 'g', time, vzr, 'k:', 'LineWidth', 1.5); ylabel('Z (m/s)'); xlabel('Time (s)'); grid on;
    legend('Actual','Reference','Location','northeast','FontSize',4,'Box','off');
    exportgraphics(fig, output_pdf, 'Append', true); close(fig);

    % === IVME ===
    fig = figure;
    sgtitle(sprintf('Drone %d - Acceleration vs Time', i));
    subplot(3,1,1); plot(time, ax, 'b', time, axr, 'k:', 'LineWidth', 1.5); ylabel('X (m/s^2)'); grid on;
    legend('Actual','Reference','Location','northeast','FontSize',4,'Box','off');
    subplot(3,1,2); plot(time, ay, 'r', time, ayr, 'k:', 'LineWidth', 1.5); ylabel('Y (m/s^2)'); grid on;
    legend('Actual','Reference','Location','northeast','FontSize',4,'Box','off');
    subplot(3,1,3); plot(time, az, 'g', time, azr, 'k:', 'LineWidth', 1.5); ylabel('Z (m/s^2)'); xlabel('Time (s)'); grid on;
    legend('Actual','Reference','Location','northeast','FontSize',4,'Box','off');
    exportgraphics(fig, output_pdf, 'Append', true); close(fig);

    % === THRUST ===
    fig = figure;
    sgtitle(sprintf('Drone %d - Thrust vs Time', i));
    plot(time, thrust, 'm', 'LineWidth', 1.5); grid on;
    xlabel('Time (s)'); ylabel('Thrust (N)');
    legend('Reference','Location','northeast','FontSize',4,'Box','off');
    exportgraphics(fig, output_pdf, 'Append', true); close(fig);

    % === DISTURBANCE FORCE ===
    fig = figure;
    sgtitle(sprintf('Drone %d - Disturbance Force vs Time', i));
    subplot(3,1,1); plot(time, fx, 'c', 'LineWidth', 1.5); ylabel('Fx (N)'); grid on;
    subplot(3,1,2); plot(time, fy, 'm', 'LineWidth', 1.5); ylabel('Fy (N)'); grid on;
    subplot(3,1,3); plot(time, fz, 'y', 'LineWidth', 1.5); ylabel('Fz (N)'); xlabel('Time (s)'); grid on;
    exportgraphics(fig, output_pdf, 'Append', true); close(fig);

    % === EULER AÇILARI ===
    fig = figure;
    sgtitle(sprintf('Drone %d - Euler Angles vs Time', i));
    subplot(3,1,1); plot(time, roll_a, 'b', time, roll_r, 'k:', 'LineWidth', 1.5); ylabel('Roll (rad)'); grid on;
    legend('Actual','Reference','Location','northeast','FontSize',4,'Box','off');
    subplot(3,1,2); plot(time, pitch_a, 'r', time, pitch_r, 'k:', 'LineWidth', 1.5); ylabel('Pitch (rad)'); grid on;
    legend('Actual','Reference','Location','northeast','FontSize',4,'Box','off');
    subplot(3,1,3); plot(time, yaw_a, 'g', time, yaw_r, 'k:', 'LineWidth', 1.5); ylabel('Yaw (rad)'); xlabel('Time (s)'); grid on;
    legend('Actual','Reference','Location','northeast','FontSize',4,'Box','off');
    exportgraphics(fig, output_pdf, 'Append', true); close(fig);
end
