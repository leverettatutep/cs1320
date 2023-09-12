% Note that you shouldn't have to do anything in this file
clearvars -EXCEPT sim_node
close all

%% Setup
% construct for the container class which I use to shuttle data around
interface = RobotInterface();

if ~exist('sim_node', 'var')
    setenv('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp');
    sim_node = ros2node(strcat(interface.name, '_robot_sim_node'));
end


% this sets up the stuff to send the state estimate out for recording
stop_publisher = ros2publisher(...
    sim_node,...
    strcat(interface.name, '/stop'),...
    'std_msgs/Header');
wheel_cmd_publisher = ros2publisher(...
    sim_node,...
    strcat(interface.name, '/wheel_cmd'),...
    'geometry_msgs/Twist');
pause(0.5)

% this sets up the stuff to send out simulated measurements
range_publisher = ros2publisher(...
    sim_node,...
    '/scan', ...
    'sensor_msgs/LaserScan');
wheel_publisher = ros2publisher(...
    sim_node,...
    '/wheels', ...
    'sensor_msgs/JointState');
pause(0.5)

% this sets up the stuff to get the sensor measurements from the robot. The
% measurement updates will be initiated by receiving a sensor message
scan_subscriber = ros2subscriber(...
    sim_node,...
    '/scan', ...
    @interface.update_lidar);
% this sets up the stuff to get the wheel speed measurements
wheel_subscriber = ros2subscriber(...
    sim_node,...
    '/wheels',...
    @interface.update_wheels);
% this sets up the stuff to get the wheel speed commands
wheel_cmd_subscriber = ros2subscriber(...
    sim_node,...
    strcat(interface.name, '/wheel_cmd'),...
    @interface.update_wheel_cmd);
pause(0.5)

%% Simulation initialization
% initialize time and the simulation step size
t = 0;
dt = 0.01;

% set up a flag to detect if we hit the wall
collision = false;

% Initialize the simulated robot position and velocity.
X = [0; rand() * 2 - 1; rand() * pi - pi / 2; 0; 0];

% wheel diameter is 2.625 inches, convert to radius and meters
r_wheel = 2.625 / 2 * 2.54 / 100;
y_wheel = 0.1;
counts_per_rad = 2^12 / 2.0 / pi;

lidar_interval = 0.1;
wheel_speed_interval = 0.05;
control_interval = 0.2;

% keep track of when the last time I got a measurement or ran the guidance.
% Randomize the measurement times to simulate them not being aligned
% perfectly with your prediction interval or with each other.
last_control = -inf;
last_lidar = -rand() * lidar_interval;
last_wheel_speed = -rand() * wheel_speed_interval;
next_redraw = inf;

% places to save things
X_save = [];
t_save = [];

% and indices to support saving
t_idx = 0;

wall_normals = [[0, 1]; [-1, 0]; [0, -1]];
wall_positions = [[2.5, -1]; [5, 0]; [2.5, 1]];
wall_x_lim = [[0, 5]; [-inf, inf]; [0, 5]];
wall_y_lim = [[-inf, inf]; [-1, 1]; [-inf, inf]];

% plot initialization
figure;
this_ax = gca;
axis(this_ax, 'equal')
xlim(this_ax, [-0.5, 5.5])
hold(this_ax, 'on')
walls = plot([0, 5, 5, 0], [-1, -1, 1, 1], Color='k', LineWidth=2);
vehicle = quiver(X(1), X(2), 0, 0, Color='c', LineWidth=2, MaxHeadSize=1);
scan = scatter([],[], 200, '.', MarkerFaceColor='m', MarkerEdgeColor='m');
history = plot(0, 0);
scatter(4, 0)

start_time = posixtime(datetime('now', 'TimeZone', 'local'));
% loop until you tell us to stop, we run into the wall, or ctrl-c
while ~collision && ~interface.stop
    t_idx = t_idx + 1;
    interface.t = t;
    
    % check to see if it is time to run the guidance
    if (t - last_control) >= control_interval
        omega = interface.get_guidance(t);
        u_cmd = [mean(omega) * r_wheel, diff(omega) * r_wheel / y_wheel];
        interface.publish(u_cmd, wheel_cmd_publisher);
        
        % this helps avoid numerical issues with the interval checks
        last_control = t - eps;
        
        % redraw right before the next command update
        next_redraw = t + control_interval - dt - eps;
    end

    % check to see if it is time to get a lidar measurement
    if (t - last_lidar) >= lidar_interval
        % R was found in experiment
        R_lidar = 8.7169e-7;

        n_theta = 35;
        theta_min = -pi + (rand() * pi / n_theta / 2);
        dtheta = 2 * pi / n_theta;
        theta_max = pi;
        theta = theta_min:dtheta:theta_max;
        theta = theta + X(3);

        v = [cos(theta); sin(theta)];

        % we have a corridor that is 5 meters long and 2 meters wide. The walls
        % are all planar. 
        %
        % -------------------------y--
        % |                        |   
        % |                    x---o        
        % |
        % ----------------------------
        d = inf(n_theta,1);
        intersections = zeros(2, n_theta);
        for idx = 1:n_theta
            [this_intersection, this_distance] = rayt(...
                X(1:2), v(:, idx), ...
                wall_positions, wall_normals, wall_x_lim, wall_y_lim);
            d(idx) = this_distance + sqrt(R_lidar) * randn();
            intersections(:, idx) = this_intersection;
        end
        
        last_lidar = last_lidar + lidar_interval;

        % send it out (to ourselves), when we get it a measurement update
        % will be triggered
        msg = ros2message(range_publisher);
        msg.header.frame_id = 'world';
        msg.angle_min = single(theta_min);
        msg.angle_max = single(theta_max);
        msg.angle_increment = single(dtheta);
        msg.range_min = single(0.01);
        msg.range_max = single(10.0);
        msg.ranges = single(d);
        ninf = sum(isinf(d));
        msg.ranges(isinf(d)) = rand(ninf, 1) * 5 + 5;
        send(range_publisher, msg);
        
        scan.XData = intersections(1, :);
        scan.XData(isinf(d)) = nan;
        scan.YData = intersections(2, :);
        scan.YData(isinf(d)) = nan;
    end
    if (t - last_wheel_speed) >= wheel_speed_interval
        last_wheel_speed = last_wheel_speed + wheel_speed_interval;

        % send it out (to ourselves), when we get it a measurement update
        % will be triggered
        msg = ros2message(wheel_publisher);
        msg.position = [
            floor(X(4) * counts_per_rad);
            floor(X(5) * counts_per_rad)];
        send(wheel_publisher, msg);
    end

    % if your filter has issued a stop command then send it to the robot
    if interface.stop
        msg = ros2message(stop_publisher);
        msg.Header.Stamp = posixtime(datetime('now', 'TimeZone', 'local'));
        send(stop_publisher, msg);
    end

    % throttle the simulation so that the messages can keep up
    now = posixtime(datetime('now', 'TimeZone', 'local'));
    pausetime = max(min(t - (now - start_time), dt), 0);
    pause(pausetime)

    % save the simulation step
    X_save(t_idx, :) = X;
    t_save(t_idx) = t;

    % step the simulation forward and check to see if we hit the obstacle
    if interface.stop
        break
    end
    
    cmd = interface.sim_cmd;
    u_l = cmd(1) - cmd(2) * y_wheel / 2;
    u_r = cmd(2) * y_wheel / 2 + cmd(1);
    u = [u_l; u_r];
    omega = u / r_wheel;
    u(u < -0.15) = -0.15;
    u(u > 0.15) = 0.15;
    v = mean(u);
    theta_dot = diff(u) / y_wheel;
    X_dot = [cos(X(3)) * v; sin(X(3)) * v; theta_dot; omega(1); omega(2)];
    X = X + X_dot * dt;
    t = t + dt;

    collision = X(1) > 5 || X(2) < -1 || X(2) > 1;

    if t >= next_redraw
        next_redraw = inf;
        
        vehicle.XData = X(1);
        vehicle.YData = X(2);
        vehicle.UData = cos(X(3)) / 2;
        vehicle.VData = sin(X(3)) / 2;
        if ~isempty(X_save)
            history.XData = X_save(:, 1);
            history.YData = X_save(:, 2);
        end
    end
end

if collision
    disp('Oh no! You hit a wall!')
else
    disp('Stopped on user command')
end

% % plot things
% plot_results(t_kalman, X_kalman_save, t_save, X_true_save)
% save( ...
%     strcat(interface.name, '_sim_results.mat'), ...
%     't_save', 'X_true_save', ...
%     't_kalman', 'X_kalman_save', 'P_kalman_save', ...
%     't_prior', ...
%     'X_prior_prev', 'X_prior_next', ...
%     'P_prior_prev', 'P_prior_next', ...
%     't_lidar', ...
%     'X_lidar_prior', 'X_lidar_posterior', ...
%     'P_lidar_prior', 'P_lidar_posterior', ...
%     't_wheel', ...
%     'X_wheel_prior', 'X_wheel_posterior', ...
%     'P_wheel_prior', 'P_wheel_posterior', ...
%     't_wheel_tilde', 'wheel_tilde', ...
%     't_lidar_tilde', 'lidar_tilde')