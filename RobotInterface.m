% This implements a container class for a robot interface. This way I can
% keep everything together as I call the steps from different places. You
% should not need to touch this.
classdef RobotInterface < handle
    properties (Access = public)
        lidar_valid
        lidar_theta
        lidar_range
        lidar_theta_history
        lidar_range_history

        gyro_valid
        gyro_rate
        gyro_history

        wheel_1_valid
        wheel_1_count
        wheel_1_command
        wheel_1_history
        wheel_1_command_history

        wheel_2_valid
        wheel_2_count
        wheel_2_command
        wheel_2_history
        wheel_2_command_history

        sim_cmd

        t
        stop
        info
        name
    end
    methods (Access = public)
        function obj = RobotInterface()
            obj.stop = false;
            [name, info] = robot_initialization();
            obj.name = name;
            obj.info = info;

            obj.lidar_valid = false;
            obj.gyro_valid = false;
            obj.wheel_1_valid = false;
            obj.wheel_2_valid = false;

            % some places to save intermediate results
            obj.t = [];
            obj.lidar_theta_history = [];
            obj.lidar_range_history = [];
            obj.gyro_history = [];
            obj.wheel_1_history = [];
            obj.wheel_1_command_history = [];
            obj.wheel_2_history = [];
            obj.wheel_2_command_history = [];

            obj.sim_cmd = [0, 0];
        end

        function update_lidar(obj, msg)
            obj.lidar_theta = msg.angle_min:msg.angle_increment:msg.angle_max;
            obj.lidar_range = msg.ranges';
            obj.lidar_valid = true;
        end

        function update_wheels(obj, msg)    
            obj.wheel_1_count = msg.position(1);
            obj.wheel_2_count = msg.position(2);
            obj.wheel_1_valid = true;
            obj.wheel_2_valid = true;
        end

        function update_wheel_cmd(obj, msg)
            obj.sim_cmd = [msg.linear.x, msg.angular.z];
        end

        function u = get_guidance(obj, t)
            if ~(obj.lidar_valid && obj.wheel_1_valid && obj.wheel_2_valid)
                u = [0; 0];
                return
            end

            lidar_scan.theta = obj.lidar_theta;
            lidar_scan.range = obj.lidar_range;

            wheel_encoders.left = obj.wheel_1_count;
            wheel_encoders.right = obj.wheel_2_count;


            [u, stop, info] = LE_robot_guidance(...
                posixtime(datetime('now', 'TimeZone', 'local')), ...
                lidar_scan, wheel_encoders, obj.info);
            obj.stop = stop;
            obj.info = info;

            obj.t(end + 1) = t;
            obj.lidar_theta_history(end + 1, :) = obj.lidar_theta;
            obj.lidar_range_history(end + 1, :) = obj.lidar_range;
            obj.wheel_1_history(end + 1) = obj.wheel_1_count;
            obj.wheel_1_command_history(end + 1) = u(1);
            obj.wheel_2_history(end + 1) = obj.wheel_2_count;
            obj.wheel_2_command_history(end + 1) = u(2);
        end

        function publish(obj, u, cmd_pub)
            cmd_msg = ros2message(cmd_pub);
            if obj.stop
                cmd_msg.linear.x = 0;
                cmd_msg.angular.z = 0;
            else
                cmd_msg.linear.x = u(1);
                cmd_msg.angular.z = u(2);
            end
            send(cmd_pub, cmd_msg)
        end
    end
end