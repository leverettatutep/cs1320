function [u, stop, info] = robot_guidance(t, lidar_scan, wheel_encoders, info)
% robot guidance function
%
% This will be called by the sim / robot interface. This function receives
% sensor data which you can use to compute wheel speed commands that will be
% used to steer the robot down the corridor. Output should be the rotation rate
% of each wheel in rad/s. The example code here creates a randomly driving
% robot.
%
% Arguments:
%   t -- the current time (in seconds). Zero is the unix epoch (midnight
%        GMT on 1970-01-01)
%   lidar_scan -- a structure with fields:
%       .lidar_theta - a vector of angles (rad) where there is a lidar distance
%           reading available. An angle of 0 is straight ahead of the robot.
%           The axis of rotation is vertically "up" and rotations are
%           right-handed.
%       .lidar_range - a vector of distance readings from the lidar (m)
%           corresponding to the angles in lidar_range
%   wheel_encoders -- a structure with fields:
%       .left -- encoder count for rotation of the left wheel. Positive is the
%           robot moving forward.
%       .right -- encoder count for rotation of the right wheel. Positive is
%           the robot moving forward.
% 
% Returns:
%   u -- a 1 x 2 array giving the commanded rotation rate (rad / s) for the
%       left and right wheels. Left wheel should go in the first entry and
%       right in the second.
%   stop -- boolean, true if you want the robot to stop because you have
%       finished your task. False if you want to keep going
%   info -- anything you want to save. This will loop back in (the outer
%       managament code will give it back to you next time this function runs)
%       so if you need to keep track of anything between runs you can put it in
%       here. Make it an array, cell array, structure, whatever you like.
u = [1, 1] * 0.15 + randn(1, 2) / 10;
if abs(mean(u)) > 0.15
    u = u / norm(u) * 0.15;
end
u = u/0.0333;
stop = false;
end