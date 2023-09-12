function [u, stop, info] = LE_robot_guidance(t, lidar_scan, wheel_encoders, info)
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
%   u -- a 1 x 2 array giving the commanded body translation and body rotation rate for the
%       left and right wheels. Left wheel should go in the first entry and
%       right in the second.
%   stop -- boolean, true if you want the robot to stop because you have
%       finished your task. False if you want to keep going
%   info -- anything you want to save. This will loop back in (the outer
%       managament code will give it back to you next time this function runs)
%       so if you need to keep track of anything between runs you can put it in
%       here. Make it an array, cell array, structure, whatever you like.
figure(2)
h = polarplot(lidar_scan.theta,lidar_scan.range);
[x,y] = pol2cart(lidar_scan.theta,lidar_scan.range);
filex = fopen('x.dat','w');
filey = fopen('y.dat','w');
fprintf(filex,'%f\r\n',x);
fprintf(filey,'%f\r\n',y);
fclose(filex);
fclose(filey);
figure(3)
hxy = scatter(x,y);
[p,s] = polyfit(x,y,1);
u = [0;0];
stop = false;
% u = input('Two speeds')
whatwant = input(['Right Left sTop Speed Go Cw ccW'],'s')
if whatwant == 'r'
    u = right();
end
if whatwant == 'l'
    u = left();
end
if whatwant == 't'
    u = stopit();
    stop = true;
end
if whatwant == 's'
    info.speed = input('Enter speed ');
end
if whatwant == 'c'
    u = clockWise();
end
if whatwant == 'w'
    u = counterClockWise();
end
if whatwant == 'g'
    u = forward();
end

u = u/.0333;
u = u * info.speed;
end

function u = left()
    u = [0;2];
end

function u = forward()
    u = [1;1];
end

function u = right()
    u = [2;0];
end

function u = stopit()
    u=[0;0];
end

function u = clockWise()
    u = [1;-1];
end

function u = counterClockWise()
    u = [-1;1];
end