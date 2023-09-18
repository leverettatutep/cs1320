function [u, stop, info] = LE_robot_guidance(t, lidar_scan, wheel_encoders, info)
    info.numberCalled = info.numberCalled + 1;
    info.numberCalled
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
% Diameter of the wheel = 2.625 inches = 0.0333375 meters
% Center of wheel is 0.1 meter (about 4 inches) from center of robot
% Vcenter = Wwheel * 0.03333375 and Vcenter = Wwheel / 0.1 so Wwheel = 
figure(2)
h = polarplot(lidar_scan.theta,lidar_scan.range);
theta = lidar_scan.theta * 180/pi();
range = lidar_scan.range;
[rangeMin,iMin]=min(abs(range));
thetaMin = theta(iMin);
info.angleAtMin(info.numberCalled) = thetaMin;
info.rangeAtMin(info.numberCalled) = rangeMin;

if info.numberCalled == 1 %Initialize the routine
    if thetaMin > 0 %See if the left or right wall is the closest
        info.goal = 90; %left is closest
    else
        info.goal = -90; %right is closest
    end
    info.firstTime = false;
end

TenDegrees = 5*pi()/6/2; %2 is a fudge factor
OneCM = 3/2/2; %Second 2 is a fudge factor This does not work.
u = [0;0];
stop = false;

%Which way am I pointed
[junk, ZeroIndex] = min(abs(theta));
if ZeroIndex + size(theta,2)/2 > size(theta,2)
    One80Index = ZeroIndex - floor(size(theta,2)/2);
else
    One80Index = ZeroIndex + floor(size(theta,2)/2); %not sure its floor
end

gain = .01;
RotateAmount = gain*(thetaMin - info.goal)*TenDegrees;
if RotateAmount < 0 
    RotateAmount = -RotateAmount;
    uu = clockWise();
else
    uu = counterClockWise()*RotateAmount;
end
u = double(uu * RotateAmount);

% if info.numberCalled == 10
    % stop = false;
    % u = clockWise()*TenDegrees;
    % u = [1,1] * OneCM;
% end
% if info.numberCalled == info.trialsToDo
%     stop = true;
%     u = [0;0];
%     fid = fopen('id.dat','w');
%     fprintf(fid,'%f,',info.angleAtMin);
%     fclose(fid);
% end

% % [x,y] = pol2cart(lidar_scan.theta,lidar_scan.range);
% % filex = fopen('x.dat','w');
% % filey = fopen('y.dat','w');
% % fprintf(filex,'%f\r\n',x);
% % fprintf(filey,'%f\r\n',y);
% % fclose(filex);
% % fclose(filey);
% % figure(3)
% % hxy = scatter(x,y);
% % [p,s] = polyfit(x,y,1);
% % u = [0;0];
% % stop = false;
% % % u = input('Two speeds')
% % whatwant = input(['Cw or ccW and speed'],'s')
% % if whatwant == 't'
% %     u = stopit();
% %     stop = true;
% % end
% % if whatwant == 'c'
% %     u = clockWise();
% % end
% % if whatwant == 'w'
% %     u = counterClockWise();
% % end
% % 
% % u = u/.0333;
% % u = u * info.speed;
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