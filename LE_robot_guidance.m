function [uu, stop, info] = LE_robot_guidance(t, lidar_scan, wheel_encoders, info)
    info.numberCalled = info.numberCalled + 1;
    info.numberCalled
    TenDegrees = 5*pi()/6/2; %2 is a fudge factor
    OneCM = 3/2/2; %Second 2 is a fudge factor This does not work.
    u = [0;0];
    stop = false;

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

% Plot the lidar in polar
    figure(2)
    h = polarplot(lidar_scan.theta,lidar_scan.range);

% Get the lidar data in a convenient variable
    theta = lidar_scan.theta * 180/pi();
    range = lidar_scan.range;
    [rangeMin,iMin]=min(range);
    thetaMin = theta(iMin);
    info.angleAtMin(info.numberCalled) = thetaMin;
    info.rangeAtMin(info.numberCalled) = rangeMin;

% Is this the first time in? Initialize
    if info.state == 0 %Initialize the routine
        if thetaMin > 0 %See if the left or right wall is the closest
            info.goal = 90; %left is closest
        else
            info.goal = -90; %right is closest
        end
        info.state = 1; %Align to wall
    end

% Align to within tol degrees to wall, doesn't work if starts in exact
% middle
    if info.state == 1 
        tol = 7;
        angMisalign = thetaMin - info.goal;
        if abs(angMisalign) > tol
            gain = .01;
            RotateAmount = gain*(angMisalign)*TenDegrees;
            if RotateAmount < 0 
                RotateAmount = -RotateAmount;
                u = clockWise()*RotateAmount;
            else
                u = counterClockWise()*RotateAmount;
            end
        else
            info.state = 2 %Wall walk
        end
    end

% Wall walk    
    if info.state == 2 
        forwardGain = 100;
        u = forward() * OneCM * forwardGain;
        % thetaMin
        % stop = true;
    end

    uu = double(u);

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