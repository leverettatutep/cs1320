function [name, info] = robot_initialization()
% Arguments:
%   no arguments
% 
% Returns:
%   name: your name (string no spaces or special characters other than "_")
%   info: information which will be passed around back to your function

% You must set all of the return variables
name = 'test';
info.test = 0;
info.speed = 100000;
info.numberCalled = 0;
info.trialsToDo = 20;
info.angleAtMin = zeros(1,info.trialsToDo);
info.rangeAtMin = zeros(1,info.trialsToDo);
info.goal = 0;
info.state = 0; %initialize
info.closest = 10;
info.farthest = 20;
info.turnamount = 10;
end