function [q, qd, qdd, t, dt] = computeTraj(jointInit, controlPoints,orient, robot)
% Copyright 2021 MathWorks, Inc
%
% This function use features available in the Robotics System Toolbox to
% compute a smooth trajectory between the desired end effector position and
% the current robot configuration
%
% For more information on the features implemented here visit:
%   https://www.mathworks.com/help/robotics/referencelist.html?type=function&category=manipulators

n=length(controlPoints)*5;

[s,sd,sdd,t] = trapveltraj(controlPoints,n);
dt=t(2);

timestep = 0.1;
ik = inverseKinematics('RigidBodyTree', robot);
ik.SolverParameters.AllowRandomRestart = false;
weights = [1 1 1 0.8 0.8 0.8];
endEffector = 'gripper';

initialGuess = wrapToPi(jointInit);


for i =1:size(s,2)

    MTH= trvec2tform(s(:,i)')*eul2tform(orient, 'xyz');
    [robotPos(i, :),var] = ik(endEffector, MTH, weights, initialGuess);
    robotPos(i, :) = robotPos(i, :);
    initialGuess = wrapToPi(robotPos(i, :));
    
end


q = robotPos';
qd = gradient(q,dt);
qdd = gradient(qd,dt);



end
