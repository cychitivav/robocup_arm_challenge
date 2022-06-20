function [q, qd, qdd, t, dt] = computeTraj(jointInit, controlPoints,orient, robot,endEffector )
% Copyright 2021 MathWorks, Inc
%
% This function use features available in the Robotics System Toolbox to
% compute a smooth trajectory between the desired end effector position and
% the current robot configuration
%
% For more information on the features implemented here visit:
%   https://www.mathworks.com/help/robotics/referencelist.html?type=function&category=manipulators

factor = 1/2;
n=length(controlPoints)*20;

[s,sd,sdd,t] = trapveltraj(controlPoints,n);
t = t/factor;
dt=t(2);
t(end)
dt
ik = inverseKinematics('RigidBodyTree', robot);
%gik = generalizedInverseKinematics('RigidBodyTree', robot);

ik.SolverParameters.AllowRandomRestart = false;

% constrain = constraintCartesianBounds(endEffector);
% 
% constrain.Bounds=[ -0.1 0.1 
%                    -0.1 0.1
%                    -0.2 1]

weights = [1 1 1 0.9 0.9 0.8];


initialGuess = wrapToPi(jointInit);



for i =1:size(s,2)

    MTH= trvec2tform(s(:,i)')*eul2tform(orient, 'xyz');
    [robotPos(i, :),var] = ik(endEffector, MTH, weights, initialGuess);
    robotPos(i, :) = wrapToPi(robotPos(i, :));
    initialGuess = robotPos(i, :);
    
end


q = robotPos';
qd = gradient(q,dt);
qdd = gradient(qd,dt);



end
