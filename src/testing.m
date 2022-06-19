clear, clc, close all


load data.mat

load('exampleHelperKINOVAGen3GripperROSGazebo.mat');
endEffector = 'gripper';
ik = inverseKinematics('RigidBodyTree', robot);

% Add camera link to bracelet link
bodyCam = rigidBody('camera');
camera_MTH = trvec2tform([0 -0.055 -0.055])*eul2tform([pi/2 0 0],'xyz');
jnt = rigidBodyJoint('jnt','fixed');
setFixedTransform(jnt,camera_MTH)
bodyCam.Joint = jnt;

addBody(robot,bodyCam,'Bracelet_Link')

currentRobotJConfig = homeConfiguration(robot);


target_pos = mean(pcWorld.Location);



pos = [ 0.0285    0.4730    0.0900
       -0.0430    0.6161   -0.0800
        0.3540    0.5508   -0.0146
        0.3       0.4       0
        0.4       0.2       0
        0.35      0         0   
        0.35      -0.2      0
        0.3       -0.4      0];



controlPoints = pos';
controlPoints = posObj';

n=length(pos)*5;
tInterval = [0 3];
t = linspace(0,3,n);
dt=t(2);



[s,sd,sdd,pp] = bsplinepolytraj(controlPoints,tInterval,t);


[s,sd,sdd,t] = trapveltraj(controlPoints,n);

plot3(s(1,:),s(2,:),s(3,:))
hold on
plot3(controlPoints(1,:),controlPoints(2,:),controlPoints(3,:),'-*')

figure(2)
subplot(3,1,1)
plot(t,s')

subplot(3,1,2)
plot(t,sd')

subplot(3,1,3)
plot(t,sdd')

jointInit =[ 0 0 0 0 0 0 0]

figure(3)
subplot(2,1,1)
    pcshow(pcWorld)
    view(-90,0)
    ylim( [-0.5   0.8])
    zlim( [-0.2   0.8])
    hold on
    plot3(controlPoints(1,:),controlPoints(2,:),controlPoints(3,:),'-*')

subplot(2,1,2)
    pcshow(pcWorld)
    view(-90,90)
    ylim( [-0.5   0.8])
    xlim( [-0.2   0.8])
    hold on
    plot3(controlPoints(1,:),controlPoints(2,:),controlPoints(3,:),'-*')


%%
initialGuess = wrapToPi(jointInit);
weights = [1 1 1 0.8 0.8 0.8];

for i =1:size(s,2)

    MTH= trvec2tform(s(:,i)')*eul2tform([0 pi 0], 'xyz');
    [robotPos(i, :),var] = ik(endEffector, MTH, weights, initialGuess);
    robotPos(i, :) = robotPos(i, :);
    initialGuess = wrapToPi(robotPos(i, :));
    
end


q = robotPos';
qd = gradient(q,dt);
qdd = gradient(qd,dt)';

%%
figure(3)
subplot(3,1,1)
plot(t,q')

subplot(3,1,2)
plot(t,qd')

subplot(3,1,3)
plot(t,qdd')


pause(1)
close all
figure(1)
for k=1:size(q,2)
    
  
    pcshow(pcWorld)
    view(90,15)
    xlim( [-0.5 1])
    ylim( [-1   1])
    zlim( [-0.2   0.8])
    hold on
    show(robot,q(:,k)')
    plot3(s(1,1:k),s(2,1:k),s(3,1:k))
    plot3(controlPoints(1,:),controlPoints(2,:),controlPoints(3,:),'-*')
    hold off
    pause(0.2)
end