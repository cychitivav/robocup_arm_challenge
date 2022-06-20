
pos_vec = [   -0.0193   -0.57    0.12
    0.0285    0.4630    0.1200
   -0.0430    0.6061    0.01
    0.3540    0.5508    0.04
    0.3514    0.6998    0.018];

%pos_vec(:,1) = pos_vec(:,1) - 0.2

orient_vec = [  0    pi   -pi/2;
            0    pi    pi/2;
            pi    0      pi/2;
            pi    0      pi/4;
            pi    0      pi/2];


posBotlBin = [-0.3 -0.45 h];        % blue bin
posCanBin = [-0.3 0.45 h];          % greeBin

isBottle = [1 0 1 0 1]; % Blue bin

jntStateSub = ROSobjects.jntStateSub;

for i = 1:size(pos_vec,1)
    
    % pick
    posTarget = pos_vec(i,:);

    controlPoints = posTarget;
    controlPoints(3) = controlPoints(3)+0.3;
    controlPoints = [controlPoints; posTarget]';
 
    
    orient = orient_vec(i,:);

    moveTo(robot,controlPoints, orient,ROSobjects)

    activateGripper('close', ROSobjects);
    

    % retract

    if isBottle(i)
       posBin = posBotlBin;
    else
       posBin= posCanBin;
    end

    posTarget(3)=h; 
    controlPoints = posTarget;
    
    posTarget(2)=posBin(2); 
    controlPoints = [controlPoints;posTarget];


    controlPoints = [controlPoints;posBin]';

    moveTo(robot,controlPoints, orient,ROSobjects);

    activateGripper('open', ROSobjects);
end

