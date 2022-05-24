baseLinkPose = trvec2tform([-0.13 -0.1 0.6]);

fixedObjects = cat(3, ...
    trvec2tform([-0.1493 -0.6787 0.6138]) * eul2tform([0 pi/2 -pi/2], 'xyz'), ... Blue bottle
    trvec2tform([-0.1015 0.373 0.6743]) * eul2tform([0 pi/2 pi/2], 'xyz'), ... Green can
    trvec2tform([-0.1730 0.5161 0.5057]) * eul2tform([pi 0 pi/2], 'xyz'), ... Yellow bottle
    trvec2tform([0.2240 0.4508 0.5854]) * eul2tform([pi 0 pi/4], 'xyz'), ... Red can
    trvec2tform([0.2214 0.5998 0.5559]) * eul2tform([pi 0 pi/2], 'xyz')); % Red bottle

isBottle = [1 0 1 0 1]; % Blue bin

jntStateSub = ROSobjects.jntStateSub;

for i = 1:size(fixedObjects, 3)
    q = receive(jntStateSub).Position(2:8);

    T = baseLinkPose\fixedObjects(:, :, i);
    moveTo(robot, q, T, ROSobjects);

    activateGripper('close', ROSobjects);

    if isBottle(i)
        q = receive(jntStateSub).Position(2:8);
        moveTo(robot, q, blueBinPose, ROSobjects);
    else
        q = receive(jntStateSub).Position(2:8);
        moveTo(robot, q, greenBinPose, ROSobjects);
    end

    activateGripper('open', ROSobjects);
end
