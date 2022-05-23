fixedObjects = cat(3,...
            trvec2tform([-0.1592 -0.6765 0.6338]) * eul2tform([0 0 0],'xyz'), ... Blue bottle
            trvec2tform([-0.1065 0.363 0.6743]) * eul2tform([0.0009 -0.0135 1.3401],'xyz'), ... Green can
            trvec2tform([0.0735 0.2639 0.6338]) * eul2tform([0 0 0],'xyz'), ...  Blue bottle 2
            trvec2tform([-0.1430 0.5261 0.5557]) * eul2tform([1.5592 1.4658 1.5682],'xyz'), ...  Yellow bottle 
            trvec2tform([0.2140 0.4408 0.5854]) * eul2tform([3.1301 -0.0084 -3.0172],'xyz'), ...  Red can
            trvec2tform([0.2214 0.5998 0.5559]) * eul2tform([1.5708 -0.4342 1.5320],'xyz')); % Red bottle

isBottle = [1 0 1 1 0 1]; % Blue bin


jntStateSub = ROSobjects.jntStateSub;

for i=1:size(fixedObjects,3)
    q = receive(jntStateSub).Position(2:8);
    moveTo(robot, q,fixedObjects(:,:,i), ROSobjects);

%     activateGripper('close');

    if isBottle(i)
        q = receive(jntStateSub).Position(2:8);
        moveTo(robot, q,blueBinPose, ROSobjects);
    else
        q = receive(jntStateSub).Position(2:8);
        moveTo(robot, q,greenBinPose, ROSobjects);
    end   
%     activateGripper('open');
end