function [ptCloud, msh] = estimate(robot, ROSobjects)

inspectionPoses = cat(3, ...
    trvec2tform([0.4 -0.25 0.5]) * eul2tform(deg2rad([-160 10 0]), 'xyz'), ...
    trvec2tform([0.4 0.25 0.5]) * eul2tform(deg2rad([160 10 0]), 'xyz'));

ptCloud = sense(robot, ROSobjects);

for i = 1:size(inspectionPoses, 3)
    moveto(robot, inspectionPoses(:, :, i), ROSobjects);
    pause(3)
    pcCurrent = sense(robot, ROSobjects);

    gridStep = 0.001;
    ptCloud = pcmerge(ptCloud, pcCurrent, gridStep);
end

msh = collisionMesh(ptCloud.Location);

end
