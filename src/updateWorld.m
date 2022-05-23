function pcWorld= updateWorld(robot, ROSobjects,pcCurrent,pcWorld)

gridStep = 0.001;

pcWorld= pcmerge(pcWorld, pcCurrent, gridStep);


end
