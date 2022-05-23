function activateGripper(state)
    
    persistent gripAct gripGoal gripperCommand
    
    if isempty(gripAct) || ~isvalid(gripAct)
        [gripAct,gripGoal] = rosactionclient('/my_gen3/custom_gripper_controller/gripper_cmd');
        gripperCommand = rosmessage('control_msgs/GripperCommand');
    end
    
    
    if strcmp(state,'open')  
        pos = 0.0;
    elseif strcmp(state,'close')
        pos = 0.04;
    else
        warning('Gripper action incorrect, available options [open,close]')
        return
    end

    gripperCommand.Position = pos;
    gripperCommand.MaxEffort = 1000;
    gripGoal.Command = gripperCommand;

    sendGoalAndWait(gripAct,gripGoal);
end