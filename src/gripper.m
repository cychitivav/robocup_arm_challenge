function status = gripper(action)
    if strcmp(action,'close')
        pos = 0.04;
    elseif strcmp(action,'open')
        pos = 0;
    else
        warning('Gripper action incorrect, available options [open,close]')
        return
    end
    
    [gripClient,gripGoal] = rosactionclient('/my_gen3/custom_gripper_controller/gripper_cmd');

    gripperCommand = rosmessage('control_msgs/GripperCommand');
    gripperCommand.Position = pos;
    gripperCommand.MaxEffort = 500;
    
    gripGoal.Command = gripperCommand;

    waitForServer(gripClient);
    result = sendGoalAndWait(gripClient,gripGoal);
    
    status = result.ReachedGoal;
    gripClient.delete;
end