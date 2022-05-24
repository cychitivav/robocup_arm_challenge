function activateGripper(state, ROSobjects)
    
    pause(3)

    gripClient = ROSobjects.gripAction.client;
    gripGoal = ROSobjects.gripAction.goalMsg;
        
    


    
    if strcmp(state,'open')  
        pos = 0.0;
    elseif strcmp(state,'close')
        pos = 0.044;
    else
        warning('Gripper action incorrect, available options [open,close]')
        return
    end

    gripperCommand = rosmessage('control_msgs/GripperCommand');
    gripperCommand.Position = pos;
    gripperCommand.MaxEffort = 500;
    
    gripGoal.Command = gripperCommand;

    sendGoalAndWait(gripClient,gripGoal);
end