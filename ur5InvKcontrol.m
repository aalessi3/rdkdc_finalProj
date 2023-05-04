function result = ur5InvKcontrol(q_start, q_goal, ur5, steps)
    
%     q_start = ur5InvKin(g_start);

    g_goal = ur5FwdKin(q_goal);
       
    points = interp(g_start, g_goal, steps);
    
    disp("ur5InvKcontrl : Moving to start")

    ur5.move_joints(q_start(:,1), 5);
    pause(5);
    
    g = g_start(); %Grab inital config to change
    
    disp("ur5InvKcontrol : Entering Control Loop")
    
    for i = 1:steps

        q = ur5InvKin(points(:,:,i));
%         result = optimalJointConfig(ur5, q);

        ur5.move_joints(q(:,1), .3);

        pause(.3);
    end

    result = 1;
end


