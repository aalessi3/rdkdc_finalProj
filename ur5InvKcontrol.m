function result = ur5InvKcontrol(q_start, q_goal, ur5, steps)
    
    time = 0.1;
%     q_start = ur5InvKin(g_start);

    g_goal = ur5FwdKin(q_goal);

    g_start = ur5FwdKin(q_start);
       
    points = interp(g_start, g_goal, steps);
    
    disp("ur5InvKcontrl : Moving to start")

    ur5.move_joints(q_start, 5);
    pause(5);
    
    g = g_start(); %Grab inital config to change
    
    disp("ur5InvKcontrol : Entering Control Loop")
    
    for i = 1:steps

%         disp("points_1")
%         points(:,:,1)
%         disp("G_start")
%         g_start
%         disp("Back from Inverse")
%         ur5InvKin_wrap(g_start)
%         disp("Current Joint Stares : Read");
%         ur5.get_current_joints()

        q = ur5InvKin_wrap(points(:,:,i));
%         disp("all Q");
%         q
        [result, q_op, ind] = optimalJointConfig(ur5, q);
%         disp("q_op")
%         q_op
% 
%         disp("q_start")
%         q_start
        
        if (result == 1)
            ur5.move_joints(q_op, time);
            disp(ind);
        else 
            disp("No optimal Q found");
            
        end
        

        pause(time);
    end

    result = 1;
end


