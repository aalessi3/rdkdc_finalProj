function [result, error] = ur5InvKcontrol(q_start, q_goal, ur5, steps)
    
    %Function Hyper Parameters
    time = 1;
    
    %Obtain start and goal SE3
    g_goal = ur5FwdKin(q_goal);
    g_start = ur5FwdKin(q_start);
    
    %Interpolate in cart space
    points = interp(g_start, g_goal, steps);
    
    %Move to starting configuration
    disp("ur5InvKcontrl : Moving to start")
    ur5.move_joints(q_start, 5);
    pause(5);
    
    %Enter Control Loop
    disp("ur5InvKcontrol : Entering Control Loop")
    for i = 1:steps
        
        %Obtain ith joint config
        q = ur5InvKin_wrap(points(:,:,i));
        
        %Choose Optimal joint config
        [result, q_op, ind] = optimalJointConfig(ur5, q);
        
        
        %Check new pose for singularity
        Jb = ur5BodyJacobian(q_op);
        if(manipulability(Jb, 'invcond') < 0.01)
            disp(qk);
            error = -1;
            return
        end

        %Move to next position
        if (result == 1)
            ur5.move_joints(q_op, time);
            disp(ind);
        else 
            disp("No optimal Q found");
            error = -1;
            return
        end
        
        pause(time);

    end

    gst = ur5FwdKin(q_op);
    gtt = g_goal\gst;
    xik = getXi(gtt);
    error = norm(xik(1:3));
    result = 1;
    
end


