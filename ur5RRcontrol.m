function error = ur5RRcontrol(q_start, q_goal, ur5, K)
    
    %Function Hyper Parameters
    posThresh = .005;
    rotThresh = .02618;
    T = 0.01;
    
    %Move to start configuration
    disp("ur5RRcontrl : Moving to start configuration")
    ur5.move_joints(q_start, 5);
    pause(5);
    
    %Obtain Goal Pose and Initial Error
    g_goal = ur5FwdKin(q_goal);
    qk = ur5.get_current_joints();
    gst = ur5FwdKin(qk);
    gtt = g_goal\gst; 
    xik = getXi(gtt);
    
    %Resolved Rate Control Loop
    disp("ur5RRControl : Entering Control Loop")
    while(norm(xik(1:3)) > posThresh || norm(xik(4:6)) > rotThresh)

        Jb = ur5BodyJacobian(qk);

        %Dynamically determine K such that max joint velocity is a constant
        v = inv(Jb)*xik;
        qv = K*T*inv(Jb)*xik;
        [val, index] = max(abs(qv));
        K = (pi/2)/(abs(v(index))*T)/50;
        K = min(50, K);

        %Update Step
        qk_1 = qk-K*T*inv(Jb)*xik;
        qk = qk_1;
        
        %Obtain New Error
        gst = ur5FwdKin(qk);
        gtt = g_goal\gst;
        xik = getXi(gtt);
        
        %Check new pose for singularity
        if(manipulability(Jb, 'invcond') < 0.01)
            disp(qk);
            error = -1;
            return
        end

        %Move to new position
        ur5.move_joints(qk, 0.5);
        pause(0.5);
    end

    %Return Final error
    error = norm(xik(1:3));

end

    
