function error = ur5RRcontrol(q_start, q_goal, ur5, K)
    
    posThresh = .01;
    rotThresh = .02618;
    T = 0.07;
    steps = 20;
    
    disp("ur5RRcontrl : Moving to start configuration")
    ur5.move_joints(q_start, 5);
    pause(5);
    
    g_start = ur5FwdKin(q_start);
    g_goal = ur5FwdKin(q_goal);

    points = interp(g_start, g_goal, steps);

    
    for i = 1:steps

        g_goal = points(:,:,i);
        i
        qk = ur5.get_current_joints(); %Initial joint config of robot
        gst = ur5FwdKin(qk); %Initial pose of robot
        gtt = g_goal\gst; %Intitial error Same as inv(gdesire)*gst
        xik = getXi(gtt);
    
        disp("ur5RRControl : Entering Control Loop")
    
        while(norm(xik(1:3)) > posThresh || norm(xik(4:6)) > rotThresh)
    
            Jb = ur5BodyJacobian(qk);
    
            v = Jb'*xik;
    
            qv = K*T*Jb'*xik;
    
            [val, index] = max(abs(qv));
           
            K = (pi/2)/(abs(v(index))*T)/50; %Obtain max k : TODO find better solution
            K = min(35, K);
            qk_1 = qk-K*T*Jb'*xik;
    
            qk = qk_1;
    
            gst = ur5FwdKin(qk);
    
            gtt = g_goal\gst;
    
            xik = getXi(gtt);
            
            if(manipulability(Jb, 'invcond') < 0.01)
                disp(qk);
                error = -1;
                return
            end
    
              pause(0.1);
              ur5.move_joints(qk, 0.1);
        end
    
        error = norm(xik(1:3));
    
    end

    error = 1;

end

    
