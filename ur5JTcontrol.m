function error = ur5JTcontrol(q_start, q_goal, ur5, K)
    
    posThresh = .01;
    rotThresh = .02618;
    T = 0.01;
    
    disp("ur5JTControl : Moving to Start Confiuration")
    %Move robot into non 
    ur5.move_joints(q_start, 5);
    pause(5);

    g_goal = ur5FwdKin(q_goal);
    qk = ur5.get_current_joints(); %Initial joint config of robot
    gst = ur5FwdKin(qk); %Initial pose of robot
    gtt = g_goal\gst; %Intitial error Same as inv(gdesire)*gst
    xik = getXi(gtt);

    Jb = ur5BodyJacobian(qk);
    
    v_old = Jb'*xik;

    a_old = xik;

    disp("ur5JTControl : Entering Control Loop")

    while(norm(xik(1:3)) > posThresh || norm(xik(4:6)) > rotThresh)

        Jb = ur5BodyJacobian(qk);
    
        v = Jb'*xik;

        qv = K*T*Jb'*xik;

        [val, index] = max(abs(qv));

        K = (pi/2)/(abs(v(index))*T)/50; %TODO Fix me for oscillations
%         disp(K)
        K = min(40,K);
        
        qk_1 = qk-K*T*Jb'*xik;

        gk_1 = ur5FwdKin(qk_1);

        gtt_prop = g_goal\gk_1;
        xik_prop = getXi(gtt_prop);
        disp("error")
        norm(xik_prop(1:3))

        qk = qk_1;

        gst = ur5FwdKin(qk);

        gtt = g_goal\gst;

        xik = getXi(gtt);
        
        if(manipulability(Jb, 'invcond') < 0.0001)
            disp("singularity");
            disp(qk/pi);
            error = -1;
            return
        end

          pause(0.03);
          ur5.move_joints(qk, 0.03);
          
    end

    error = norm(xik(1:3));





end
