function error = ur5JTcontrol(g_start, g_goal, ur5, K)
    
    posThresh = .005;
    rotThresh = .02618;
    T = 0.01;
    
    disp("ur5JTControl : Moving to Start Confiuration")

    q_start = ur5InvKin_wrap(g_start); %Move robot into non 
    ur5.move_joints(q_start, 5);
    pause(5);

    qk = ur5.get_current_joints(); %Initial joint config of robot
    gst = ur5FwdKin(qk); %Initial pose of robot
    gtt = g_goal\gst; %Intitial error Same as inv(gdesire)*gst
    xik = getXi(gtt);

    disp("ur5JTControl : Entering Control Loop")

    while(norm(xik(1:3)) > posThresh || norm(xik(4:6)) > rotThresh)

        Jb = ur5BodyJacobian(qk);
    
        v = Jb'*xik;

        qv = K*T*Jb'*xik;

        [val, index] = max(abs(qv));

        K = (pi/2)/(abs(v(index))*T)/50; %TODO Fix me for oscillations

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

          pause(0.2);
          ur5.move_joints(qk, 0.2);
    end

    error = norm(xik(1:3));





end