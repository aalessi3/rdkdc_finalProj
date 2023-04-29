function error = ur5RRcontrol(gdesired, K, ur5)
    
    posThresh = .005;
    rotThresh = .02618;
    T = 0.01;
    
    qk = [pi/3 pi/3 pi/4 pi/3 pi/2 pi]'; %Move robot into non 
    ur5.move_joints(qk, 4);
    pause(4);
    qk = ur5.get_current_joints(); %Initial joint config of robot
    gst = ur5FwdKin(qk); %Initial pose of robot
    gtt = gdesired\gst; %Intitial error Same as inv(gdesire)*gst
    xik = getXi(gtt);

    while(norm(xik(1:3)) > posThresh || norm(xik(4:6)) > rotThresh)

        Jb = ur5BodyJacobian(qk);
        qk_1 = qk-K*T*inv(Jb)*xik;
        qk = qk_1;

        gst = ur5FwdKin(qk);
        gtt = gdesired\gst;
        xik = getXi(gtt);

        if(manipulability(Jb, 'invcond') < 0.01)
            disp(qk);
            error = -1;
            return
        end

          pause(.05);
          ur5.move_joints(qk, .2);
    end

    error = norm(xik(1:3));





end
