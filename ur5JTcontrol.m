function error = ur5JTcontrol(gdesired, K, ur5)
    
    posThresh = .005;
    rotThresh = .02618;
    T = 0.01;
    
    qk = [pi/3 pi/3 pi/4 pi/3 pi/2  8*pi/9]'; %Move robot into non 
    ur5.move_joints(qk, 4);
    pause(4);
    qk = ur5.get_current_joints(); %Initial joint config of robot
    gst = ur5FwdKin(qk); %Initial pose of robot
    gtt = gdesired\gst; %Intitial error Same as inv(gdesire)*gst
    xik = getXi(gtt);

    while(norm(xik(1:3)) > posThresh || norm(xik(4:6)) > rotThresh)

        Jb = ur5BodyJacobian(qk);
        qk_1 = qk-K*T*Jb'*xik;
    
        v = Jb'*xik;
        qv = qk - qk_1;
        [val, index] = max(abs(qv));
%         disp(val);
        K = (pi/2)/(abs(v(index))*T)/50;
        qk_1 = qk-K*T*Jb'*xik;
        disp("Vel");
        disp(qv);

%         gst = ur5FwdKin(qk_1); %Initial pose of robot
%         gtt = gdesired\gst;
%         xik_1 = getXi(gtt);
% 
%         while(norm(xik_1) > norm(xik))
%             K = K/2;
%             qk_1 = qk-K*T*Jb'*xik;
%             gst = ur5FwdKin(qk_1); %Initial pose of robot
%             gtt = gdesired\gst;
%             xik_1 = getXi(gtt);
%         end

        qk = qk_1;



%         for i = 1:6
%             if(qk(i)>pi)
%                 qk(i) = qk(i) - pi;
%             end
%             if(qk(i)<-pi)
%                 qk(i) = qk(i) + pi;
%             end
%         end

        gst = ur5FwdKin(qk);
        gtt = gdesired\gst;
        xik = getXi(gtt);
        
        if(manipulability(Jb, 'invcond') < 0.01)
            disp(qk);
            error = -1;
            return
        end
          disp(qk);
          pause(0.2);
          ur5.move_joints(qk, 0.2);
    end

    error = norm(xik(1:3));





end
