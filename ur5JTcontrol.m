function error = ur5JTcontrol(q_start, q_goal, ur5, K)
    
    %Control Hyper Parameters
    posThresh = .01;
    rotThresh = .02618;
    T = 0.007;
    steps = 20;
    
    %Move to start configuration
    disp("ur5JTcontrl : Moving to start configuration");
    ur5.move_joints(q_start, 5);
    pause(5);
    
    %Generate start and end SE3
    g_start = ur5FwdKin(q_start);
    g_goal = ur5FwdKin(q_goal);
    
    %Obtain intermediate points to assure linear traj in cartegian space
    points = interp(g_start, g_goal, steps);
    
    for i = 1:steps

        %Obtain initial error for step i
        g_goal = points(:,:,i);
        qk = ur5.get_current_joints();
        gst = ur5FwdKin(qk);
        gtt = g_goal\gst; 
        xik = getXi(gtt);
    
        %Enter Control loop for step i
        disp("ur5JTControl : Entering Control Loop")
        while(norm(xik(1:3)) > posThresh || norm(xik(4:6)) > rotThresh)
    
            Jb = ur5BodyJacobian(qk);
    
            %Dynamically determine K such that max joint velocity is a constant
            v = Jb'*xik;
            qv = K*T*Jb'*xik;
            [val, index] = max(abs(qv));
            K = (pi/2)/(abs(v(index))*T)/50; %Obtain max k : TODO find better solution
            K = min(35, K);

            %Update Step
            qk_1 = qk-K*T*Jb'*xik;
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
            ur5.move_joints(qk, 0.01);
            pause(0.01);
        end
    
    end
    
    %Return Final error
    error = norm(xik(1:3));

end

    
