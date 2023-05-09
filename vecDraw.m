function result = vecDraw(q_start,pointVec,ur5)

    time = 0.3;
    steps = size(pointVec(:,1));
    g_start = ur5FwdKin(q_start);
    points = repmat(g_start, [1,1,steps]);
    
    q_op = zeros([6, steps]);
    
    disp("Moving Home");
    ur5.move_joints([0; -1.57; 0 ; -1.57; 0 ; 0], 5);
    pause(5);

    disp("VecDraw : Moving to start configuration");
    ur5.move_joints(q_start, 5);
    pause(5);
    disp("At Start Config");
    q_start
    disp("Entering Drawing Loop");
    for i = 1:steps

        points(1:2, 4, i) = points(1:2, 4, i) + pointVec(i,:)';
        
        q_i = ur5InvKin_wrap(points(:,:,i));
        [result, q_op_i, ind] = optimalJointConfig(ur5, q_i);
        q_op(:,i) = q_op_i;
        q_op_i
        
                     
    end
    
    ur5.move_joints(q_op, time);
    pause(time*steps);
    
    disp("Moving Home");
    ur5.move_joints([0; -1.57; 0 ; -1.57; 0 ; 0], 5);
    pause(5);

   result = 1;
   
end