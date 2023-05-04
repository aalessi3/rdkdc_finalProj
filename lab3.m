
ur5 = ur5_interface();

INV = false;
RR = true;
JT = false;


if RR

    disp("Testing Resolved Rate Controller")
    
    K = 1;

    q_start = [pi/3 pi/3 pi/2 pi/4 pi/7 pi/9]';
    g_start = ur5FwdKin(q_start);

    q_goal = [pi/3 pi/5 pi/4 pi/6 pi/3 pi/7]';
    g_goal = ur5FwdKin(q_goal);

    points = interp(g_start, g_final, 100);

    pause(0.5)

    Frame_Test = tf_frame('base_link', 'Frame_goal',g_goal);

    ur5RRcontrol(q_start, q_goal, ur5, K)
end

if JT

    disp("Testing Jacobian Controller")
    
    K = 1;

    q_start = [pi/3 pi/3 pi/2 pi/4 pi/7 pi/9]';
    g_start = ur5FwdKin(q_start);

    q_goal = [pi/3 pi/5 pi/4 pi/6 pi/3 pi/7]';
    g_goal = ur5FwdKin(q_goal);

    pause(0.5)

    Frame_Test = tf_frame('base_link', 'Frame_goal',g_goal);

    ur5JTcontrol(g_start, g_goal, ur5, K)

end

if INV
    
    disp("Testing InvKin Controll")
    
    t = [0 0 0]';
    
    t2 = [0;0;89.2/1000];
    
    g_start = [0 -1 0 0.3; -1 0 0 -0.4; 0 0 -1 0.22; 0 0 0 1]*[ROTX(pi/2),t2;0 0 0 1]*[ROTZ(pi/2),t; 0 0 0 1];
    
    g_final = [0 -1 0 -0.3; -1 0 0 -0.2; 0 0 -1 0.22; 0 0 0 1]*[ROTX(pi/2),t2;0 0 0 1]*[ROTZ(pi/2),t; 0 0 0 1];
    
    ur5InvKcontrol(g_start, g_final, ur5, 100);


end
