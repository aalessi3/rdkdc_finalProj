% ur5 = ur5_interface();
% disp("Moving to example config 1")
% base = [0 -pi/2 0 -pi/2 0 0 ]';
% ur5.move_joints(base,5);
% 
% B = ur5FwdKin(base);
% pause(0.3);
% Frame_Test = tf_frame('base_link', 'Frame_Test', B);
% disp( B)
% pause(5);
% 
% disp("Moving to example config 2")
% base = [0 -pi/2 pi/3 -pi/2 pi/5 pi/4 ]';
% ur5.move_joints(base,5);
% 
% B = ur5FwdKin(base);
% pause(0.3);
% Frame_Test = tf_frame('base_link', 'Frame_Test', B);


disp("Testing Resolved Rate Controller")

K = 1;
ur5 = ur5_interface();
R0a = EULERXYZ([-pi/5, pi/3, -pi/6]);
ratio = 5;
T0a = [2;2;1]/ratio;
g0a = [R0a, T0a;...
    0 0 0 1];
Frame_A = tf_frame('base_link', 'Frame_A', g0a);


disp("goa--------------------")
disp(g0a)

q_goal = ur5InvKin_wrap(g0a);
q_goal = q_goal(:,4);

disp(q_goal)
g_goal = ur5FwdKin(q_goal);
ur5.move_joints([0;0;0;0;0;0],5);
disp("g_goal--------------------")
disp(g_goal)


ur5_interface.get_current_joints()
% ur5_interface.get_current_transformation()
% 
% urdisp("Sending Non Singulare Goal");
% ur5RRcontrol(g_goal, K, ur5)
% 
% q_goal = [pi/4 pi/5 0 pi/6 pi/3 pi/9]';
% g_goal = ur5FwdKin(q_goal);
% disp("Sending Singular Goal");
% ur5RRcontrol(g_goal, K, ur5)