% 
% disp("Testing Jacobian and Printing Error");
% 
% 
% q = [pi/2 0 pi pi/2 7 0]';
% g = ur5FwdKin(q);
% 
% Jb = ur5BodyJacobian(q);
% 
% e1 = [1; 0; 0; 0; 0; 0];
% e2 = [0; 1; 0; 0; 0; 0];
% e3 = [0; 0; 1; 0; 0; 0];
% e4 = [0; 0; 0; 1; 0; 0];
% e5 = [0; 0; 0; 0; 1; 0];
% e6 = [0; 0; 0; 0; 0; 1];
% 
% ep = 0.0001;
% 
% dg_q1 = (1/(2*ep))*(ur5FwdKin(q+ep*e1)-ur5FwdKin(q-ep*e1));
% dg_q2 = (1/(2*ep))*(ur5FwdKin(q+ep*e2)-ur5FwdKin(q-ep*e2));
% dg_q3 = (1/(2*ep))*(ur5FwdKin(q+ep*e3)-ur5FwdKin(q-ep*e3));
% dg_q4 = (1/(2*ep))*(ur5FwdKin(q+ep*e4)-ur5FwdKin(q-ep*e4));
% dg_q5 = (1/(2*ep))*(ur5FwdKin(q+ep*e5)-ur5FwdKin(q-ep*e5));
% dg_q6 = (1/(2*ep))*(ur5FwdKin(q+ep*e6)-ur5FwdKin(q-ep*e6));
% 
% xi1 = Skew2Vec(inv(g)*dg_q1)';
% xi2 = Skew2Vec(inv(g)*dg_q2)';
% xi3 = Skew2Vec(inv(g)*dg_q3)';
% xi4 = Skew2Vec(inv(g)*dg_q4)';
% xi5 = Skew2Vec(inv(g)*dg_q5)';
% xi6 = Skew2Vec(inv(g)*dg_q6)';
% 
% Japprox = [xi1 xi2 xi3 xi4 xi5 xi6];
% 
% norm(Japprox-Jb)

ur5 = ur5_interface();

INV = true;
RR = false;
JT = false;


if RR

    disp("Testing Resolved Rate Controller")
    
    K = 1;

    q_start = [0 0 0 0 0 0]';
    g_start = ur5FwdKinz(q_start);

    q_goal = [pi/3 pi/5 pi/4 pi/6 pi/3 pi/7]';
    g_goal = ur5FwdKin(q_goal);

    pause(0.5)

    Frame_Test = tf_frame('base_link', 'Frame_goal',g_goal);

    ur5RRcontrol(g_start, g_goal, ur5, K)
end

if JT

    disp("Testing Jacobian Controller")
    
    K = 1;

    q_start = [0 0 0 0 0 0]';
    g_start = ur5FwdKinz(q_start);

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
    
    g_final = [0 -1 0 -0.3; -1 0 0 0.39; 0 0 -1 0.22; 0 0 0 1]*[ROTX(pi/2),t2;0 0 0 1]*[ROTZ(pi/2),t; 0 0 0 1];
    
    ur5InvKcontrol(g_start, g_final, ur5, 100);


end
