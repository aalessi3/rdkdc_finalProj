% ur5 = ur5_interface();
% disp("Moving to example config 1")
% base = [0 -pi/2 0 -pi/2 0 0 ]';
% ur5.move_joints(base,5);
% 
% B = ur5FwdKin(base);
% pause(0.3);
% Frame_Test = tf_frame('base_link', 'Frame_Test', B);
% 
% pause(5);
% 
% disp("Moving to example config 2")
% base = [0 -pi/2 pi/3 -pi/2 pi/5 pi/4 ]';
% ur5.move_joints(base,5);
% 
% B = ur5FwdKin(base);
% pause(0.3);
% Frame_Test = tf_frame('base_link', 'Frame_Test', B);
% 
% pause(5);

disp("Testing Jacobian and Printing Error");


q = [pi/2 0 pi pi/2 7 0]';
g = ur5FwdKin(q);

Jb = ur5BodyJacobian(q);

e1 = [1; 0; 0; 0; 0; 0];
e2 = [0; 1; 0; 0; 0; 0];
e3 = [0; 0; 1; 0; 0; 0];
e4 = [0; 0; 0; 1; 0; 0];
e5 = [0; 0; 0; 0; 1; 0];
e6 = [0; 0; 0; 0; 0; 1];

ep = 0.0001;

dg_q1 = (1/(2*ep))*(ur5FwdKin(q+ep*e1)-ur5FwdKin(q-ep*e1));
dg_q2 = (1/(2*ep))*(ur5FwdKin(q+ep*e2)-ur5FwdKin(q-ep*e2));
dg_q3 = (1/(2*ep))*(ur5FwdKin(q+ep*e3)-ur5FwdKin(q-ep*e3));
dg_q4 = (1/(2*ep))*(ur5FwdKin(q+ep*e4)-ur5FwdKin(q-ep*e4));
dg_q5 = (1/(2*ep))*(ur5FwdKin(q+ep*e5)-ur5FwdKin(q-ep*e5));
dg_q6 = (1/(2*ep))*(ur5FwdKin(q+ep*e6)-ur5FwdKin(q-ep*e6));

xi1 = Skew2Vec(inv(g)*dg_q1)';
xi2 = Skew2Vec(inv(g)*dg_q2)';
xi3 = Skew2Vec(inv(g)*dg_q3)';
xi4 = Skew2Vec(inv(g)*dg_q4)';
xi5 = Skew2Vec(inv(g)*dg_q5)';
xi6 = Skew2Vec(inv(g)*dg_q6)';

Japprox = [xi1 xi2 xi3 xi4 xi5 xi6];

norm(Japprox-Jb)

% 
% disp("Generating manipulability figures");
% 
% interval = pi/360;
% q = [pi/4 pi/5 pi/3 pi/7 pi/3 pi/9]';
% k = 0;
% sigmamin = [];
% invcond = [];
% detjac = [];
% range = -pi/4:interval:pi/4;
% for i = range
%     q(3) = i;
%     J = ur5BodyJacobian(q);
%     k = k + 1;
%     sigmamin(k) = manipulability(J,"sigmamin");
%     invcond(k) = manipulability(J,"invcond");
%     detjac(k) = manipulability(J,"detjac");
%     disp(det(J))
% end
% figure
% plot(range, sigmamin)
% legend('sigmamin')
% figure
% plot(range, invcond)
% legend('invcond')
% figure
% plot(range, detjac)
% legend('detjac')
% 
% % ----------------------------------------------
% interval = pi/360;
% % q = [-pi/4 -pi/5 pi/6 pi/7 -pi/8 pi/4]';
% q = [pi/4 pi/5 pi/3 pi/7 pi/3 pi/9]';
% k = 0;
% sigmamin = [];
% invcond = [];
% detjac = [];
% range = -pi/4:interval:pi*5/4;
% for i = range
%     q(5) = i;
%     J = ur5BodyJacobian(q);
%     k = k + 1;
%     sigmamin(k) = manipulability(J,"sigmamin");
%      invcond(k) = manipulability(J,"invcond");
%     detjac(k) = manipulability(J,"detjac");
%     disp(det(J))
% end
% figure
% plot(range, sigmamin)
% legend('sigmamin')
% figure
% plot(range, invcond)
% legend('invcond')
% figure
% plot(range, detjac)
% legend('detjac')
% 
% interval = pi/360;
% q = [pi/4 pi/5 pi/3 pi/7 pi/3 pi/9]';
% k = 0;
% sigmamin = [];
% invcond = [];
% detjac = [];
% range = -pi/4:interval:pi/4;
% for i = range
%     q(3) = i;
%     J = ur5BodyJacobian(q);
%     k = k + 1;
%     sigmamin(k) = manipulability(J,"sigmamin");
%     invcond(k) = manipulability(J,"invcond");
%     detjac(k) = manipulability(J,"detjac");
%     disp(det(J))
% end
% figure
% plot(range, sigmamin)
% legend('sigmamin')
% figure
% plot(range, invcond)
% legend('invcond')
% figure
% plot(range, detjac)
% legend('detjac')
% 
% % ----------------------------------------------
% interval = pi/360;
% % q = [-pi/4 -pi/5 pi/6 pi/7 -pi/8 pi/4]';
% q = [pi/4 pi/5 pi/3 pi/7 pi/3 pi/9]';
% k = 0;
% sigmamin = [];
% invcond = [];
% detjac = [];
% range = -pi/4:interval:pi*5/4;
% for i = range
%     q(5) = i;
%     J = ur5BodyJacobian(q);
%     k = k + 1;
%     sigmamin(k) = manipulability(J,"sigmamin");
%     invcond(k) = manipulability(J,"invcond");
%     detjac(k) = manipulability(J,"detjac");
%     disp(det(J))
% end
% figure
% plot(range, sigmamin)
% legend('sigmamin')
% figure
% plot(range, invcond)
% legend('invcond')
% figure
% plot(range, detjac)
% legend('detjac')
% 
% disp("Generating test Xi vectors and diplaying error norms");
% q1 = [0 0 0 pi/2 0 0];
% q2 = [0 pi/2 0 0 pi/3 0];
% q3 = [0 0 pi/4 0 0 pi/2];
% q4 = [1 1.57 3.14 1 3 2.3];
% 
% g1 = ur5FwdKin(q1);
% g2 = ur5FwdKin(q2);
% g3 = ur5FwdKin(q3);
% g4 = ur5FwdKin(q4);
% 
% xi1 = getXi(g1);
% xi2 = getXi(g2);
% xi3 = getXi(g3);
% xi4 = getXi(g4);
% 
% g1_p = expm(Vec2Skew(xi1));
% g2_p = expm(Vec2Skew(xi2));
% g3_p = expm(Vec2Skew(xi3));
% g4_p = expm(Vec2Skew(xi4));
% 
% 
% error1 = norm(g1-g1_p)
% error2 = norm(g2-g2_p)
% error3 = norm(g3-g3_p)
% error4 = norm(g4-g4_p)


disp("Testing Resolved Rate Controller")
K = 1;
q_goal = [pi/3 pi/5 pi/4 pi/6 pi/3 pi/7]';
g_goal = ur5FwdKin(q_goal);
ur5 = ur5_interface();
pause(0.5)
Frame_Test = tf_frame('base_link', 'Frame_goal',g_goal);

disp("Sending Non Singulare Goal");

ur5JTcontrol(g_goal, K, ur5);

% q_goal = [pi/4 pi/5 pi/3 pi/6 pi/3 pi/9]';

% g_goal = ur5FwdKin(q_goal);
% disp("Sending Singular Goal");
% ur5RRcontrol(g_goal, K, ur5)