function gst = ur5FwdKin(theta)
% Compute the forward kinematic map of the UR5. All necessary parameters (e.g.
% the base twists, gst0, etc) should be defined inside the function.

T1 = theta(1);
T2 = theta(2);
% T2 = theta(2) + pi/2;
T3 = theta(3);
% T4 = theta(4) + pi/2;
T4 = theta(4);
T5 = theta(5);
T6 = theta(6);

ratio = 1000;
L1 = 425/ratio;     %mm
L2 = 392/ratio;
L3 = 109.3/ratio;
L4 = 94.75/ratio;
L5 = 82.5/ratio;


u1 = [cos(T1)*cos(T5)-cos(T2+T3+T4)*sin(T1)*sin(T5);...
cos(T5)*sin(T1)+cos(T1)*cos(T2+T3+T4)*sin(T5);...
sin(T2+T3+T4)*sin(T5);...
0];

u2 = [-cos(T6)*(cos(T2+T3+T4)*cos(T5)*sin(T1)+cos(T1)*sin(T5))+sin(T1)*sin(T2+T3+T4)*sin(T6);...
-cos(T6)*sin(T1)*sin(T5)+cos(T1)*(cos(T2+T3+T4)*cos(T5)*cos(T6)-sin(T2+T3+T4)*sin(T6));...
cos(T5)*cos(T6)*sin(T2+T3+T4)+cos(T2+T3+T4)*sin(T6);...
0];

u3 = [cos(T6)*sin(T1)*sin(T2+T3+T4)+(cos(T2+T3+T4)*cos(T5)*sin(T1)+cos(T1)*sin(T5))*sin(T6);...
sin(T1)*sin(T5)*sin(T6)-cos(T1)*(cos(T6)*sin(T2+T3+T4)+cos(T2+T3+T4)*cos(T5)*sin(T6));...
cos(T2+T3+T4)*cos(T6)-cos(T5)*sin(T2+T3+T4)*sin(T6);...
0];

u4 = [cos(T1)*(L3+L5*cos(T5))+sin(T1)*(L1*sin(T2)+L2*sin(T2+T3)+L4*sin(T2+T3+T4)-L5*cos(T2+T3+T4)*sin(T5));...
(L3+L5*cos(T5))*sin(T1)-cos(T1)*(L1*sin(T2)+L2*sin(T2+T3)+L4*sin(T2+T3+T4)-L5*cos(T2+T3+T4)*sin(T5));...
L1*cos(T2)+L2*cos(T2+T3)+L4*cos(T2+T3+T4)+L5*sin(T2+T3+T4)*sin(T5);...
1];

t = [0;0;89.2/1000];
 dummy = [0;0;0];
gst = [ROTZ(0),t;0 0 0 1]*[u1, u2, u3, u4]*[ROTX(-pi/2),dummy;0 0 0 1]*[ROTY(pi/2),dummy;0 0 0 1];
% gst = [ROTZ(pi),dummy;0 0 0 1]*[u1, u2, u3, u4];
% gst = [u1, u2, u3, u4];
end

