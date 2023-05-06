function gst = fwd(theta,L)
% Compute the forward kinematic map of the UR5. All necessary parameters (e.g.
% the base twists, gst0, etc) should be defined inside the function.

T1 = theta(1);
T2 = theta(2)+ pi/2;
T3 = theta(3);
T4 = theta(4)+ pi/2;
T5 = theta(5);
T6 = theta(6);


L1 = L(1);     %mm
L2 = L(2);
L3 = L(3);
L4 = L(4);
L5 = L(5);
L6 = L(6);   %89.2/1000
L7 = L(7);
L8 = L(8);

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

t = [0;0;L6];
dummy = [0;0;0];

ROT90Z = [0 -1 0 ;1 0 0; 0 0 1];
ROTm90X = [1 0 0; 0 0 1; 0 -1 0];
ROT90Y = [0 0 1; 0 1 0 ;-1 0 0];

gripper = [1 0 0 0;...
           0  1 0 L7;...
           0 0 1 L8;...
           0 0 0 1];    %not used now 
       
gst = [ROT90Z,t;0 0 0 1]*[u1, u2, u3, u4]*[ROTm90X,dummy;0 0 0 1]*[ROT90Y,dummy;0 0 0 1]*gripper;


% gst = [ROTZ(pi/2),t;0 0 0 1]*[u1, u2, u3, u4]*[ROTX(-pi/2),dummy;0 0 0 1]*[ROTY(pi/2),dummy;0 0 0 1];
% gst = [ROT180Z,dummy;0 0 0 1]*[u1, u2, u3, u4];
end


