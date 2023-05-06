function [ theta ] = ur5InvKin_wrap( gd )
dummy = [0;0;0];
t = [0;0;89.2/1000];
gripper = [1 0 0 0;...
           0  1 0 -49/1000;...
           0 0 1 122.28/1000;...
           0 0 0 1];    %not used now 
gd = [ROTZ(pi/2),-t;0 0 0 1]*gd*inv(gripper)*[ROTY(-pi/2),dummy;0 0 0 1]*[ROTX(pi/2),dummy;0 0 0 1];
theta = ur5InvKin(gd);
end
