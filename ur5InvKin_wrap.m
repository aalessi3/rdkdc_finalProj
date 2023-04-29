function [ theta ] = ur5InvKin_wrap( gd )
 dummy = [0;0;0];
t = [0;0;89.2/1000];
gd = [ROTZ(pi),-t;0 0 0 1]*gd*[ROTY(-pi/2),dummy;0 0 0 1]*[ROTX(pi/2),dummy;0 0 0 1];
theta = ur5InvKin(gd);
end
