time = 5;
steps = 100;

ur5 = ur5_interface();

q_init = [0 0 0 0 0 0]';
t = [0 0 0]';
t2 = [0;0;89.2/1000];
g_start = [0 -1 0 0.3; -1 0 0 -0.4; 0 0 -1 0.22; 0 0 0 1];
g_start_p = g_start*[ROTX(pi/2),t2;0 0 0 1]*[ROTZ(pi/2),t; 0 0 0 1];
q_start = ur5InvKin(g_start_p);
g = g_start_p();

g_final = [0 -1 0 -0.3; -1 0 0 0.39; 0 0 -1 0.22; 0 0 0 1];
g_final_p = g_final*[ROTX(pi/2),t2;0 0 0 1]*[ROTZ(pi/2),t; 0 0 0 1];
q_final = ur5InvKin(g_final_p);

points = interp(g_start_p, g_final_p, steps);

ur5.move_joints(q_init(:,1), time);
pause(time);

ur5.move_joints(q_start(:,1), time);
pause(time);

for i = 1:steps
    g(1:3, 4) = points(:, i);
    q = ur5InvKin(g);
    ur5.move_joints(q(:,1), .3);
    pause(.3);
end



% ur5.move_joints(q_final(:,1), time);
% pause(time);

