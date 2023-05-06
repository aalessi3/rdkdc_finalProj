%test ur5fwdkin
theta = sym('theta',[6 1]);
ur5 = ur5_interface();

base = [-pi/3 -pi/3 0 pi/2 -pi/3 pi/6 ]';
B = ur5FwdKin(base);
back = ur5InvKin_wrap(B);
disp(back/pi);
disp(ur5FwdKin(base(:,1)));

ur5.move_joints(base,10);

pause(0.3);
Frame_Test = tf_frame('base_link', 'Frame_Test', B);
pause(0.3);