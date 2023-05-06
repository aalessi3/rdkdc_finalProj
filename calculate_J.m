%test ur5fwdkin
theta = sym('theta',[6 1]);
L = sym('L',[8 1]);
base = [pi/4 0 -pi/3 0 -pi/3 0 ]';

Test = simplify(fwd(theta,L));

ur5BodyJacobian(base)
invT = inv(Test);
for i = 1:6
    gdot = diff(Test,theta(i));
    u1_m = simplify(invT*gdot);
    u1 = [u1_m(1,4);u1_m(2,4);u1_m(3,4);-u1_m(2,3);u1_m(1,3);-u1_m(1,2);];
    disp(u1)
end

