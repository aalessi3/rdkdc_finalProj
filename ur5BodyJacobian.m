function J = ur5BodyJacobian(theta)
% Compute the Jacobian matrix for the UR5. All necessary parameters are to be
% defined inside the function. Again, parameters such as twists and g st (0) (if needed) should
% be defined in the function.
theta1 = theta(1);
theta2 = theta(2);
theta3 = theta(3);
theta4 = theta(4);
theta5 = theta(5);
theta6 = theta(6);

ratio = 1000;
L1 = 425/ratio;     %mm
L2 = 392/ratio;
L3 = 109.3/ratio;
L4 = 94.75/ratio;
L5 = 82.5/ratio;

u1 = [ 

                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              (L2*sin(theta2 + theta3 - theta5))/2 + (L3*sin(theta2 + theta3 + theta4 + theta5))/2 + (L4*sin(theta2 + theta3 + theta4 + theta5))/2 + (L1*sin(theta2 + theta5))/2 - (L3*sin(theta2 + theta3 + theta4 - theta5))/2 + (L4*sin(theta2 + theta3 + theta4 - theta5))/2 + (L1*sin(theta2 - theta5))/2 + (L2*sin(theta2 + theta3 + theta5))/2;...
 L5*cos(theta2)*cos(theta3)*cos(theta4)*cos(theta6) - L1*cos(theta6)*sin(theta2)*sin(theta5) - L2*cos(theta2)*cos(theta6)*sin(theta3)*sin(theta5) - L2*cos(theta3)*cos(theta6)*sin(theta2)*sin(theta5) - L3*cos(theta2)*cos(theta3)*sin(theta4)*sin(theta6) - L3*cos(theta2)*cos(theta4)*sin(theta3)*sin(theta6) - L3*cos(theta3)*cos(theta4)*sin(theta2)*sin(theta6) - L5*cos(theta2)*cos(theta6)*sin(theta3)*sin(theta4) - L5*cos(theta3)*cos(theta6)*sin(theta2)*sin(theta4) - L5*cos(theta4)*cos(theta6)*sin(theta2)*sin(theta3) + L3*sin(theta2)*sin(theta3)*sin(theta4)*sin(theta6) - L3*cos(theta2)*cos(theta5)*cos(theta6)*sin(theta3)*sin(theta4) - L3*cos(theta3)*cos(theta5)*cos(theta6)*sin(theta2)*sin(theta4) - L3*cos(theta4)*cos(theta5)*cos(theta6)*sin(theta2)*sin(theta3) - L4*cos(theta2)*cos(theta3)*cos(theta6)*sin(theta4)*sin(theta5) - L4*cos(theta2)*cos(theta4)*cos(theta6)*sin(theta3)*sin(theta5) - L4*cos(theta3)*cos(theta4)*cos(theta6)*sin(theta2)*sin(theta5) - L5*cos(theta2)*cos(theta3)*cos(theta5)*sin(theta4)*sin(theta6) - L5*cos(theta2)*cos(theta4)*cos(theta5)*sin(theta3)*sin(theta6) - L5*cos(theta3)*cos(theta4)*cos(theta5)*sin(theta2)*sin(theta6) + L4*cos(theta6)*sin(theta2)*sin(theta3)*sin(theta4)*sin(theta5) + L5*cos(theta5)*sin(theta2)*sin(theta3)*sin(theta4)*sin(theta6) + L3*cos(theta2)*cos(theta3)*cos(theta4)*cos(theta5)*cos(theta6);...
 L1*sin(theta2)*sin(theta5)*sin(theta6) - L3*cos(theta2)*cos(theta3)*cos(theta6)*sin(theta4) - L3*cos(theta2)*cos(theta4)*cos(theta6)*sin(theta3) - L3*cos(theta3)*cos(theta4)*cos(theta6)*sin(theta2) - L5*cos(theta2)*cos(theta3)*cos(theta4)*sin(theta6) + L2*cos(theta2)*sin(theta3)*sin(theta5)*sin(theta6) + L2*cos(theta3)*sin(theta2)*sin(theta5)*sin(theta6) + L3*cos(theta6)*sin(theta2)*sin(theta3)*sin(theta4) + L5*cos(theta2)*sin(theta3)*sin(theta4)*sin(theta6) + L5*cos(theta3)*sin(theta2)*sin(theta4)*sin(theta6) + L5*cos(theta4)*sin(theta2)*sin(theta3)*sin(theta6) + L3*cos(theta2)*cos(theta5)*sin(theta3)*sin(theta4)*sin(theta6) + L3*cos(theta3)*cos(theta5)*sin(theta2)*sin(theta4)*sin(theta6) + L3*cos(theta4)*cos(theta5)*sin(theta2)*sin(theta3)*sin(theta6) + L4*cos(theta2)*cos(theta3)*sin(theta4)*sin(theta5)*sin(theta6) + L4*cos(theta2)*cos(theta4)*sin(theta3)*sin(theta5)*sin(theta6) + L4*cos(theta3)*cos(theta4)*sin(theta2)*sin(theta5)*sin(theta6) + L5*cos(theta5)*cos(theta6)*sin(theta2)*sin(theta3)*sin(theta4) - L4*sin(theta2)*sin(theta3)*sin(theta4)*sin(theta5)*sin(theta6) - L3*cos(theta2)*cos(theta3)*cos(theta4)*cos(theta5)*sin(theta6) - L5*cos(theta2)*cos(theta3)*cos(theta5)*cos(theta6)*sin(theta4) - L5*cos(theta2)*cos(theta4)*cos(theta5)*cos(theta6)*sin(theta3) - L5*cos(theta3)*cos(theta4)*cos(theta5)*cos(theta6)*sin(theta2);...
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  cos(theta2 + theta3 + theta4 - theta5)/2 - cos(theta2 + theta3 + theta4 + theta5)/2;...
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    cos(theta2 + theta3 + theta4)*sin(theta6) + sin(theta2 + theta3 + theta4)*cos(theta5)*cos(theta6);...
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    cos(theta2 + theta3 + theta4)*cos(theta6) - sin(theta2 + theta3 + theta4)*cos(theta5)*sin(theta6)];                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         
% 

u2 = [
                                                                                                                                                                                                                                                              -sin(theta5)*(L4 + L1*cos(theta3 + theta4) + L2*cos(theta4));...
 L2*sin(theta4)*sin(theta6) - L4*cos(theta5)*cos(theta6) + L5*sin(theta5)*sin(theta6) - L2*cos(theta4)*cos(theta5)*cos(theta6) + L1*cos(theta3)*sin(theta4)*sin(theta6) + L1*cos(theta4)*sin(theta3)*sin(theta6) - L1*cos(theta3)*cos(theta4)*cos(theta5)*cos(theta6) + L1*cos(theta5)*cos(theta6)*sin(theta3)*sin(theta4);...
 L2*cos(theta6)*sin(theta4) + L4*cos(theta5)*sin(theta6) + L5*cos(theta6)*sin(theta5) + L1*cos(theta3)*cos(theta6)*sin(theta4) + L1*cos(theta4)*cos(theta6)*sin(theta3) + L2*cos(theta4)*cos(theta5)*sin(theta6) + L1*cos(theta3)*cos(theta4)*cos(theta5)*sin(theta6) - L1*cos(theta5)*sin(theta3)*sin(theta4)*sin(theta6);...
                                                                                                                                                                                                                                                                                                               cos(theta5);...
                                                                                                                                                                                                                                                                                                  -cos(theta6)*sin(theta5);...
                                                                                                                                                                                                                                                                                                   sin(theta5)*sin(theta6)];

% J = [u1,u2,u3,u4,u5,u6];

u3 = [
                                                                                            -sin(theta5)*(L4 + L2*cos(theta4));...
 L2*sin(theta4)*sin(theta6) - L4*cos(theta5)*cos(theta6) + L5*sin(theta5)*sin(theta6) - L2*cos(theta4)*cos(theta5)*cos(theta6);...
 L2*cos(theta6)*sin(theta4) + L4*cos(theta5)*sin(theta6) + L5*cos(theta6)*sin(theta5) + L2*cos(theta4)*cos(theta5)*sin(theta6);...
                                                                                                                   cos(theta5);...
                                                                                                      -cos(theta6)*sin(theta5);...
                                                                                                       sin(theta5)*sin(theta6)];

u4 = [
                                         -L4*sin(theta5);...
 L5*sin(theta5)*sin(theta6) - L4*cos(theta5)*cos(theta6);...
 L4*cos(theta5)*sin(theta6) + L5*cos(theta6)*sin(theta5);...
                                             cos(theta5);...
                                -cos(theta6)*sin(theta5);...
                                 sin(theta5)*sin(theta6)];

u5 = [
               0;...
  L5*cos(theta6);...
 -L5*sin(theta6);...
               0;...
     sin(theta6);...
     cos(theta6)];

u6 = [0;0;0;1;0;0];

J = [u1,u2,u3,u4,u5,u6];

end