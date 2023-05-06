clc; 
clear;
steps = 10;

ur5 = ur5_interface(); 

ur5.get_current_joints()

if abs((norm(ur5.get_current_joints()) - norm([0;-1.57;0;-1.57;0;0]))) <= 1e-3
    disp('UR5 is already at the home configuration')
else 
    disp('Moving to home loaction...')
    ur5.move_joints([0;-1.57;0;-1.57;0;0],5)
    pause(5)
    disp('At home location')

end

disp('Moved the robot to start loaction and then press enter')

w = waitforbuttonpress;
if w == 0
    start_q = ur5.get_current_joints();
    start_location = ur5FwdKin(start_q); 
    disp('Start location recorded')
    disp(start_q);
end

disp('Move the robot to target location and then press enter')
w = waitforbuttonpress; 
if w == 0
    target_q = ur5.get_current_joints();
    target_location = ur5FwdKin(target_q); 
    disp('Target location recorded')
    disp(target_q);
end

disp('Moving back to home position')
ur5.move_joints([0;-1.57;0;-1.57;0;0],5);
pause(5)
disp('At home location')

s = [start_location(1,4), start_location(2,4)];
t = [target_location(1,4), target_location(2,4)];

points = intermediatePoints(s,t);

start_location1 = start_location;
target_location1 = start_location;
target_location1(1,4) = points(1,3); 
target_location1(2,4) = points(1,4); 

start_location2 = target_location1; 
target_location2 = start_location;
target_location2(1,4) = points(2,3); 
target_location2(2,4) = points(2,4); 
target_location2(3,4) = target_location(3,4);

start_location3 = target_location2;
target_location3 = start_location;
target_location3(1,4) = points(3,3); 
target_location3(2,4) = points(3,4); 


q_start_1 = start_q;
[result, q_start_2, ind] = optimalJointConfig(ur5, ur5InvKin_wrap(start_location2));
[result, q_start_3, ind] = optimalJointConfig(ur5, ur5InvKin_wrap(start_location3));

[result, q_goal_1, ind] = optimalJointConfig(ur5, ur5InvKin_wrap(target_location1));
[result, q_goal_2, ind] = optimalJointConfig(ur5, ur5InvKin_wrap(target_location2));
q_goal_3 = target_q;

INV = false;
RR = false;
JT = false;
%---------------- Traj A using InvKin ----------------
% 
disp("Entering Control Loop for InvKin Control : Traj A")

disp("Drawing Line Segment 1");

while(ur5InvKcontrol(q_start_1, q_goal_1, ur5, steps) ~= 1)

    ur5.get_current_joints()

end

disp("Drawing Line Segement 2");

while(ur5InvKcontrol(q_start_2, q_goal_2, ur5, steps) ~= 1)

    ur5.get_current_joints()

end

disp("Drawing Line Segement 3");

while(ur5InvKcontrol(q_start_3, q_goal_3, ur5, steps) ~= 1)

    ur5.get_current_joints()

end

disp('Moving back to home position')
ur5.move_joints([0;-1.57;0;-1.57;0;0],5);
pause(5)
disp('At home location')


%----------------Traj A using RR control---------------




%---------------Generate traj B --------------- using start_location 1 =
%target Location 1



