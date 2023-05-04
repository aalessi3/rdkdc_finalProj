clc; 
clear;

ur5 = ur5_interface(); 


if norm(ur5.get_current_joints()) <= 1e-3
    disp('UR5 is already at the home configuration')
else 
    disp('Moving to home loaction...')
    ur5.move_joints([0;0;0;0;0;0],10)
    pause(10)
    disp('At home location')
end



disp('Moved the robot to goal loaction and then press enter')

w = waitforbuttonpress;
if w == 1
    start_location = ur5FwdKin(ur5.get_current_joints()); 
    disp('Start location recorded')
end

disp('Move the robot to target location and then press enter')
w = waitforbuttonpress; 
if w == 1
    target_location = ur5FwdKin(ur5.get_current_joints()); 
    disp('Target location recorded')
end

disp('Moving back to home position')
ur5.move_joints([0;0;0;0;0;0],10)
pause(10)
disp('At home location')

s = [start_location(1,4), start_location(2,4)];
t = [target_location(1,4), target_location(2,4)];

points = intermediatePoints(s,t);

start_location1 = start_location;
target_location1 = start_location;
target_location1(1,4) = points(1,3); 
target_location1(2,4) = points(1,4); 

start_location2 = target_location1; 
target_location2 = target_location;
target_location2(1,4) = points(2,3); 
target_location2(2,4) = points(2,4); 

start_location3 = target_location2;
target_location3 = target_location;
target_location3(1,4) = points(3,3); 
target_location3(2,4) = points(3,4); 



