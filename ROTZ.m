<<<<<<< Updated upstream
function RotationZ = ROTZ(q)

RotationZ = [cos(q),-sin(q),0;...
        sin(q),cos(q),0;...
        0,      0,    1];
end

=======
function matrix = ROTZ(theta)
   
    matrix = [cos(theta) -sin(theta) 0; sin(theta) cos(theta) 0; 0 0 1];
    
end
>>>>>>> Stashed changes
