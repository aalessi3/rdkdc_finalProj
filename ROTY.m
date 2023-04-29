<<<<<<< Updated upstream
function RotationY = ROTY(q)
    RotationY = [cos(q),0,sin(q);...
        0,1,0;...
        -sin(q),0,cos(q)];
end

=======
function matrix = ROTY(theta)
   
    matrix = [cos(theta) 0 sin(theta); 0 1 0 ; -sin(theta) 0 cos(theta)];
    
end
>>>>>>> Stashed changes
