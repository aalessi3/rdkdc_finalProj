<<<<<<< Updated upstream
function RotationX = ROTX(q)
%ROTX Summary of this function goes here
%   input: radians
    RotationX = [1,0,0;...
        0,cos(q),-sin(q);...
        0,sin(q),cos(q)];
end

=======
function matrix = ROTX(theta)

    matrix = [1 0 0; 0 cos(theta), -sin(theta); 0 sin(theta) cos(theta) ];

end
>>>>>>> Stashed changes
