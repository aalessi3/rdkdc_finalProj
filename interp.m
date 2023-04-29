function [points] = interp(g_start, g_final, steps)
    
    points = zeros(3, steps);
    p_start = g_start(1:3, 4);
    p_final = g_final(1:3, 4);

    
    for i = 1:steps
        points(:, i) = p_start + ((p_final-p_start)/steps)*i;
    end


end