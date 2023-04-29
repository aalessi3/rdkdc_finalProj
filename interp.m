function [points] = interp(g_start, g_final, steps)
    
    g_temp = g_start;
%     cart_points = zeros(3, steps);
    points = repmat(g_start, [1,1, steps]);
    p_start = g_start(1:3, 4);
    p_final = g_final(1:3, 4);

    
    for i = 1:steps
        g_temp(1:3, 4) = p_start + ((p_final-p_start)/steps)*i;
        points(:,:,i) = g_temp();
%         points(:, i) = p_start + ((p_final-p_start)/steps)*i;

    end


end