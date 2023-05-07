% function [ points ] = drawfigure()
    % Create a plot
    figure();
    canvas = [1 1; 1 -1; -1 -1; -1 1];
    hold on
    plot(canvas(:, 1), canvas(:, 2), 'r-');
    plot(0, 0, 'ro');
    hold off;
    % Initialize an empty array to store the points
    global points;
    points = [];
    global mouse_down;
    mouse_down = false;
    % Set the callback functions for the plot
    set(gcf, 'WindowButtonMotionFcn', @dragCallback);
    set(gcf, 'WindowButtonUpFcn', @end_drag);
    set(gcf, 'WindowButtonDownFcn', @start_drag);
    set(gcf, 'KeyPressFcn', @check_key);
    % Wait for the user to finish dragging and close the plot
    disp('Drag over the plot to record points:');
    waitfor(gcf);
    disp('Recorded points:');
    disp(points);
    % Define a function to be called when the user drags the mouse over the plot
    function dragCallback(src, evt)
    global mouse_down;
    global points
    if mouse_down
    % Get the current point and add it to the array
    point = get(gca, 'CurrentPoint');
    disp(point(1,1:2));
    points = [points; point(1,1:2)];
    % Update the plot with the new point
    hold on
    plot(point(1,1), point(1,2), 'b*');
    hold off
    end
    end
    function start_drag(~, ~)
    global mouse_down;
    mouse_down = true;
    end
    % Define the function that clears the mouse-down flag
    function end_drag(~, ~)
    global mouse_down;
    global points
    mouse_down = false;
    % plot(points(:, 1), points(:, 2), 'b-');
    end

    % Define the function that checks if the "Enter" key is pressed
    function check_key(~, event)
        global points
        % Check if the "Enter" key is pressed
        if strcmp(event.Key, 'return')
            plot(points(:, 1), points(:, 2), 'b-');
        end
        points(:,1) = points(:,1) - points(1,1);
        points(:,2) = points(:,2) - points(1,2);
    end
% 
% end


