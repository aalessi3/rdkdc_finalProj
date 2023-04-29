function [solution, qtt] = optimalJointConfig(ur5, qtt_array)
%% Outputs  
% optimum joint configuration for time t+1 
%% Inputs 
% ur5: ur5 interface object  
% qt_array: Possible array of joint configuration for tiem t+1 
[~, n_col] = size(qtt_array);

min_norm = 1000; 
q = ur5.current_joint_states(); 
qtt = []; 
solution = 0;

for i = 1:n_col
    if (ur5.check_collision(qtt_array(:,i)))
        norm_ = norm(q - qtt_array(:,i)); 
        if (norm_ < min_norm) 
            min_norm = norm_; 
            qtt = qtt_array(:,i);
            solution = 1; 
        end
    end

end


end