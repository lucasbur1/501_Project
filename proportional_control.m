function [joint_velocities, error] = proportional_control(x, x_d, x_d_dot, J, k)
    %PROPORTIONAL_CONTROL Calculates joint velocities needed for
    % proportional control. 
    %   INPUTS:
    %       x: Current position (3 x 1 vector)
    %       x_d: Desired position (3 x 1 vector)
    %       x_d_dot: Desired Velocity (3 x 1 vector)
    %       J: Jacobian (3 x n matrix)
    %       k: gain

    error = x_d - x;
%     disp("ERROR: ")
%     disp(error);
    joint_velocities = pinv(J) * (x_d_dot + k*error);
    
end