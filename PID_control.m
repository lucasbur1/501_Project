function [joint_velocities, error] = PID_control(error, d_error, i_error, x_d_dot, J, k_p, k_d, k_i)
    %PROPORTIONAL_CONTROL Calculates joint velocities needed for
    % proportional control. 
    %   INPUTS:
    %       x: Current position (3 x 1 vector)
    %       x_d: Desired position (3 x 1 vector)
    %       x_d_dot: Desired Velocity (3 x 1 vector)
    %       J: Jacobian (3 x n matrix)
    %       k: gain

    p_effect = k_p * error;
    d_effect = k_d * d_error;
    i_effect = k_i * i_error;
    joint_velocities = pinv(J) * (x_d_dot + p_effect + d_effect + i_effect);
    
end