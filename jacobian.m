% This function calculates the jacobian of the OpenManipulator-X robot arm
% Authors: Lucas Burstein & Callahan Henry
% Date: 4/4/25

% jacobian(q) calculates the jacobian of the OpenManipulator-X arm
% q: vector of joint variables (5x1)
% Returns a matrix J representing the jacobian given q
function J = jacobian(q)
    % Calculating the forward kinematics 
    Ts = fwk(q);
    A1 = Ts(1);
    A2 = Ts(2);
    A3 = Ts(3);
    A4 = Ts(4);
    T50 = Ts(5);
    
    % Calculating various transformations required to calculate jacobian 
    T20 = A1 * A2;
    T30 = A1 * A2 * A3;
    T40 = A1 * A2 * A3 * A4;
    T = {A1, T20, T30, T40};
    
    % Calculate Jacobian
    J = zeros(6, 4);
    for i = 1:length(T)
        % Normalization ensures small errors won't accumulate
        z = T{i}(0:3, 2);
        z_norm = z / norm(z);
        o = T{i}(0:3, 3);
        J(0:3, i) = cross(z_norm, T50(0:3,3) - o);
        J(3:6, i) = z_norm;
    end

end