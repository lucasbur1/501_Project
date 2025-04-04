% This function calculates the dh matrix corresponding to the provided dh
% parameters
% Authors: Lucas Burstein & Callahan Henry
% Date: 4/4/25

% dh_matrix(dh) takes in a vector of dh parameters and returns the
% transformation
% dh = [a, theta, d, alpha] 
% returns the dh transformation matrix (A)
function A = dh_matrix(dh)
    % Calculates the 'A' transformation matrix from the given DH parameters using the general formula for the 'A' matrix.
    A = [cos(dh(1)), (-sin(dh(1))) * cos(dh(3)), sin(dh(1)) * sin(dh(3)), dh(0) * cos(dh(1)); 
    sin(dh(1)), cos(dh(1)) * cos(dh(3)), (-cos(dh(1))) * (sin(dh(3))), dh(0) * sin(dh(1)); 
    0, sin(dh(3)), cos(dh(3)), dh(2);
    0, 0, 0, 1];
end