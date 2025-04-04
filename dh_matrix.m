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
    A = [cosd(dh(2)), (-sind(dh(2))) * cosd(dh(4)), sind(dh(2)) * sind(dh(4)), dh(1) * cosd(dh(2)); 
    sind(dh(2)), cosd(dh(2)) * cosd(dh(4)), (-cosd(dh(2))) * (sind(dh(4))), dh(1) * sind(dh(2)); 
    0, sind(dh(4)), cosd(dh(4)), dh(3);
    0, 0, 0, 1];
end