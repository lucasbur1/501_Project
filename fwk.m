% This script calculates the forward kinematics of the OpenManipulator-X
% arm
% Authors: Lucas Burstein & Callahan Henry
% Date: 4/4/25

% TEST

% fwk(q) calculates the forward kinematics of the OpenManipulator-X arm
% q: vector of joint variables (5x1)
% Returns a cell array of matrices representing the forward kinematics
function Ts = fwk(q)
    % Defining the joint angles
    q1 = q(1);
    q2 = q(2);
    q3 = q(3);

% fwk(q) calculates the forward kinematics of the OpenManipulator-X arm
% q: vector of joint variables (5x1)
% Returns a cell array of matrices representing the forward kinematics
function Ts = fwk(q)
    q4 = q(4);
    q5 = q(5);

    % Defining the link lengths (mm)
    l0 = 36.076/1000;
    l1 = 60.25/1000;
    l2 = 130.23/1000;
    l3 = 124/1000;
    l4 = 133.4/1000;
    gamma = 1.385; 

    % Calculating and storing the DH Parameters for each link in the order [a, theta, d, alpha]
    dh0 = [0, 0, l0, 0]; 
    dh1 = [0, q1, l1, -1.5708];
    dh2 = [l2, q2-gamma, 0, 0];
    dh3 = [l3, q3+gamma, 0, 0];
    dh4 = [l4, q4, 0, 0];

    % Calculating the transformation matrices between consecutive frames using the DH parameters. 
    % The function 'dh_matrix' takes the DH parameters of a particular link as input and returns the transformation matrix of that frame with respect to the previous frame.
    A1 = dh_matrix(dh0); % A1 is the transformation matrix that converts frame 0 to 1
    A2 = dh_matrix(dh1); % A2 is the transformation matrix that converts frame 1 to 2
    A3 = dh_matrix(dh2); % A3 is the transformation matrix that converts frame 2 to 3
    A4 = dh_matrix(dh3); % A4 is the transformation matrix that converts frame 3 to 4
    A5 = dh_matrix(dh4); % A5 is the transformation matrix that converts frame 4 to 5

 
    % This gives the pose of the end-effector with respect to the base frame. 
    % This is obtained by multiplying all the individual transformation matrices between consecutive frames (T50 = A1*A2*A3*A4*A5)
    T50 = A1* A2 * A3 * A4 * A5; 

    % Return cell array of required variables
    Ts = {A1, A2, A3, A4, T50};
end