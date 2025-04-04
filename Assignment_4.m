% Setting up Robot:
robot = Robot();
move_robot = false;

% Initializing Variables:
J = [1, 2, 3, 4;
     0, 0, 1, 1;
     2, 3, 4, 0]; % Test jacobian
x_d = [1, 1, 1]'; 
x_d_dot = [0, 0, 0]'; % Assuming no desired velocity
k = 4.61;
dt = 1; % pause time



for i = 1 : 10

    % Finding Current Position:
    joint_readings = robot.getJointsReadings();
    q = joint_readings(1, :)';
    x = [0, 0, 0]'; % (REPLACE WITH FORWARD KINEMATICS):

    % Calculating Jacobian:
    J = J; % (REPLACE)
    
    % Calculating Needed Joint Velocities:
    q_dot = proportional_control(x, x_d, x_d_dot, J, k)'
    
    % If we need to control it with joint positions:
    q = q + q_dot * dt; % integral
    
    
    % Sending Joint velocities to robot:
    if move_robot
        robot.writeVelocities(q_dot);
        % robot.writeJoints(q); 
    end

    pause(dt);
    
end

