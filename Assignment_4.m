%% Setup robot
travelTime = 5; % Defines the traverse 
robot = Robot();
robot.writeTime(travelTime); % Write travel time
robot.writeMotorState(true); % Write position mode
disp("Initializing...")

%% Determinin Forward Kinematics for Home Configuration
% robot.writeJoints([0,0,0,0]); % Write joints to zero position
% pause(travelTime); % Wait for trajectory completion
% joint_readings = robot.getJointsReadings();
% q = [joint_readings(1, 1), joint_readings(1, 2), joint_readings(1, 3), joint_readings(1, 4), 0];
% disp(q);
% Ts = fwk(q);
% disp(Ts{5})



% Invalid dimension. Dimension must be a nonnegative integer number or a colon (:).

% Preparing to move the robot in velocity mode
move_robot = true;
robot.writeMode('v');


% Initializing Variables:
x_d = [0.2823, 0, 0.2150]'; 
x_d_dot = [0, 0, 0]'; % Assuming no desired velocity
k = 4.61;
dt = 0.1; % pause time

for i = 1 : 1000

      % Finding Current Position:
    joint_readings = robot.getJointsReadings();
    q = [joint_readings(1, 1), joint_readings(1, 2), joint_readings(1, 3), joint_readings(1, 4), 0];
    fprintf('Current q:')
    disp(q)

    % Calculating forward kinematics
    Ts = fwk(q);
    x = Ts{5}(1:3,4);

    % Calculating Jacobian:
    J = jacobian(q); 
    J_lin = J(1:3,:);
    
    % Calculating Needed Joint Velocities:
    q_dot = proportional_control(x, x_d, x_d_dot, J_lin, k)';
    q_dot_deg = q_dot * 180 / pi;
    % disp(q_dot_deg);
    
    % If we need to control it with joint positions:
%     q = q(1:4) + q_dot * dt; % integral
%     fprintf('updated q:')
%     disp(q)
    
    % Sending Joint velocities to robot:
    if move_robot
        robot.writeVelocities(q_dot_deg);
        disp("JOINT VELOCITIES")
        disp(q_dot)
        % robot.writeJoints(q);
        % pause(dt)

    end
    
end

