%% Setup robot
travelTime = 0.5; % Defines the traverse 
robot = Robot();
robot.writeTime(travelTime); % Write travel time
% robot.writeMotorState(true); % Write position mode
disp("Initializing...")

%% Determinin Forward Kinematics for Home Configuration
% robot.writeJoints([0,0,0,90]); % Write joints to zero position
% pause(travelTime)pause; % Wait for trajectory completion
% 
% joint_readings = robot.getJointsReadings();
% q = [joint_readings(1, 1), joint_readings(1, 2), joint_readings(1, 3), joint_readings(1, 4)];
% disp("JOINTS: ")
% disp(q);
% 
% Ts = fwk(q);
% disp("Forward Kinematics: ")
% disp(Ts{5})
% 
% J = jacobian(q);
% disp("Jacobian: ")
% disp(J);


% Invalid dimension. Dimension must be a nonnegative integer number or a colon (:).

% Preparing to move the robot in velocity mode
move_robot = true;
robot.writeMode('p');


% Initializing Variables:
% Home configuration
x_d = [0.2823, 0, 0.2150]'; 
% x_d = [0, 0, .35]';
x_d_dot = [0, 0, 0]'; % Assuming no desired velocity
% k = 4.61;
k = 200;
dt = 0.175; % pause time

% Finding Current Position:
joint_readings = robot.getJointsReadings();
q = [joint_readings(1, 1), joint_readings(1, 2), joint_readings(1, 3), joint_readings(1, 4)];
% fprintf('Current q:')
% disp(q)

% Boolean to stop once error is small enough (0.1%)
run = true;

% Finding Initial Error:
Ts0 = fwk(q);
x0 = Ts0{5}(1:3,4);
error_0 = norm(x_d - x0);

% Start timer & store data for graph
tic
errors = [];
time = [];

while(run)
    % Calculating forward kinematics
    Ts = fwk(q);
    x = Ts{5}(1:3,4);

    % Calculating Jacobian:
    J = jacobian(q); 
    J_lin = J(1:3,:);
    
    % Calculating Needed Joint Velocities:
    [q_dot, error] = proportional_control(x, x_d, x_d_dot, J_lin, k);
    q_dot = q_dot';

    errors(end+1) = norm(error);
    time(end+1) = toc;

    if (norm(error) < (0.01* error_0))
        run = false;
        % Finish timer 
        toc;
    end
    % q_dot_deg = q_dot * 180 / pi;
    % disp(q_dot_deg);
    
    % If we need to control it with joint positions:
    q = q(1:4) + q_dot * dt; % integral
    %fprintf('updated q:')
    %disp(q)
    
    % Sending Joint velocities to robot:
    if move_robot
        % robot.writeVelocities(q_dot);
        %disp("JOINT VELOCITIES")
        %disp(q_dot)
        robot.writeJoints(q);
        pause(dt)
    end
    
end


disp("SUCCESS!")

% Plotting error over time
plot(time, errors, 'b');
title('Differential Control Error');
xlabel('Time (s)')
ylabel('Error Norm (m)')
grid on;

