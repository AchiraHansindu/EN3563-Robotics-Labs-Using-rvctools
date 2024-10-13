%3.1
mdl_puma560;

%3.2
p560;

% 3.3
disp('Zero angle configuration:');
qz = p560.qz;
disp(p560.qz);
disp('Ready configuration:');
qr = p560.qr;
disp(p560.qr);
disp('Stretch configuration:');
qs = p560.qs;
disp(p560.qs);
disp('Nominal configuration:');
qn = p560.qn;
disp(p560.qn);

% 3.4
p560.tool = SE3(0, 0, 0.2);

%3.5
T_zero = p560.fkine(qz);  % Forward kinematics for zero angle configuration
T_ready = p560.fkine(qr);  % Forward kinematics for ready configuration
T_stretch = p560.fkine(qs);  % Forward kinematics for stretch configuration
T_nominal = p560.fkine(qn);  % Forward kinematics for nominal configuration

disp('Position and Orientation (TCP) for Zero Angle:');
disp(T_zero);
disp('Position and Orientation (TCP) for Ready:');
disp(T_ready);
disp('Position and Orientation (TCP) for Stretch:');
disp(T_stretch);
disp('Position and Orientation (TCP) for Nominal:');
disp(T_nominal);

%3.6
p560.plot3d(qz);  % Visualization for zero angle configuration
hold on;
title('Puma 560 - Zero Angle Configuration');

p560.plot3d(qr);  % Visualization for ready configuration
hold on;
title('Puma 560 - Ready Configuration');

p560.plot3d(qs);  % Visualization for stretch configuration
hold on;
title('Puma 560 - Stretch Configuration');

p560.plot3d(qn);  % Visualization for nominal configuration
hold on
title('Puma 560 - Nominal Configuration');

%3.7
p560.tool = SE3();  % Reset tool to no extension

%3.8
T_nominal = p560.fkine(qn);  % Forward kinematics for nominal configuration
q_inv = p560.ikine6s(T_nominal);  % Inverse kinematics

disp('Inverse kinematics joint vector for nominal configuration:');
disp(q_inv);

%3.9
q_left_elbow_up = p560.ikine6s(T_nominal, 'l', 'u');  % Left hand, elbow up
q_left_elbow_down = p560.ikine6s(T_nominal, 'l', 'd');  % Left hand, elbow down
q_right_elbow_up = p560.ikine6s(T_nominal, 'r', 'u');  % Right hand, elbow up
q_right_elbow_down = p560.ikine6s(T_nominal, 'r', 'd');  % Right hand, elbow down

% Display the results
disp('Left hand, elbow up:');
disp(q_left_elbow_up);
disp('Left hand, elbow down:');
disp(q_left_elbow_down);
disp('Right hand, elbow up:');
disp(q_right_elbow_up);
disp('Right hand, elbow down:');
disp(q_right_elbow_down);

%3.10
T_unreachable = SE3(10, 10, 10); 
q_unreachable = p560.ikine6s(T_unreachable);

% Display the result
disp('Inverse kinematics result for unreachable point:');
disp(q_unreachable);

%3.11
close all;
clear all;

mdl_puma560;  % Load Puma 560 robot model

T1 = SE3(0.8, 0, 0) * SE3.Ry(pi/2);  % First transformation
T2 = SE3(-0.8, 0, 0) * SE3.Rx(pi);  % Second transformation

q1 = p560.ikine6s(T1);  % Inverse kinematics for T1
q2 = p560.ikine6s(T2);  % Inverse kinematics for T2

t = [0:0.05:2]';  % Time vector for trajectory
q = jtraj(q1, q2, t);  % Joint space trajectory from q1 to q2

p560.plot3d(q);  % Plot the robot motion
