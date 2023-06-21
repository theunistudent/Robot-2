%%%%%%%%%%%%%
% MTRN4230 Trajectory control sample code
% Mark Whitty
% 20210624
% Adapted from Corke ch 3.

%% Planning simple quintic polynomial trajectories
%% Ex 1
close all; clear variables; clc;
startup_rvc;
dbstop if error
mdl_puma560

T1 = transl(0.4, 0.2, 0) * trotx(pi);
T2 = transl(0.4, -0.2, 0) * trotx(pi/2);
q1 = p560.ikine6s(T1);
q2 = p560.ikine6s(T2);

t = [0:0.01:5];
s = mtraj(@tpoly, q1, q2, t);
sd = [zeros(1,6); diff(s)];
sdd = [zeros(1,6); diff(sd)];
sddd = [zeros(1,6); diff(sdd)];

%Show position, velocity and acceleration as a function of time
f1 = figure(1);
subplot(4,1,1)
plot(t, s); grid on; xlim([min(t), max(t)]);
title('Quintic polynomial trajectory');
xlabel('Time');
ylabel('Position [rad]')
legend('q1', 'q2', 'q3', 'q4', 'q5', 'q6');
subplot(4,1,2)
plot(t, sd); grid on; xlim([min(t), max(t)]);
xlabel('Time'); 
ylabel('Velocity [rad/s]')
subplot(4,1,3)
plot(t, sdd); grid on; xlim([min(t), max(t)]);
xlabel('Time');
ylabel('Acceleration [rad/s/s]')
subplot(4,1,4)
plot(t, sddd); grid on; xlim([min(t), max(t)]);
xlabel('Time');
ylabel('Jerk [rad/s/s/s]')

%% Planning joint space trajectories with lspb
f2 = figure(2);
s = mtraj(@lspb, q1, q2, t);
sd = [zeros(1,6); diff(s)];
sdd = [zeros(1,6); diff(sd)];
sddd = [zeros(1,6); diff(sdd)];

subplot(4,1,1)
plot(t, s); grid on; xlim([min(t), max(t)]);
title('LSPB trajectory');
xlabel('Time');
ylabel('Position [rad]')
legend('q1', 'q2', 'q3', 'q4', 'q5', 'q6');
subplot(4,1,2)
plot(t, sd); grid on; xlim([min(t), max(t)]);
xlabel('Time'); 
ylabel('Velocity [rad/s]')
subplot(4,1,3)
plot(t, sdd); grid on; xlim([min(t), max(t)]);
xlabel('Time');
ylabel('Acceleration [rad/s/s]')
subplot(4,1,4)
plot(t, sddd); grid on; xlim([min(t), max(t)]);
xlabel('Time');
ylabel('Jerk [rad/s/s/s]')


f4 = figure(4);
T = p560.fkine(s);
p = transl(T);  % Just translational components
plot(p(:, 1), p(:, 2)); axis equal; xlabel('X [m]'); ylabel('Y [m]'); grid on; title('Locus of joint space path');

%% Cartesian Motion
% Note giving a scalar value to ctraj breaks it, as it uses lspb and due to poor numberical precision the values are not bounded within [0, 1] for interpolation.
% Hence, apply a full vector of times, scaled to [0,1]
Ts = ctraj(T1, T2, t/5);
qc = p560.ikine6s(Ts, 'alpha', 0.2);

f5 = figure(5);
qplot(t, qc); xlabel('Time [s]'); ylabel('Angle [rad]');
title('Joint trajectories for Cartesian path');

f6 = figure(6);
pc = transl(Ts);  % Just translational components
plot(pc(:, 1), pc(:, 2)); axis equal; xlabel('X [m]'); ylabel('Y [m]'); grid on; title('Locus of Cartesian path');

%% Cartesian motion through a singularity
T1 = transl(0.5, 0.3, 0.44) * troty(pi/2);
T2 = transl(0.5, -0.3, 0.44) * troty(pi/2);
% Note giving a scalar value to ctraj breaks it, as it uses lspb and due to poor numberical precision the values are not bounded within [0, 1] for interpolation.
% Hence, apply a full vector of times, scaled to [0,1]
Ts = ctraj(T1, T2, t/5);
qc = p560.ikine6s(Ts);

f7 = figure(7);
qplot(t, qc); xlabel('Time [s]'); ylabel('Angle [rad]');
title('Joint trajectories for Cartesian path through a singularity');

f8 = figure(8);
qc2 = p560.jtraj(T1, T2, t);
qplot(t, qc2); xlabel('Time [s]'); ylabel('Angle [rad]');
title('Joint trajectories for a joint space path through a Singularity');




