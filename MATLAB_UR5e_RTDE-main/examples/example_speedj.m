% Author: Raghav Hariharan
% For MTRN4230 2023

%% ----- EXAMPLE ServoJ -----
% This program shows how to use the servoJ command
clear all;
% 
% TCP Host and Port settings
% host = '127.0.0.1'; % THIS IP ADDRESS MUST BE USED FOR THE VM
% host = '192.168.0.100'; % THIS IP ADDRESS MUST BE USED FOR THE REAL ROBOT
host = '192.168.230.128';
port = 30003;


% Calling the constructor of rtde to setup tcp connction
rtde = rtde(host,port);

home = [-588.53, -133.30, 371.91, 2.2214, -2.2214, 0.00];


joint_q = [-1.54, -1.83, -2.28, -0.59, 1.60, 0.023];

joint_speeds = [deg2rad(50),0,0,0,0,0];
acceleration = 0.5;

rtde.movej(home);


[poses,jointPositions,jointVelocities,jointAccelerations,torques] = rtde.speedj(joint_speeds,acceleration);


rtde.drawPath(poses);
rtde.drawJointPositions(jointPositions);
rtde.drawJointVelocities(jointVelocities);
rtde.drawJointAccelerations(jointAccelerations);                                                                                                                                         