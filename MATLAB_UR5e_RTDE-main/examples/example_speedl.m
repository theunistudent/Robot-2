% Author: Raghav Hariharan
% For MTRN4230 2023

%% ----- EXAMPLE ServoJ -----
% This program shows the basic useage of the rtde library.
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


xd = [0.1,0,0.0,0,0,0];

acceleration = 0.5;
time = 2;

rtde.movej(home);


[poses,jointPositions,jointVelocities,jointAccelerations,torques] = rtde.speedl(xd,acceleration,time);


rtde.drawPath(poses);
rtde.drawJointPositions(jointPositions);
rtde.drawJointVelocities(jointVelocities);
rtde.drawJointAccelerations(jointAccelerations);                                                                                                                                         