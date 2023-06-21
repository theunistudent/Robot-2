% Author: Raghav
startup_rvc;
close all; clear all; clc;

%% Revolute 2 link arm
% Parameters are defined in the toolbox here: https://www.petercorke.com/RTB/r9/html/Link.html
L(1) = Link('revolute', 'd', 0, 'a', 3, 'alpha', 0);  % Link 1. 
L(2) = Link('revolute', 'd', 0, 'a', 4, 'alpha', 0);  % Link 2.
q1_arm = SerialLink(L, 'name', 'q1 two link')  % Put the two links together into a serial manipulator

% Add an end effector of zero length
q1_arm.tool = transl(0, 0, 0)

% The joint angles
q = deg2rad([30,60])

% Lets plot this arm!
q1_arm.plot(q)

% The teach command opens up a manipulatable GUI
q1_arm.teach

% Calculate the forward kinematics
eepose = q1_arm.fkine(q)

% Translation Matrix
eepose.transl


% Calculate the inverse Kinematics
% The mask indicates how many links this serial link manipulator has
qi = q1_arm.ikine(eepose,'mask', [1 1 0 0 0 0])
% Sometimes you need to provide an inital guess for the inverse kinematic
% solver. This can be done like the following
%qi = q1_arm.ikine(T, 'q0', [1 3], 'mask', [1 1 0 0 0 0])


% Calculate the jacobian
j = q1_arm.jacob0(q)


%% Manipulator with a prismatic joint
clear all
L_1 = 5;
L_2 = 4;
L_3 = 4;
d3 = 3;

L(1) = Link([0 L_1 0 pi/2]);  % Link 1. Order of parameters = [theta, d, a, alpha]

L(2) = Link([0 0 L_2 pi/2]);  % Link 2
L(2).offset = pi/2;

L(3) = Link([-pi/2 0 0 0]);  % Link 3
L(3).jointtype = 'P';
L(3).qlim = [0 d3];  % Range of motion for prismatic joint (must be positive)
L(3).offset = L_3;  % Offset is added so rotation is measured from the previous z axis as zero


q3_arm = SerialLink(L, 'name', 'q3 three link')  % Put the links together into a serial manipulator

% Lets plot this arm
q3_arm.plot([deg2rad(30) deg2rad(60) d3])


q3_arm.teach

% Calculate the Forward Kinematics
eepose = q3_arm.fkine([deg2rad(30) deg2rad(60) d3])

% Calculate the inverse kinematics
qi = q3_arm.ikine(eepose,'mask', [1 1 1 0 0 0])


%%
% Load in the ur5 model. Note this is not the ur5e model
% Notice in the work space we have 2 predefined joint angles. qr and qz.
% Plot both of them to see what they are. 
mdl_ur5;

% In this section we have a look at the transformation matrix assocaited 
% with the forward kinematics. 
% This transformation matrix can be broken up into its transformation and
% rotational components.

% Set q as qr
q = deg2rad([-90, -60, 90, 0, 90, 0])

% Plot the ur5 at that joint position.
ur5.plot(q)

% Calculate the forward kinematics at the selected position
T = ur5.fkine(q)


% Now add in a slight position displacement to to the very first joint angle.
q2 = q +[0.01 0 0 0 0 0];
Td = ur5.fkine(q2)

ur5.plot([q;q2])

% What differences can you observe between T and Td? They are very slightly
% different. Just one minor displacement in the very first joint affects
% the final position of the end effector.

% For reference:
% You can extract out the Translation component
translation = T.transl
% You can extract out the rotational component
rotation = T.R

% We can calculate the jacobian at point q : 
J = ur5.jacobe(q)

