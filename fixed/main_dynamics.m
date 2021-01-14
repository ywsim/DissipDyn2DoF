%% Init
addpath(genpath(pwd))
clear; clc; close all;
flagWriteFcn = true;

%% Generate Rigid Body System
robot = gen_fixed_leg_reflected();

%% Generate Contact Jacobian
robot = gen_contact(robot, flagWriteFcn);

%% Generate Transmission D/G
robot = gen_transmission(robot, flagWriteFcn);

%% Calculate (Un)Reduced Dynamics of the Rigid Body System
robot = get_dynamics(robot, flagWriteFcn);

%% Save
cd dynamics
save('robot_dynamics.mat', 'robot');
cd ..
disp('- saved robot dynamics')
