% clear all
% close all
% clc
% 
% addpath('./PSO'); % Adding PSO library
% load('variables.mat')

kin = KinematicValidation(robot, joint_lim, des_frame, des_vel, des_wrench); % Building object
kin.back_fwd_calculation_loop([0 0 0], P, DMT, 2) % Calculate and optimize motor torques and speeds

