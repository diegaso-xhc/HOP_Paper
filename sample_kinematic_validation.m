clear all
close all
clc

addpath('./PSO'); % Adding PSO library

links = {'link1', 'link2', 'link3'}; % A cell with the names of the bodies for the RST
joints = {'jnt1', 'jnt2', 'jnt3'}; % A cell with the names of the joints
type = {'revolute', 'revolute', 'revolute'}; % Types of the joints 
frames = {trvec2tform([0.0 0.15 0]), ... % 4x4 homogenous transformation matrices representing each joint w.r.t the previous joint
    trvec2tform([0.5 0 0]), ...
    trvec2tform([0.5 0 0]), ...
    trvec2tform([0.25, 0, 0])};
home = [0 0 0]; % Home positions for each joint
robot = KinematicValidation.build_rigid_body_model(links, joints, type, frames, home); % The KinematicValidation class has a function to build rigid body trees as well.
joint_lim = [-pi/2 pi/4; -pi/2 0; -pi/2 0]; % Joint limits 
%%%% Definition of a sample target frame for the end effector
axang = [0 0 1 -3*pi/4]; 
rotm = axang2rotm(axang);
M = eye(4);
M(1:3,1:3) = rotm;
M(1:3,4) = [0.7 -0.3 0]';
des_frame = M;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%% Definition of a random twist w.r.t. the base frame (in our case, since
%%%% it is a 2D-kinematic chain, there is only one angular velocity (1:3) and only 
%%%% two components on the linear velocity part (4:6)
des_vel = 5*rand(6,1);
des_vel(1:2,1) = 0;
des_vel(end,1) = 0;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
P = rand(3,2); % Tendon coupling Matrix (in this case 3 joints, 2 tendons)
DMT = [15 0; 0 4]; % A diagonal matrix containing the Drive motor train coefficients
%%%% Definition of a random desired wrench. Similarly, due to this
%%%% particular problem, some coefficients are automatically zero
des_wrench = zeros(6,1);
des_wrench(1:2,1) = 0;
des_wrench(end,1) = 0;
des_wrench(3:5,1) = 1./des_vel(3:5,1);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

save('variables.mat', 'robot', 'joint_lim', 'des_frame', 'des_vel', 'des_wrench', 'P', 'DMT');

kin = KinematicValidation(robot, joint_lim, des_frame, des_vel, des_wrench); % Building object
kin.back_fwd_calculation_loop([0 0 0], P, DMT, 2) % Calculate and optimize motor torques and speeds

%%%% Getting results
Jacobian = kin.geom_jacobian;
sq_error_pos = kin.get_pos_square_error; % frame error
sq_error_vel = kin.get_vel_square_error; % twist error
sq_error_wrench = kin.get_wrench_square_error; % wrench error
config = kin.curr_config; % Current joint positions
endeff = kin.curr_endeff_frame; % Current end effector frame
current_twist = kin.curr_vel;
current_wrench = kin.curr_wrench;
desired_frame = kin.des_frame;
desired_twist = kin.des_vel;
desired_wrench = kin.des_wrench;

%%%% Visualize robot configuration
% show(kin.rigid_body_model, kin.curr_config)