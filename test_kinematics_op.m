clear all
close all
clc

link1 = rigidBody('link1');
jnt1 = rigidBodyJoint('jnt1', 'revolute');
jnt1.HomePosition = 0;
tform = trvec2tform([0.0 0.15 0]);
setFixedTransform(jnt1, tform);
link1.Joint = jnt1;

link2 = rigidBody('link2');
jnt2 = rigidBodyJoint('jnt2', 'revolute');
jnt2.HomePosition = 0;
tform2 = trvec2tform([0.5 0 0]);
setFixedTransform(jnt2, tform2);
link2.Joint = jnt2;

link3 = rigidBody('link3');
jnt3 = rigidBodyJoint('jnt3', 'revolute');
jnt3.HomePosition = 0;
tform3 = trvec2tform([0.5 0 0]);
setFixedTransform(jnt3, tform3);
link3.Joint = jnt3;

bodyEndEffector = rigidBody('endeffector');
tform5 = trvec2tform([0.25, 0, 0]); % User defined
setFixedTransform(bodyEndEffector.Joint,tform5);

robot = rigidBodyTree;
robot.DataFormat = 'row'; % To allow users to modify joint values in vector form
addBody(robot, link1, 'base');
addBody(robot, link2, 'link1');
addBody(robot, link3, 'link2');
addBody(robot,bodyEndEffector,'link3');

gik = generalizedInverseKinematics('RigidBodyTree',robot,'ConstraintInputs',{'joint', 'pose'});
jointConst = constraintJointBounds(robot);
jointConst.Bounds = [-pi/2 pi/4; -pi/2 0; -pi/2 0];

poseConst = constraintPoseTarget('endeffector');
axang = [0 0 1 -pi/4];
rotm = axang2rotm(axang);
% M = [0 1 0 1;-1 0 0 -0.2;0 0 1 0;0 0 0 1];
M = eye(4);
M(1:3,1:3) = rotm;
M(1:3,4) = [0.9 -0.3 0]';
poseConst.TargetTransform = M;
poseConst.PositionTolerance = 0.004; % 4mm of tolerance from target position

configSol = gik([0.5934 -1.1424 -1.0552],jointConst,poseConst);
tformf_solv = getTransform(robot,configSol,'endeffector','base');
Jac = geometricJacobian(robot, configSol, 'endeffector');
Jsi = pseudo_inv(Jac);
% First need to get th dot
V = rand(6,1);
P = rand(3,2);
F = rand(6,1);
DMT = [15 0; 0 4];

th_dot = Jsi*V;
h_dot = P'*th_dot;
wm_dot = DMT*h_dot;

tao_joint = Jac'*F;
Psi = pseudo_inv(P);
f_tend = Psi*tao_joint;
tao_motor = DMT*h_dot;