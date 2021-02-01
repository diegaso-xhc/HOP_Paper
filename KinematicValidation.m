classdef KinematicValidation < handle
    properties
       rigid_body_model % This is the complete rigid model of the finger following the modeling from RST
       joint_lim % This is a matrix containing the lower and upper limits of each joint. Dimensions: n_joints x 2
       des_frame % This is the desired frame that the end effector has to reach
       des_vel % This is the desired twist at the end effector (6x1: (1:3,1) corresponds to w and (4:6,1) corresponds to v) with respect to a base frame
       des_wrench % This is the desired wrench at the end effector (6x1: (1:3,1) corresponds to m and (4:6,1) corresponds to f) with respect to a base frame
       initial_guess % This is the initial guess for the inverse kinematics calculation
       gen_inverse_kinematics % Generalized Inverse Kinematics objects
       curr_config % This is the current configuration of the rigid body model
       geom_jacobian % This is the geometric jacobian at a given configuration
       curr_endeff_frame % Current frame of the end effector
    end
    methods
        function obj = KinematicValidation(rigid_body_model, joint_lim, des_frame, des_vel, des_wrench)
            obj.rigid_body_model = rigid_body_model;
            obj.des_vel = des_vel;
            obj.des_wrench = des_wrench;
            obj.initial_guess = zeros(length(obj.rigid_body_model.NumBodies));
            obj.gen_inverse_kinematics = generalizedInverseKinematics('RigidBodyTree', obj.rigid_body_model, 'ConstraintInputs', {'joint', 'pose'});
            obj.joint_lim = constraintJointBounds(obj.rigid_body_model);
            obj.joint_lim.Bounds = joint_lim;
            obj.des_frame = constraintPoseTarget('endeffector');
            obj.des_frame.TargetTransform = des_frame;
            obj.des_frame.PositionTolerance = 0.002; % 2mm of tolerance from target position
            obj.curr_config = obj.initial_guess; % Initial configuration
        end
        function set_desired_frame(obj, des_frame)
            obj.des_frame.TargetTransform = des_frame;           
        end
        function run_inverse_kinematics(obj, initial_guess)
            if nargin == 1                
                obj.curr_config = obj.gen_inverse_kinematics(obj.initial_guess, obj.joint_lim, obj.des_frame); % If the user does not provide an initial joint positions guess
            else
                obj.curr_config = obj.gen_inverse_kinematics(initial_guess, obj.joint_lim, obj.des_frame);
            end
            obj.geom_jacobian = geometricJacobian(obj.rigid_body_model, obj.curr_config, 'endeffector'); % Geometric Jacobian with respect to the base frame 
        end
        function sq_error = get_square_error(obj)
            sq_error = obj.des_frame.TargetTransform - obj.curr_endeff_frame;
            sq_error = sq_error.*sq_error;
            sq_error = sum(sum(sq_error));                        
        end
        function get_endeff_frame(obj)
            obj.curr_endeff_frame = getTransform(obj.rigid_body_model, obj.curr_config, 'endeffector', 'base');            
        end
    end
    methods (Static)
        function rigid_body_model = build_rigid_body_model(name_links, name_joints, type, frames, home)        
            % This function calculates the rigid body model using the RST 
            rigid_body_model = rigidBodyTree;
            rigid_body_model.DataFormat = 'row'; % Allows users to modify joint values in vector form
            n_links = size(name_links, 2);            
            rigid_body_structure = cell(n_links, 3);  
            for  i = 1: n_links + 1
               if i < n_links + 1
                   rigid_body_structure{i, 1} = rigidBody(name_links{i}); % Build rigid bodies
                   rigid_body_structure{i, 2} = rigidBodyJoint(name_joints{i}, type{i}); % Assign joints to each rigid body
                   rigid_body_structure{i, 2}.HomePosition = home(i); % Initialize the position of the joint
                   rigid_body_structure{i, 3} = frames{i}; % Reference the frame of each joint
                   setFixedTransform(rigid_body_structure{i, 2}, rigid_body_structure{i, 3}); 
                   rigid_body_structure{i, 1}.Joint = rigid_body_structure{i, 2};
               else
                   rigid_body_structure{i, 1} = rigidBody('endeffector'); % Adding end effector
                   rigid_body_structure{i, 3} = frames{i}; % Adding the frame of the end effector                   
                   setFixedTransform(rigid_body_structure{i, 1}.Joint, rigid_body_structure{i, 3}); 
               end               
               if  i == 1
                   component = 'base';                  
               else
                   component = name_links{i - 1};
               end
               addBody(rigid_body_model, rigid_body_structure{i, 1}, component); % Add components to the rigid body model               
            end
        end
        function Jsi = pseudo_inv(J)
            % This function calculates the pseudo inverse of an input matrix. Depending
            % on the configuration of J, it follows two procedures. The first one for
            % the case where J is tall (rows > columns) and the seconf one when J is
            % fat (columns > rows)
            m = size(J, 1); % No. of rows
            n = size(J, 2); % No. of columns
            if n > m
                tn = J*J';
                Jsi = J'*(tn\eye(size(tn, 1)));
            elseif n < m
                tn = J'*J;
                Jsi = (tn\eye(size(tn, 1)))*J';
            else
                Jsi = J\eye(n); % Simply the inverse of J
            end
        end
    end
    
    
end