classdef KinematicValidation < handle
    properties
       rigid_body_model % This is the complete rigid model of the finger following the modeling from RST
       joint_lim % This is a matrix containing the lower and upper limits of each joint. Dimensions: n_joints x 2
       des_frame % This is the desired frame that the end effector has to reach
       des_vel % This is the desired twist at the end effector (6x1: (1:3,1) corresponds to w and (4:6,1) corresponds to v) with respect to a base frame
       des_wrench % This is the desired wrench at the end effector (6x1: (1:3,1) corresponds to m and (4:6,1) corresponds to f) with respect to a base frame
       initial_guess % This is the initial guess for the inverse kinematics calculation
       gen_inverse_kinematics % Generalized Inverse Kinematics objects (it allows the specification of joint limits)
       inverse_kinematics % Inverse Kinematics objects (it does not allow the specification of joint limits)
       curr_config % This is the current configuration of the rigid body model
       geom_jacobian % This is the geometric jacobian at a given configuration
       curr_endeff_frame % Current frame of the end effector
       curr_vel % This is the current twist at the end effector (w.r.t. base frame)
       curr_wrench % This is the current wrench at the end effector (w.r.t. base frame)       
       curr_joint_speed % Current joint speed
       curr_tendon_speed % Current tendon speed
       curr_motor_speed % Current motor speed
       curr_joint_torque % Current joint torque
       curr_tendon_force % Current tendon force
       curr_motor_torque % Current motor torque
    end
    methods
        function obj = KinematicValidation(rigid_body_model, joint_lim, des_frame, des_vel, des_wrench)
            obj.rigid_body_model = rigid_body_model;
            obj.des_vel = des_vel;
            obj.des_wrench = des_wrench;
            obj.initial_guess = zeros(1, obj.rigid_body_model.NumBodies - 1); % Initial guess is a zero value on each joint
            obj.gen_inverse_kinematics = generalizedInverseKinematics('RigidBodyTree', obj.rigid_body_model, 'ConstraintInputs', {'joint', 'pose'}); % Initialize an inverse kinematics object
            obj.inverse_kinematics = inverseKinematics('RigidBodyTree', obj.rigid_body_model); % Initialize the inverse kinematics object
            obj.joint_lim = constraintJointBounds(obj.rigid_body_model); % Joint limits
            obj.joint_lim.Bounds = joint_lim; % Joint limits
            obj.des_frame = constraintPoseTarget('endeffector');
            obj.des_frame.TargetTransform = des_frame; % Desired frame for end effector
            obj.des_frame.PositionTolerance = 0.002; % 2mm of tolerance from target position
            obj.curr_config = obj.initial_guess; % Initial configuration
            run_forward_kinematics(obj, obj.curr_config); % Initial end effector frame
        end
        function set_desired_frame(obj, des_frame)
            obj.des_frame.TargetTransform = des_frame; % Desired frame for end effector           
        end
        function run_inverse_kinematics(obj, type, initial_guess)
            % Type determines the inverse_kinematics object to use. For
            % faster iterations use type 2. For accurate, within bounds,
            % results use type 1
            if nargin == 2   
                % Use the original initial guess vector                
                if type == 1                     
                   obj.curr_config = obj.gen_inverse_kinematics(obj.initial_guess, obj.joint_lim, obj.des_frame); % If the user does not provide an initial joint positions guess
                else
                   obj.curr_config = obj.inverse_kinematics('endeffector', obj.des_frame.TargetTransform, [1 1 1 1 1 1], obj.initialguess); 
                end
            else
                % If there is an initial_guess vector as input
                if type == 1
                   obj.curr_config = obj.gen_inverse_kinematics(initial_guess, obj.joint_lim, obj.des_frame);
                else
                   obj.curr_config = obj.inverse_kinematics('endeffector', obj.des_frame.TargetTransform, [1 1 1 1 1 1], initial_guess);  
                end
            end
            obj.geom_jacobian = geometricJacobian(obj.rigid_body_model, obj.curr_config, 'endeffector'); % Geometric Jacobian with respect to the base frame 
        end
        function run_forward_kinematics(obj, joint_vector)
            % This function calculates the forward kinematics of a given
            % rigid body model and returns the homogenous transformation
            % matrix of the end effector represented on the base frame
            if nargin == 1
                % Get frame with currently recorded joint configuration                
                obj.curr_endeff_frame = getTransform(obj.rigid_body_model, obj.curr_config, 'endeffector', 'base');
            else
                % If there is an input joint_vector
                obj.curr_endeff_frame = getTransform(obj.rigid_body_model, joint_vector, 'endeffector', 'base');                
            end            
        end
        function back_fwd_calculation_loop(obj, initial_guess, tendon_coupl_model,  drive_train_model, type)
            if nargin == 5
                run_inverse_kinematics(obj, type, initial_guess)
            else
                run_inverse_kinematics(obj, type)
            end
            Jsi = pseudo_inv(obj.geom_jacobian); % Pseudo Inverse or simply inverse of the geometric jacobian
            drive_train_si = pseudo_inv(drive_train_model); % Pseudo Inverse or simply inverse of the drive train model
            %%%%%%%%%%%%%%%%%% Backward calculation %%%%%%%%%%%%%%%%%%%%%%%
            %%%% Velocity section            
            q_dot = Jsi*obj.des_vel; % Joint speed
            h_dot = tendon_coupl_model'*q_dot; % Tendon speed
            w_motor = drive_train_si*h_dot; % From tendon speed to motor speed
            %%%%%%%%%%%%%%%%%%%%%%
            %%%% Wrench section
            tao_joint = obj.geom_jacobian'*obj.des_wrench;
            f_tendon = pseudo_inv(tendon_coupl_model)*tao_joint;
            tao_motor = drive_train_si*f_tendon; % From tendon force to motor torque            
            %%%%%%%%%%%%%%%%%%%%%%            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %%%%%%%%%%% Forward calculation and optimization %%%%%%%%%%%%%%
            %%%% Velocity section   
            G_of_th = obj.geom_jacobian*pseudo_inv(tendon_coupl_model')*drive_train_model;
            curr_vel = G_of_th*w_motor; % Resulting twist from forward calculation
                                        % G is a matrix simplifying the
                                        % operation used for the optimization
            min_val = (w_motor - 2)';
            max_val = (w_motor + 2)';    
            w_motor_opt = lsqlin(G_of_th, obj.des_vel, eye(size(G_of_th, 2)), max_val'.*ones(size(G_of_th, 2), 1)); % Using the optimization toolbox
%             w_motor_opt = KinematicValidation.call_pso(length(w_motor), G_of_th, obj.des_vel, min_val, max_val, 0);           
%             w_motor_opt = w_motor_opt.bestSolution.Position'; % This position has to do with PSO not with the robot
            obj.curr_vel = G_of_th*w_motor_opt; % Actual twist at end effector
            obj.curr_motor_speed = w_motor_opt; % Assigning optimized motor speed
            obj.curr_tendon_speed = drive_train_model*obj.curr_motor_speed;
            obj.curr_joint_speed = pseudo_inv(tendon_coupl_model')*obj.curr_tendon_speed;          
            %%%%%%%%%%%%%%%%%%%%%%
            %%%% Wrench section
            B_of_th = Jsi'*tendon_coupl_model*drive_train_model;
            curr_wrench = B_of_th*tao_motor; % Resulting wrench from forward calculation
                                             % B is a matrix simplifying the
                                             % operation used for the optimization            
            min_val = (tao_motor - 2)';
            max_val = (tao_motor + 2)';
            tao_motor_opt = lsqlin(B_of_th, obj.des_wrench, eye(size(B_of_th, 2)), max_val'.*ones(size(B_of_th, 2), 1)); % Using the optimization toolbox
%             tao_motor_opt = KinematicValidation.call_pso(length(tao_motor), B_of_th, obj.des_wrench, min_val, max_val, 0);
%             tao_motor_opt = tao_motor_opt.bestSolution.Position'; % This position has to do with the PSO algorithm not with the robot
            obj.curr_wrench = B_of_th*tao_motor_opt; % Actual wrench at end effector
            obj.curr_motor_torque = tao_motor_opt; % Assigning optimized motor speed
            obj.curr_tendon_force = drive_train_model*obj.curr_motor_torque;
            obj.curr_joint_torque = tendon_coupl_model*obj.curr_tendon_force;
            get_endeff_frame(obj) % Finally assign the current end effector frame
            %%%%%%%%%%%%%%%%%%%%%%            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%            
        end       
        function sq_pos_error = get_pos_square_error(obj)
            % This function calculates the square error between the
            % current end effector frame and the desired frame
            sq_pos_error = obj.des_frame.TargetTransform - obj.curr_endeff_frame;
            sq_pos_error = sq_pos_error.*sq_pos_error;
            sq_pos_error = sum(sum(sq_pos_error));                        
        end
        function sq_vel_error = get_vel_square_error(obj)
            % This function calculates the square error between the
            % current end effector twist and the desired twist
            sq_vel_error = obj.curr_vel - obj.des_vel;
            sq_vel_error = sq_vel_error.*sq_vel_error;
            sq_vel_error = sum(sum(sq_vel_error));
        end
        function sq_wrench_error = get_wrench_square_error(obj)
            % This function calculates the square error between the
            % current end effector wrench and the desired wrench
            sq_wrench_error = obj.curr_wrench - obj.des_wrench;
            sq_wrench_error = sq_wrench_error.*sq_wrench_error;
            sq_wrench_error = sum(sum(sq_wrench_error));
        end
        function get_endeff_frame(obj)
            % This function returns the current end effector's frame
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
        function out = call_pso(nVar, matrix, exp_result, min_val, max_val, disp)
            %%%% Problem Definition            
            probl.CostFunction = @(x, G, V) cost(x, G, V); % Cost Function
            probl.nVar = nVar; % Number of unknown (decision) variables
            probl.varSize = [1 probl.nVar]; % Matrix size of decision variables
            probl.varMin = min_val; % Lower bound of decision variables
            probl.varMax = max_val; % Upper bound of decision variables                       
            %%%% Parameters of PSO            
            % Parameters for the pso algorithm (Clerc and Kennedy, 2002)
            kappa = 1;
            phi1 = 2.05;
            phi2 = 2.05;
            phi = phi1 + phi2;
            chi = 2*kappa/abs(2 - phi - sqrt(phi*phi - 4*phi));
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%            
            param.maxIt = 100; % Maximum number of iterations
            param.nPop = 50; % Population size or swarm size
            param.w = chi; % Intertia coefficient
            param.wdamp = 1; % Damping ratio of Inertia weight
            param.c1 = chi*phi1; % Personal acceleration coefficient
            param.c2 = chi*phi2; % Social acceleration coefficient
            param.displ = disp; % This is the flag to display the information            
            %%%% PSO main loop            
            out = pso_opt(probl, param, matrix, exp_result);            
            %%%% Optional plotting
            if param.displ == 1
                plot(out.bestCosts, 'r')
                title('Iterations over time')
                xlabel('Iterations')
                ylabel('Cost Function')
            end
        end
    end
end