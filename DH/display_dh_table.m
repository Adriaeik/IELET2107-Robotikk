function display_dh_table(dh_table)
% DISPLAY_DH_TABLE - Pretty print the DH table information
%
% Usage:
%   display_dh_table(dh_table)

    fprintf('\n=== DH Table Information ===\n');
    fprintf('Number of joints: %d\n', dh_table.num_joints);
    fprintf('Number of variable parameters: %d\n\n', dh_table.num_variables);
    
    % Display table header
    fprintf('Joint |   θ   |   d   |   a   |   α   |\n');
    fprintf('------|-------|-------|-------|-------|\n');
    
    % Display each joint
    for i = 1:dh_table.num_joints
        joint = dh_table.joints(i);
        fprintf('  %d   | %s | %s | %s | %s |\n', i, ...
                format_param(joint.theta), ...
                format_param(joint.d), ...
                format_param(joint.a), ...
                format_param(joint.alpha));
    end
    
    % Display variable parameters
    if dh_table.num_variables > 0
        fprintf('\n=== Variable Parameters ===\n');
        for i = 1:dh_table.num_variables
            name = dh_table.param_info.names{i};
            range = dh_table.param_info.ranges(i, :);
            type = dh_table.param_info.types{i};
            fprintf('%s: [%.3f, %.3f] (%s)\n', name, range(1), range(2), type);
        end
    end
    
    function str = format_param(param)
        if param.variable
            str = sprintf('%s*', param.name);
        else
            str = sprintf('%.2f', param.value);
        end
        str = pad(str, 5, 'both');
    end
end

function values = get_variable_values(dh_table)
% GET_VARIABLE_VALUES - Extract current values of all variable parameters
%
% Usage:
%   values = get_variable_values(dh_table)
%
% Output:
%   values - Array of current values for variable parameters

    values = zeros(1, dh_table.num_variables);
    
    for i = 1:dh_table.num_variables
        joint_idx = dh_table.param_info.indices(i, 1);
        param_idx = dh_table.param_info.indices(i, 2);
        
        params = {'theta', 'd', 'a', 'alpha'};
        param_name = params{param_idx};
        
        values(i) = dh_table.joints(joint_idx).(param_name).value;
    end
end

function dh_table = set_variable_values(dh_table, values)
% SET_VARIABLE_VALUES - Update values of variable parameters
%
% Usage:
%   dh_table = set_variable_values(dh_table, values)
%
% Input:
%   values - Array of new values for variable parameters

    if length(values) ~= dh_table.num_variables
        error('Number of values (%d) must match number of variable parameters (%d)', ...
              length(values), dh_table.num_variables);
    end
    
    for i = 1:dh_table.num_variables
        joint_idx = dh_table.param_info.indices(i, 1);
        param_idx = dh_table.param_info.indices(i, 2);
        
        params = {'theta', 'd', 'a', 'alpha'};
        param_name = params{param_idx};
        
        % Clamp value to range
        range = dh_table.param_info.ranges(i, :);
        value = max(min(values(i), range(2)), range(1));
        
        dh_table.joints(joint_idx).(param_name).value = value;
    end
end

function A = compute_dh_transform(joint)
% COMPUTE_DH_TRANSFORM - Compute transformation matrix for a joint
%
% Usage:
%   A = compute_dh_transform(joint)

    theta = joint.theta.value;
    d = joint.d.value;
    a = joint.a.value;
    alpha = joint.alpha.value;
    
    ct = cos(theta); st = sin(theta);
    ca = cos(alpha); sa = sin(alpha);
    
    A = [ct, -st*ca,  st*sa, a*ct;
         st,  ct*ca, -ct*sa, a*st;
         0,   sa,     ca,    d;
         0,   0,      0,     1];
end

function transforms = compute_forward_kinematics(dh_table)
% COMPUTE_FORWARD_KINEMATICS - Compute all transformation matrices
%
% Usage:
%   transforms = compute_forward_kinematics(dh_table)
%
% Output:
%   transforms.A - Cell array of individual joint transforms A1, A2, ...
%   transforms.T - Cell array of cumulative transforms T01, T02, ...

    n = dh_table.num_joints;
    transforms.A = cell(n, 1);
    transforms.T = cell(n+1, 1);
    
    % Base frame
    transforms.T{1} = eye(4);
    
    % Compute each joint transformation
    for i = 1:n
        transforms.A{i} = compute_dh_transform(dh_table.joints(i));
        transforms.T{i+1} = transforms.T{i} * transforms.A{i};
    end
end

% Example usage and test cases
function run_examples()
    fprintf('=== DH Table Creator Examples ===\n\n');
    
    % Example 1: Simple 2-DOF planar robot
    fprintf('Example 1: 2-DOF Planar Robot\n');
    joints_2dof = {
        % Joint 1: variable theta, fixed d=0, fixed a=1, fixed alpha=0
        {struct('value', 0, 'variable', true, 'type', 'rotation'), 0, 1, 0}, ...
        % Joint 2: variable theta, fixed d=0, fixed a=1, fixed alpha=0
        {struct('value', 0, 'variable', true, 'type', 'rotation'), 0, 1, 0}
    };
    dh_2dof = create_dh_table(joints_2dof);
    display_dh_table(dh_2dof);
    
    % Example 2: SCARA-like robot
    fprintf('\nExample 2: SCARA-like Robot\n');
    joints_scara = {
        % Joint 1: variable theta, fixed d=1, fixed a=0.5, fixed alpha=0
        {struct('value', 0, 'variable', true, 'type', 'rotation'), 1, 0.5, 0}, ...
        % Joint 2: variable theta, fixed d=0, fixed a=0.5, fixed alpha=0
        {struct('value', 0, 'variable', true, 'type', 'rotation'), 0, 0.5, 0}, ...
        % Joint 3: fixed theta=0, variable d, fixed a=0, fixed alpha=0
        {0, struct('value', 0.2, 'variable', true, 'type', 'translation', 'range', [0, 0.5]), 0, 0}
    };
    dh_scara = create_dh_table(joints_scara);
    display_dh_table(dh_scara);
    
    % Example 3: 6-DOF manipulator with custom ranges
    fprintf('\nExample 3: 6-DOF Manipulator\n');
    joints_6dof = {
        {struct('value', 0, 'variable', true, 'type', 'rotation'), 0.5, 0, pi/2}, ...
        {struct('value', 0, 'variable', true, 'type', 'rotation', 'range', [-pi/2, pi/2]), 0, 0.4, 0}, ...
        {struct('value', 0, 'variable', true, 'type', 'rotation'), 0, 0.3, pi/2}, ...
        {struct('value', 0, 'variable', true, 'type', 'rotation'), 0.4, 0, -pi/2}, ...
        {struct('value', 0, 'variable', true, 'type', 'rotation'), 0, 0, pi/2}, ...
        {struct('value', 0, 'variable', true, 'type', 'rotation'), 0.1, 0, 0}
    };
    dh_6dof = create_dh_table(joints_6dof);
    display_dh_table(dh_6dof);
    
    % Test forward kinematics
    fprintf('\n=== Testing Forward Kinematics ===\n');
    transforms = compute_forward_kinematics(dh_2dof);
    fprintf('End-effector position for 2-DOF robot at zero config:\n');
    fprintf('x = %.3f, y = %.3f, z = %.3f\n', ...
            transforms.T{end}(1,4), transforms.T{end}(2,4), transforms.T{end}(3,4));
end