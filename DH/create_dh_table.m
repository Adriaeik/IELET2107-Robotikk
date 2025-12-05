function dh_table = create_dh_table(joint_definitions)
% CREATE_DH_TABLE - Creates a flexible Denavit-Hartenberg table structure
%
% Usage:
%   dh_table = create_dh_table(joint_definitions)
%
% Input:
%   joint_definitions - Cell array where each cell defines one joint with:
%                      {theta, d, a, alpha} parameters
%
% Each parameter can be defined as:
%   - Scalar value: constant parameter
%   - Structure with fields:
%     * value: initial/constant value
%     * variable: true/false (can this parameter change?)
%     * type: 'rotation' or 'translation' (for default ranges)
%     * range: [min, max] (custom range, overrides default)
%
% Default ranges:
%   - Rotation: [-2*pi, 2*pi] radians
%   - Translation: [-2, 2] units
%
% Example usage:
%   % Define a 3-DOF robot
%   joints = {
%       % Joint 1: variable theta, fixed d=1, fixed a=0, fixed alpha=pi/2
%       {struct('value', 0, 'variable', true, 'type', 'rotation'), ...
%        1, 0, pi/2}, ...
%       
%       % Joint 2: variable theta, fixed d=0, fixed a=1, fixed alpha=0  
%       {struct('value', 0, 'variable', true, 'type', 'rotation'), ...
%        0, 1, 0}, ...
%        
%       % Joint 3: fixed theta=0, variable d, fixed a=0, fixed alpha=0
%       {0, struct('value', 1, 'variable', true, 'type', 'translation'), ...
%        0, 0}
%   };
%   dh_table = create_dh_table(joints);

    % Default ranges
    default_rotation_range = [-2*pi, 2*pi];
    default_translation_range = [-2, 2];
    
    % Initialize DH table structure
    num_joints = length(joint_definitions);
    dh_table = struct();
    dh_table.num_joints = num_joints;
    dh_table.joints = [];
    dh_table.variable_params = [];
    dh_table.param_info = struct('names', {{}}, 'ranges', [], 'types', {{}}, 'indices', []);
    
    param_counter = 1;
    
    % Process each joint
    for i = 1:num_joints
        joint_def = joint_definitions{i};
        
        if length(joint_def) ~= 4
            error('Each joint must have exactly 4 DH parameters: {theta, d, a, alpha}');
        end
        
        % Initialize joint structure
        joint = struct();
        joint.index = i;
        joint.theta = process_parameter(joint_def{1}, 'rotation', sprintf('θ%d', i));
        joint.d = process_parameter(joint_def{2}, 'translation', sprintf('d%d', i));
        joint.a = process_parameter(joint_def{3}, 'translation', sprintf('a%d', i));
        joint.alpha = process_parameter(joint_def{4}, 'rotation', sprintf('α%d', i));
        
        % Store joint
        dh_table.joints = [dh_table.joints; joint];
        
        % Collect variable parameters
        params = {'theta', 'd', 'a', 'alpha'};
        for j = 1:4
            param = params{j};
            if joint.(param).variable
                dh_table.param_info.names{param_counter} = joint.(param).name;
                dh_table.param_info.ranges(param_counter, :) = joint.(param).range;
                dh_table.param_info.types{param_counter} = joint.(param).type;
                dh_table.param_info.indices(param_counter, :) = [i, j]; % [joint_index, param_index]
                param_counter = param_counter + 1;
            end
        end
    end
    
    dh_table.num_variables = param_counter - 1;
    
    function param_struct = process_parameter(param_def, default_type, param_name)
        % Process a single DH parameter definition
        
        if isstruct(param_def)
            % Variable parameter
            param_struct = param_def;
            param_struct.name = param_name;
            
            % Set defaults for missing fields
            if ~isfield(param_struct, 'variable')
                param_struct.variable = true;
            end
            if ~isfield(param_struct, 'type')
                param_struct.type = default_type;
            end
            if ~isfield(param_struct, 'range')
                if strcmp(param_struct.type, 'rotation')
                    param_struct.range = default_rotation_range;
                else
                    param_struct.range = default_translation_range;
                end
            end
        else
            % Constant parameter
            param_struct = struct();
            param_struct.value = param_def;
            param_struct.variable = false;
            param_struct.type = default_type;
            param_struct.range = [param_def, param_def];
            param_struct.name = param_name;
        end
    end
end
