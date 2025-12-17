classdef Constants
    %FUNCTIONWRAPPER A class to abstract two functions with numeric input/output.
    %   This class holds two function handles and allows calling them
    %   through a unified 'calculate' method.
    
    
    properties
        g = 9.81;       % Gravity (m/s^2) - ACCELERATION CONSTANT
        L = 1.531;        % Wheelbase (m) - VEHICLE CONSTANT
        h = 0.256;        % CG height (m) - VEHICLE CONSTANT
        M = 262; % Weight (kg) - VEHICLE CONSTANT
        Lf = 1.531 * .491;  % Distance CG to front axle (m) - VEHICLE CONSTANT
        Lr = 1.531 * .509;   % Distance CG to rear axle (m) - VEHICLE CONSTANT (Note: L_front + L_rear = L)
        rho_front = 0.5936; % Front Aerodynamic factor - AERODYNAMIC CONSTANT
        rho_rear = 0.5152;  % Rear Aerodynamic factor - AERODYNAMIC CONSTANT
        drag_front = 0.088;
        drag_rear = 0.25;
        rear_wing_height = 1;

        tire_pressure = 80000; % Tire Pressure (Pa)

        grip_factor = 0.65; % Factor to reduce grip by, to reflect that the testing environment has higher grip than reality.

        v_min = 5; % Plot graphs starting from this factor, to avoid anomalies in the tire graph.
        

        tir_file = "C:/Users/jh/Downloads/43105_18x7.5_10_R25B_7.tir";
        tir_params;
    end

    methods
        function obj = Constants(varargin)
        % CONSTRUCTOR
            % Usage 1: obj = Constants() -> Uses all default values
            % Usage 2: obj = Constants('Name', Value, ...) -> Overwrites specific values
            
            if nargin > 0
                % Iterate through Name-Value pairs
                for i = 1:2:length(varargin)
                    propName = varargin{i};
                    propVal = varargin{i+1};
                    
                    % Check if the property exists in the class before assigning
                    if isprop(obj, propName)
                        obj.(propName) = propVal;
                    else
                        warning('Property "%s" does not exist in Constants class.', propName);
                    end
                end
            end
            
            % Load TIR params AFTER setting properties (in case tir_file was changed)
            % 
            if exist(obj.tir_file, 'file')
                 obj.tir_params = mfeval.readTIR(obj.tir_file);
            else
                 warning('TIR file not found: %s. params not loaded.', obj.tir_file);
            end
        end
    end
end