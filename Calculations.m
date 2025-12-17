classdef Calculations
    %FUNCTIONWRAPPER A class to abstract two functions with numeric input/output.
    %   This class holds two function handles and allows calling them
    %   through a unified 'calculate' method.
    properties
        ConstantVars Constants;
    end
    
    methods (Access = private)
        function N = FrontAxleRestingMass(obj)
            N = obj.ConstantVars.M * obj.ConstantVars.g * obj.ConstantVars.Lr / obj.ConstantVars.L;
        end

        function N = RearAxleRestingMass(obj)
            N = obj.ConstantVars.M * obj.ConstantVars.g * obj.ConstantVars.Lf / obj.ConstantVars.L;
        end

        function N = LoadTransfer(obj, d)
            N = obj.ConstantVars.M * d * obj.ConstantVars.h / obj.ConstantVars.L;
        end

        function N = DragLoadTransfer(obj, v)
            N = obj.ConstantVars.drag_rear * v * v * obj.ConstantVars.rear_wing_height / obj.ConstantVars.L;
        end

        function N = FrontAxleDownforce(obj, v)
            N = obj.ConstantVars.rho_front * v * v;
        end

        function N = RearAxleDownforce(obj, v)
            N = obj.ConstantVars.rho_rear * v * v;
        end

        function N = DragForce(obj, v)
            N = (obj.ConstantVars.drag_front + obj.ConstantVars.drag_rear) * v * v;
        end

        function [Fx0, mux] = WheelLongitudinalGrip(obj, Fz, kappa, alpha, gamma, phit, Vcx)
            %vertical load, long slip, side slip, inclination angle, turn
            %slip, velocity
            output = mfeval(obj.ConstantVars.tir_params, ...
                [Fz, kappa, alpha, gamma, phit, Vcx, obj.ConstantVars.tire_pressure], ...
                111);
            Fx0 = output(:,1);
            mux = output(:,17);

        end

        function [Fx0_max, kappa_at_max] = PeakWheelLongitudinalGrip(obj, Fz_scalar, alpha, gamma, phit, Vx)
            %GEMINI DEBUGGED%
            % Finds the peak Fx0 across the entire range of longitudinal slips (kappa) 
            % for a given vertical load Fz_scalar (Single Wheel).
            
            % --- 1. Define Sweep Range and Input Size ---
            nPoints = 101; % Use a good number of points for a smooth curve
            kappa_sweep = linspace(-1.0, 1.0, nPoints); % Sweep from -100% to 100% slip

            % --- 2. Create Consistent Input Matrix for mfeval ---
            % All columns MUST be nPoints x 1 arrays
            Fz      = ones(nPoints, 1) * Fz_scalar; % Vertical load array
            kappa	= kappa_sweep';                 % Longitudinal slip (MUST be nPoints x 1)
            alpha	= ones(nPoints, 1) * alpha;     % Side slip angle array
            gamma	= ones(nPoints, 1) * gamma;     % Inclination angle array
            phit 	= ones(nPoints, 1) * phit;      % Turnslip array
            Vx   	= ones(nPoints, 1) * Vx;        % Forward velocity array
            P = ones(nPoints, 1) * obj.ConstantVars.tire_pressure;

            % --- 3. Evaluate the Tire Model ---
            output = mfeval(obj.ConstantVars.tir_params, ...
                [Fz, kappa, alpha, gamma, phit, Vx, P], ...
                111); % The 111 code is for Fx0 calculation

            % --- 4. Extract Fx0 (usually the 1st column of output for code 111) ---
            Fx0 = output(:, 1);

            % --- 5. Find the Peak Grip ---
            [Fx0_max, idx] = max(Fx0);
            kappa_at_max = kappa_sweep(idx);
            
            % figure;
            % plot(kappa_sweep, Fx0, 'r');
        end

        function [Fx0, kappa] = AxleNormalToAxleGrip(obj, N, v)
            [Fx0, kappa] = obj.PeakWheelLongitudinalGrip(N / 2, 0, 0, 0, v);
            Fx0 = Fx0 * 2 * obj.ConstantVars.grip_factor;
        end

        function G = GripMinusRequiredForce(obj, v, d)
            %GEMINI GENERATED%
            % Helper function for the root finder: G(d) = (F + R) - M * d
            % F and R are the maximum available grips from AxleGrips.
            [F, R] = obj.AxleGrips(v, d);
            G = (F + R) + obj.DragForce(v) - obj.ConstantVars.M * d;
        end

        function G = GripMinusRequiredForceBBal(obj, v, d, BBal)
            %GEMINI GENERATED%
            % Helper function for the root finder: G(d) = (F + R) - M * d
            % F and R are the maximum available grips from AxleGrips.
            [F, R] = obj.AxleGrips(v, d);
            if F / BBal * (1 - BBal) > R
                % Rear lock up, Rear limited.
                F = R / (1 - BBal) * BBal;
            else
                R = F / BBal * (1 - BBal);
            end
            G = (F + R) + obj.DragForce(v) - obj.ConstantVars.M * d;
        end
        
    end

    methods
        function obj = Calculations()
            obj.ConstantVars = Constants();
        end

        function [F, R] = AxleNormals(obj, v, d)
            F = obj.FrontAxleDownforce(v) + obj.FrontAxleRestingMass() + obj.LoadTransfer(d) - obj.DragLoadTransfer(v);
            R = obj.RearAxleDownforce(v) + obj.RearAxleRestingMass() - obj.LoadTransfer(d) + obj.DragLoadTransfer(v);
        end

        function [F, R] = AxleNormalsWithoutAero(obj, d)
            F = obj.FrontAxleRestingMass() + obj.LoadTransfer(d);
            R = obj.RearAxleRestingMass() - obj.LoadTransfer(d);
        end

        function [F, R] = AxleGrips(obj, v, d)
            [F, R] = obj.AxleNormals(v, d);
            if (F <= 0.1 || R <= 0.1)
                F = 0;
                R = 0;
            else
                % disp([F, R, v, d]);
                F = obj.AxleNormalToAxleGrip(F, v);
                R = obj.AxleNormalToAxleGrip(R, v);
            end
        end

        function [F, R] = AxleGripsWithoutAero(obj, v, d)
            [F, R] = obj.AxleNormalsWithoutAero(d);
            if (F <= 0.1 || R <= 0.1)
                F = 0;
                R = 0;
            else
                % disp([F, R, v, d]);
                F = obj.AxleNormalToAxleGrip(F, v);
                R = obj.AxleNormalToAxleGrip(R, v);
            end
        end

        function [d_peak, Bbal] = PeakDeceleration(obj, v)

            targetFunction = @(d) obj.GripMinusRequiredForce(v, d);
        
            d_min = 0.01;
            d_max = 30;
            G_min = targetFunction(d_min);
            G_max = targetFunction(d_max);
            
            % Note: d must be positive for braking/deceleration.
            options = optimset('Display', 'off');
            if sign(G_min) ~= sign(G_max)
                % A sign change exists, root finding should be stable.
                d_peak = fzero(targetFunction, [d_min, d_max], options);
            else
                try
                    d_guess = 9.81;
                    d_peak = fzero(targetFunction, d_guess, options);
                catch ME
                    warning('Calculations:RootNotFound', ...
                        'Failed to find the deceleration root for v=%.2f. Error: %s', v, ME.message);
                    d_peak = NaN;
                end
            end
            [F, R] = obj.AxleGrips(v, d_peak);
            % disp([F, R]);
            Bbal = F / (F + R);
        end

        function [d_peak] = PeakDecelerationFixedBBal(obj, v, BBal)

            targetFunction = @(d) obj.GripMinusRequiredForceBBal(v, d, BBal);
        
            d_min = 0.01;
            d_max = 30;
            G_min = targetFunction(d_min);
            G_max = targetFunction(d_max);
            
            % Note: d must be positive for braking/deceleration.
            options = optimset('Display', 'off');
            if sign(G_min) ~= sign(G_max)
                % A sign change exists, root finding should be stable.
                d_peak = fzero(targetFunction, [d_min, d_max], options);
            else
                try
                    d_guess = 9.81;
                    d_peak = fzero(targetFunction, d_guess, options);
                catch ME
                    warning('Calculations:RootNotFound', ...
                        'Failed to find the deceleration root for v=%.2f. Error: %s', v, ME.message);
                    d_peak = NaN;
                end
            end
        end

        function PlotAxleGripsVsDeceleration(obj, v, d_max)
            % PlotAxleGripsVsDeceleration Plots the front and rear axle grips 
            % (F and R) as a function of deceleration (d).
            %
            % Inputs:
            %   v: The constant forward velocity (m/s).
            %   d_max: The maximum deceleration to plot up to (e.g., 25 m/s^2).
            
            % --- 1. Define the Range for d ---
            n_points = 100; % Number of points for a smooth curve
            d_range = linspace(0, d_max, n_points);

            % --- 2. Initialize Output Arrays ---
            F_grips = zeros(1, n_points); % Front Axle Grip
            R_grips = zeros(1, n_points); % Rear Axle Grip

            % --- 3. Iterate and Calculate Grips ---
            for i = 1:n_points
                d = d_range(i);
                
                % Call the AxleGrips method
                [F, R] = obj.AxleGrips(v, d);
                
                F_grips(i) = F;
                R_grips(i) = R;
            end
            
            % --- 4. Plot the Results ---
            figure; % Open a new figure window
            hold on; % Allow multiple lines on the same plot
            
            % Plot Front Axle Grip (F) vs. Deceleration (d)
            plot(d_range, F_grips, 'b-', 'LineWidth', 2, 'DisplayName', 'Front Axle Grip (F)');
            
            % Plot Rear Axle Grip (R) vs. Deceleration (d)
            plot(d_range, R_grips, 'r-', 'LineWidth', 2, 'DisplayName', 'Rear Axle Grip (R)');
            
            plot(d_range, F_grips + R_grips, 'g--', 'LineWidth', 2, 'DisplayName', 'Combined Axle Grip (R)');
            
            % Plot the Total Required Force (M * d) for comparison
            M = obj.ConstantVars.M;
            Total_Required_Force = M * d_range - obj.DragForce(v);
            plot(d_range, Total_Required_Force, 'k:', 'LineWidth', 1.5, 'DisplayName', 'Required Braking Force (M \cdot d)');
            
            % --- 5. Annotate the Plot ---
            xlabel('Deceleration, $d$ (m/s$^2$)', 'Interpreter', 'latex');
            ylabel('Maximum Longitudinal Grip (N)', 'Interpreter', 'latex');
            title_str = sprintf('Maximum Axle Grips vs. Deceleration (at v = %.1f m/s)', v);
            title(title_str);
            grid on;
            legend('Location', 'northwest');
            hold off;
        end

        function PlotGripVsNormalForce(obj, v, Fz_max)
            % PlotGripVsNormalForce Plots the maximum longitudinal grip (Fx0_max) 
            % for a single wheel as a function of the Normal Force (Fz).
            %
            % This is a sanity check function, to make sure it's pulling
            % numbers roughly equivalent to those calculated by other
            % toolboxes.
            %
            % Inputs:
            %   v: The constant forward velocity (m/s).
            %   Fz_max: The maximum normal force (Vertical Load) to plot up to (N).
            
            % --- 1. Define the Range for Fz (Normal Force) ---
            n_points = 100; % Number of points for a smooth curve
            Fz_range = linspace(0, Fz_max, n_points);

            % --- 2. Initialize Output Array ---
            Fx0_max_values = zeros(1, n_points); % Maximum Longitudinal Grip

            % --- 3. Define Constant Tire Condition Parameters ---
            % Since we are plotting the PURE longitudinal grip curve, we set side
            % slip, inclination, and turn slip to zero.
            alpha = 0;   % Side slip angle (radians)
            gamma = 0;   % Inclination angle (radians)
            phit  = 0;   % Turn slip (1/m)

            % --- 4. Iterate and Calculate Maximum Grip ---
            for i = 1:n_points
                Fz = Fz_range(i);
                
                % Call the private method PeakWheelLongitudinalGrip
                [Fx0_max, ~] = obj.PeakWheelLongitudinalGrip(Fz, alpha, gamma, phit, v);
                
                Fx0_max_values(i) = Fx0_max;
            end
            
            % --- 5. Plot the Results ---
            figure; % Open a new figure window
            hold on;
            
            % Plot Fx0_max vs. Fz
            plot(Fz_range, Fx0_max_values, 'k-', 'LineWidth', 2, 'DisplayName', 'Maximum Longitudinal Grip, $F_{x0, max}$');
            
            % --- 6. Plot the Coefficient of Friction (Optional Helper) ---
            % The coefficient of friction (mu_x) is Fx0 / Fz.
            % We plot it on the same graph but use the secondary Y-axis (yyaxis).
            mu_x = Fx0_max_values ./ Fz_range;
            
            yyaxis right;
            plot(Fz_range, mu_x, 'g--', 'LineWidth', 1.5, 'DisplayName', 'Coefficient of Friction, $\mu_x$');
            
            % --- 7. Annotate the Plot ---
            yyaxis left;
            xlabel('Normal Force, $F_z$ (N)', 'Interpreter', 'latex');
            ylabel('Maximum Longitudinal Grip, $F_{x0, max}$ (N)', 'Interpreter', 'latex');
            
            yyaxis right;
            ylabel('Coefficient of Friction, $\mu_x = F_{x0, max} / F_z$', 'Interpreter', 'latex');
            
            title_str = sprintf('Maximum Single Wheel Grip vs. Normal Force (at v = %.1f m/s)', v);
            title(title_str);
            grid on;
            legend('Location', 'southeast');
            hold off;

        end

        function PlotPeakDecelerationVsSpeed(obj, v_max)
            % PlotPeakDecelerationVsSpeed Plots the maximum possible deceleration (d_peak)
            % as a function of forward velocity (v).
            %
            % Inputs:
            %   v_max: The maximum velocity (m/s) to plot up to.
            
            % --- 1. Define the Range for Velocity (v) ---
            n_points = 50; % Number of points for a smooth curve
            v_range = linspace(obj.ConstantVars.v_min, v_max, n_points); % Start slightly above zero to avoid division by zero/singularity
            
            % --- 2. Initialize Output Array ---
            d_peak_values = zeros(1, n_points); % Peak Deceleration
            Bbal_values = zeros(1, n_points);   % Brake Balance at peak
            
            % --- 3. Iterate and Calculate Peak Deceleration ---
            for i = 1:n_points
                v = v_range(i);
                
                % Call the PeakDeceleration method
                [d_peak, Bbal] = obj.PeakDeceleration(v);
                
                d_peak_values(i) = d_peak;
                Bbal_values(i) = Bbal;
            end
            
            % --- 4. Plot the Deceleration Results (Left Y-Axis) ---
            figure; % Open a new figure window
            hold on;
            
            % Plot Peak Deceleration (d_peak) vs. Velocity (v)
            plot(v_range, d_peak_values, 'b-', 'LineWidth', 2, 'DisplayName', 'Peak Deceleration, $d_{peak}$');
            
            
            % --- 5. Annotate the Deceleration Plot ---
            xlabel('Forward Velocity, $v$ (m/s)', 'Interpreter', 'latex');
            ylabel('Peak Deceleration, $d_{peak}$ (m/s$^2$)', 'Interpreter', 'latex');
            title_str = 'Maximum Braking Deceleration vs. Speed';
            title(title_str);
            grid on;
            
            % --- 6. Plot the Brake Balance Results (Right Y-Axis) ---
            % Brake Balance (Bbal) is often F / (F + R), a ratio, so it's good to plot it on a secondary axis.
            yyaxis right;
            plot(v_range, Bbal_values * 100, 'w:', 'LineWidth', 1.5, 'DisplayName', 'Optimal Brake Balance (\% Front)');
            ylabel('Optimal Front Brake Bias (\% Front)', 'Interpreter', 'latex');
            ylim([50 100]); % Brake balance is typically between 50% and 100%
            xlim([obj.ConstantVars.v_lim v_max]);
            
            % --- 7. Final Annotations ---
            yyaxis left; % Set primary axis back to left for legend placement
            legend('Location', 'southeast', 'Interpreter', 'latex');
            hold off;
            
            % The plot often shows d_peak decreasing as v increases due to
            % aerodynamic downforce (which increases the grip) becoming less effective
            % (or drag becoming more significant) and due to non-linear tire effects
            % (load sensitivity).
        end

        function PlotPeakDecelerationFixedBBalVsSpeed(obj, v_max, BBal_fixed)
            % PlotPeakDecelerationFixedBBalVsSpeed Plots the maximum possible deceleration (d_peak)
            % for a fixed brake bias (BBal_fixed) as a function of forward velocity (v).
            %
            % Inputs:
            %   v_max: The maximum velocity (m/s) to plot up to.
            %   BBal_fixed: The fixed front brake bias ratio (e.g., 0.65 for 65% front bias).
            
            % --- 1. Define the Range for Velocity (v) ---
            n_points = 50; % Number of points for a smooth curve
            v_range = linspace(obj.ConstantVars.v_min, v_max, n_points); % Start slightly above zero
            
            % --- 2. Initialize Output Array ---
            d_peak_fixed_values = zeros(1, n_points); % Peak Deceleration
            d_peak_ideal_values = zeros(1, n_points); % Ideal Peak Deceleration (for comparison)
            
            % --- 3. Iterate and Calculate Peak Deceleration ---
            for i = 1:n_points
                v = v_range(i);
                
                % a) Calculate Peak Deceleration for the FIXED Bias
                d_peak_fixed = obj.PeakDecelerationFixedBBal(v, BBal_fixed);
                
                % b) Calculate Peak Deceleration for the IDEAL (unconstrained) Bias
                [d_peak_ideal, ~] = obj.PeakDeceleration(v);
                
                d_peak_fixed_values(i) = d_peak_fixed;
                d_peak_ideal_values(i) = d_peak_ideal;
            end
            
            % --- 4. Plot the Results ---
            figure; % Open a new figure window
            hold on;
            
            % Plot Deceleration with Fixed Bias
            BBal_percent = BBal_fixed * 100;
            display_name_fixed = sprintf('Fixed Deceleration (%.1f%% Front Bias)', BBal_percent);
            plot(v_range, d_peak_fixed_values, 'b-', 'LineWidth', 2, 'DisplayName', display_name_fixed);
            
            % Plot Ideal Deceleration (unconstrained) for comparison
            plot(v_range, d_peak_ideal_values, 'r--', 'LineWidth', 1.5, 'DisplayName', 'Ideal Peak Deceleration');
            
            % --- 5. Annotate the Plot ---
            xlabel('Forward Velocity, $v$ (m/s)', 'Interpreter', 'latex');
            ylabel('Peak Deceleration, $d_{peak}$ (m/s$^2$)', 'Interpreter', 'latex');
            title_str = sprintf('Braking Deceleration vs. Speed for Fixed Brake Bias');
            title(title_str);
            grid on;
            legend('Location', 'southeast');
            hold off;
            
            % The difference between the 'Fixed' and 'Ideal' curves highlights the
            % performance lost due to non-optimal brake distribution.
        end

        function PlotBrakingDistanceVsSpeed(obj, v_max, BBal_fixed)
            % PlotBrakingDistanceVsSpeed Calculates and plots the braking distance (S)
            % for both ideal and fixed brake bias as a function of initial speed (v_initial).
            %
            % Inputs:
            %   v_max: The maximum initial velocity (m/s) to plot up to.
            %   BBal_fixed: The fixed front brake bias ratio (e.g., 0.65).
            
            % --- 1. Define the Sweep Range for Velocity (v) ---
            % This is the velocity *at which* we calculate the deceleration d(v).
            n_decel_points = 100; % High resolution for integration accuracy
            v_decel_range = linspace(1, v_max, n_decel_points);
            
            % --- 2. Calculate Deceleration Curves d(v) ---
            d_ideal = zeros(1, n_decel_points);
            d_fixed = zeros(1, n_decel_points);
            
            for i = 1:n_decel_points
                v = v_decel_range(i);
                
                % Ideal Deceleration
                [d_ideal(i), ~] = obj.PeakDeceleration(v);
                
                % Fixed Deceleration
                d_fixed(i) = obj.PeakDecelerationFixedBBal(v, BBal_fixed);
            end
            
            % Handle NaNs (e.g., if root finder fails at high speed)
            d_ideal(isnan(d_ideal)) = 0.01; 
            d_fixed(isnan(d_fixed)) = 0.01; 

            % --- 3. Integrate to Find Braking Distance S (v_initial) ---
            
            % The formula is S = integral(v / d(v) dv) from 0 to v_initial.
            % We can use the cumulative trapezoidal numerical integration (cumtrapz) 
            % on the integrand (v / d(v)).
            
            integrand_ideal = v_decel_range ./ d_ideal;
            integrand_fixed = v_decel_range ./ d_fixed;
            
            % cumtrapz integrates the integrand array with respect to the velocity array (v_decel_range)
            S_ideal = cumtrapz(v_decel_range, integrand_ideal);
            S_fixed = cumtrapz(v_decel_range, integrand_fixed);
            
            % --- 4. Plot the Braking Distance Results ---
            figure; % Open a new figure window
            hold on;
            
            % Plot Braking Distance with Ideal Bias
            plot(v_decel_range, S_ideal, 'r--', 'LineWidth', 1.5, 'DisplayName', 'Braking Distance (Ideal Bias)');
            
            % Plot Braking Distance with Fixed Bias
            BBal_percent = BBal_fixed * 100;
            display_name_fixed = sprintf('Braking Distance to 1 m/s (Fixed %.1f%% Front Bias)', BBal_percent);
            plot(v_decel_range, S_fixed, 'b-', 'LineWidth', 2, 'DisplayName', display_name_fixed);
            
            % --- 5. Annotate the Plot ---
            xlabel('Initial Velocity, $v_{initial}$ (m/s)', 'Interpreter', 'latex');
            ylabel('Braking Distance, $S$ (m)', 'Interpreter', 'latex');
            title_str = 'Braking Distance vs. Initial Speed (Fixed vs. Ideal Bias)';
            title(title_str);
            grid on;
            legend('Location', 'northwest');
            hold off;
            
            % Note: The difference between the fixed and ideal curves quantifies 
            % the extra stopping distance required due to the non-optimal brake bias.
        end

        function PlotAxleGripsAtPeakDecelVsSpeed(obj, v_max)
            % PlotAxleGripsAtPeakDecelVsSpeed Plots the utilized Front and Rear 
            % Axle Grip forces at the limit of adhesion (Peak Deceleration)
            % across a range of speeds.
            %
            % Inputs:
            %   v_max: The maximum velocity (m/s) to plot up to.
            
            % --- 1. Define the Range for Velocity (v) ---
            n_points = 50; 
            v_range = linspace(obj.ConstantVars.v_min, v_max, n_points); % Start at 5 m/s to ensure stability
            
            % --- 2. Initialize Output Arrays ---
            F_peak_grips = zeros(1, n_points); % Front Grip at limit
            R_peak_grips = zeros(1, n_points); % Rear Grip at limit
            
            % --- 3. Iterate to find Peak forces ---
            for i = 1:n_points
                v = v_range(i);
                
                % a) Find the max deceleration (d_peak) for this speed
                %    (This assumes optimal brake balance)
                [d_peak, ~] = obj.PeakDeceleration(v);
                
                % b) Calculate the Axle Grips specifically at that deceleration
                [F, R] = obj.AxleGrips(v, d_peak);
                
                F_peak_grips(i) = F;
                R_peak_grips(i) = R;
            end
            
            % --- 4. Plot the Results ---
            figure;
            hold on;
            
            % Plot Front Axle Grip
            plot(v_range, F_peak_grips, 'b-', 'LineWidth', 2, 'DisplayName', 'Front Axle Grip (Limit)');
            
            % Plot Rear Axle Grip
            plot(v_range, R_peak_grips, 'r-', 'LineWidth', 2, 'DisplayName', 'Rear Axle Grip (Limit)');
            
            % Plot Total Grip (Sum) for reference
            plot(v_range, F_peak_grips + R_peak_grips, 'k--', 'LineWidth', 1.5, 'DisplayName', 'Total Available Grip');
            
            % --- 5. Annotate the Plot ---
            xlabel('Forward Velocity, $v$ (m/s)', 'Interpreter', 'latex');
            ylabel('Longitudinal Grip Force (N)', 'Interpreter', 'latex');
            title_str = 'Axle Grip Forces at Peak Deceleration vs. Speed';
            title(title_str);
            grid on;
            legend('Location', 'northwest');
            
            % Add context to the plot limits
            xlim([0, v_max]);
            
            hold off;
        end
        
        function PlotAxleGripsAtNeutralVsSpeed(obj, v_max)
            % PlotAxleGripsAtPeakDecelVsSpeed Plots the utilized Front and Rear 
            % Axle Grip forces at the limit of adhesion (Peak Deceleration)
            % across a range of speeds.
            %
            % Inputs:
            %   v_max: The maximum velocity (m/s) to plot up to.
            
            % --- 1. Define the Range for Velocity (v) ---
            n_points = 50; 
            v_range = linspace(obj.ConstantVars.v_min, v_max, n_points);
            
            % --- 2. Initialize Output Arrays ---
            F_peak_grips = zeros(1, n_points); % Front Grip at limit
            R_peak_grips = zeros(1, n_points); % Rear Grip at limit
            
            % --- 3. Iterate to find Peak forces ---
            for i = 1:n_points
                v = v_range(i);
                [F, R] = obj.AxleGrips(v, 0);
                
                F_peak_grips(i) = F;
                R_peak_grips(i) = R;
            end
            
            % --- 4. Plot the Results ---
            figure;
            hold on;
            
            % Plot Front Axle Grip
            plot(v_range, F_peak_grips, 'b-', 'LineWidth', 2, 'DisplayName', 'Front Grip');
            
            % Plot Rear Axle Grip
            plot(v_range, R_peak_grips, 'r-', 'LineWidth', 2, 'DisplayName', 'Rear Grip');
            
            % Plot Total Grip (Sum) for reference
            plot(v_range, F_peak_grips + R_peak_grips, 'w--', 'LineWidth', 1.5, 'DisplayName', 'Total Available Grip');
            
            % --- 5. Annotate the Plot ---
            xlabel('Forward Velocity, $v$ (m/s)', 'Interpreter', 'latex');
            ylabel('Longitudinal Grip Force (N)', 'Interpreter', 'latex');
            title_str = 'Axle Grip Forces at 0 Acceleration vs. Speed';
            title(title_str);
            grid on;
            legend('Location', 'northwest');
            
            % Add context to the plot limits
            xlim([0, v_max]);
            
            hold off;
        end

        function PlotRearForceComparison(obj, v_max, BBal_fixed)
            % PlotRearForceComparison Plots the Rear Axle Braking Force for 
            % an Ideal (Variable) Bias vs. a Fixed Bias.
            %
            % Inputs:
            %   v_max: Maximum velocity (m/s)
            %   BBal_fixed: The fixed front bias (e.g. 0.60 for 60% Front)
    
            % 1. Setup
            n_points = 50;
            v_range = linspace(obj.ConstantVars.v_min, v_max, n_points); % Start > 0 to be safe
            
            R_optimal = zeros(1, n_points);
            R_fixed   = zeros(1, n_points);
            
            % 2. Iterate over velocity
            for i = 1:n_points
                v = v_range(i);
                
                % --- Scenario A: Optimal / Peak Braking ---
                % 1. Find the maximum possible deceleration (d_opt)
                [d_opt, ~] = obj.PeakDeceleration(v);
                
                % 2. Calculate the tire forces at this limit.
                % In the optimal scenario, the brake force distribution matches
                % the tire grip distribution exactly. Therefore, the Rear Force
                % used is exactly the Rear Grip available.
                [~, R_avail_opt] = obj.AxleGrips(v, d_opt);
                R_optimal(i) = R_avail_opt;
                
                % --- Scenario B: Fixed Brake Bias ---
                % 1. Find the max deceleration allowed by the fixed bias (d_fixed)
                d_fixed = obj.PeakDecelerationFixedBBal(v, BBal_fixed);
                
                % 2. Calculate the Total Friction Force required from tires
                % Equation: F_total = Mass * Decel - AerodynamicDrag
                F_friction_total = (obj.ConstantVars.M * d_fixed) - obj.DragForce(v);
                
                % 3. Calculate the Rear Force portion
                % With fixed bias, Rear Force is simply (1 - Bias) * Total Force
                R_fixed(i) = F_friction_total * (1 - BBal_fixed);
            end
            
            % 3. Plotting
            figure;
            
            % Create a shaded area to represent the "Lost Performance"
            % We use the 'fill' command to shade between the curves
            fill([v_range, fliplr(v_range)], [R_optimal, fliplr(R_fixed)], ...
                 [1, 0.8, 0.8], 'EdgeColor', 'none', 'FaceAlpha', 0.5, ...
                 'DisplayName', 'Unutilized Rear Grip');
            hold on;
            
            % Plot Optimal Rear Force
            plot(v_range, R_optimal, 'w-', 'LineWidth', 2, ...
                 'DisplayName', 'Peak Rear Potential (Ideal Bias)');
                 
            % Plot Fixed Rear Force
            plot(v_range, R_fixed, 'b--', 'LineWidth', 2, ...
                 'DisplayName', sprintf('Actual Rear Force (Fixed %.0f%% Bias)', BBal_fixed*100));
            
            % Formatting
            xlabel('Velocity (m/s)', 'Interpreter', 'latex');
            ylabel('Rear Longitudinal Force (N)', 'Interpreter', 'latex');
            title(['Rear Brake Force: Ideal vs. Fixed Bias (' num2str(BBal_fixed*100) '%)']);
            legend('Location', 'best');
            grid on;
            hold off;
        end
    
        function PlotAxleGripsAeroDifference(obj, v_max)
            % PlotAxleGripsAtPeakDecelVsSpeed Plots the utilized Front and Rear 
            % Axle Grip forces at the limit of adhesion (Peak Deceleration)
            % across a range of speeds.
            %
            % Inputs:
            %   v_max: The maximum velocity (m/s) to plot up to.
            
            % --- 1. Define the Range for Velocity (v) ---
            n_points = 50; 
            v_range = linspace(obj.ConstantVars.v_min, v_max, n_points);
            
            % --- 2. Initialize Output Arrays ---
            F_peak_grips = zeros(1, n_points); % Front Grip at limit
            R_peak_grips = zeros(1, n_points); % Rear Grip at limit
            F_no_aero_grips = zeros(1, n_points);
            R_no_aero_grips = zeros(1, n_points);
            
            % --- 3. Iterate to find Peak forces ---
            for i = 1:n_points
                v = v_range(i);
                [F, R] = obj.AxleGrips(v, 0);
                
                F_peak_grips(i) = F;
                R_peak_grips(i) = R;

                [F, R] = obj.AxleGripsWithoutAero(v, 0);
                F_no_aero_grips(i) = F;
                R_no_aero_grips(i) = R;
            end
            
            % --- 4. Plot the Results ---
            figure;
            hold on;
            
            % Plot Front Axle Grip
            plot(v_range, F_peak_grips, 'b-', 'LineWidth', 2, 'DisplayName', 'Front Grip w Aero');
            
            % Plot Rear Axle Grip
            plot(v_range, R_peak_grips, 'r-', 'LineWidth', 2, 'DisplayName', 'Rear Grip w Aero');

            % Plot Front Axle Grip
            % plot(v_range, F_no_aero_grips, 'b:', 'LineWidth', 2, 'DisplayName', 'Front Grip w/o Aero');
            
            % Plot Rear Axle Grip
            % plot(v_range, R_no_aero_grips, 'r:', 'LineWidth', 2, 'DisplayName', 'Rear Grip w/o Aero');
            
            % Plot Total Grip (Sum) for reference
            % plot(v_range, F_peak_grips + R_peak_grips, 'w-', 'LineWidth', 1.5, 'DisplayName', 'Total Available Grip w Aero');

            % Plot Total Grip (Sum) for reference
            % plot(v_range, F_no_aero_grips + R_no_aero_grips, 'w:', 'LineWidth', 1.5, 'DisplayName', 'Total Available Grip w/o Aero');
            
            % --- 5. Annotate the Plot ---
            xlabel('Forward Velocity, $v$ (m/s)', 'Interpreter', 'latex');
            ylabel('Longitudinal Grip Force (N)', 'Interpreter', 'latex');
            title_str = 'Longitudinal Grip at each Axle at 0 Acceleration vs. Speed';
            title(title_str);
            grid on;
            legend('Location', 'northwest');
            
            % Add context to the plot limits
            xlim([obj.ConstantVars.v_min, v_max]);
            
            hold off;
        end

        function PlotRequiredRegenToOptimize(obj, v_max, BBal_fixed)
            % PlotRequiredRegenToOptimize Calculates and plots the supplementary 
            % regenerative braking force required on the rear axle to achieve 
            % maximum theoretical deceleration, given a sub-optimal fixed brake bias.
            %
            % Inputs:
            %   v_max: Maximum velocity (m/s)
            %   BBal_fixed: The fixed hydraulic bias (e.g., 0.65 for 65% Front)

            % --- 1. Setup Ranges ---
            n_points = 50;
            v_range = linspace(obj.ConstantVars.v_min, v_max, n_points);
            
            % Initialize Arrays
            R_total_capacity = zeros(1, n_points); % The max grip available at rear
            R_hydraulic      = zeros(1, n_points); % What the mech brakes provide
            R_regen_needed   = zeros(1, n_points); % The gap to fill

            
            % --- 2. Iterate Logic ---
            for i = 1:n_points
                v = v_range(i);
                
                % A. Find the Optimal Scenario (Target)
                % We want the car to behave as if it had optimal bias.
                [d_opt, ~] = obj.PeakDeceleration(v);
                
                % B. Get the Tire Forces at this Optimal Deceleration
                [F_opt, R_opt] = obj.AxleGrips(v, d_opt);
                
                % C. Calculate Hydraulic Behavior
                % With a fixed bias (assuming safe understeer setup), the driver 
                % brakes until the FRONT locks/saturates (F_opt).
                % The rear hydraulic force is slave to the front via BBal.
                F_hyd_front = F_opt;
                F_hyd_rear  = F_hyd_front * (1 - BBal_fixed) / BBal_fixed;
                
                % D. Calculate Regen Requirement
                % We want Total Rear Force == R_opt
                Req_Force = R_opt - F_hyd_rear;
                
                % Handle cases where fixed bias might be too rear-heavy already
                if Req_Force < 0
                    Req_Force = 0; 
                    F_hyd_rear = R_opt; % Saturated by brakes alone
                end
                
                R_total_capacity(i) = R_opt;
                R_hydraulic(i)      = F_hyd_rear;
                R_regen_needed(i)   = Req_Force;
                
            end
            
            % --- 3. Visualization ---
            figure('Name', 'Regenerative Braking Requirements', 'NumberTitle', 'off');
            
            % Subplot 1: Forces (Stacked Area)
            subplot(2, 1, 1);
            hold on;
            
            % Use area plot to show how Regen stacks on top of Hydraulic
            area_data = [R_hydraulic', R_regen_needed'];
            h = area(v_range, area_data);
            
            % Styling the area
            h(1).FaceColor = [0.6 0.6 0.6]; % Grey for Mechanical/Hydraulic
            h(1).DisplayName = 'Mechanical Brake Force';
            h(1).EdgeColor = 'none';
            h(1).FaceAlpha = 0.5;
            
            h(2).FaceColor = [0.2 0.8 0.2]; % Green for Regen
            h(2).DisplayName = 'Required Regen Force';
            h(2).EdgeColor = 'none';
            h(2).FaceAlpha = 0.6;
            
            % Plot the Total Limit Line
            plot(v_range, R_total_capacity, 'k--', 'LineWidth', 1.5, 'DisplayName', 'Rear Grip Limit');
            
            ylabel('Rear Longitudinal Force (N)', 'Interpreter', 'latex');
            title(sprintf('Rear Axle Force Composition (Fixed Bias: %.0f%%)', BBal_fixed*100));
            grid on;
            legend('Location', 'northwest');
            
            % Subplot 2: Power Requirement
            subplot(2, 1, 2);
            plot(v_range, R_regen_needed, 'g-', 'LineWidth', 2);
            
            % Add a marker for max power
            [max_p, idx_p] = max(R_regen_needed);
            text(v_range(idx_p), max_p, sprintf(' Peak: %.1f N', max_p), ...
                'VerticalAlignment', 'bottom', 'FontWeight', 'bold');
            
            xlabel('Forward Velocity (m/s)', 'Interpreter', 'latex');
            ylabel('Regen Force Required (N)', 'Interpreter', 'latex');
            grid on;
            
            % Visual cleanup
            xlim([obj.ConstantVars.v_min, v_max]);
        end

        function PlotBrakingTimeVsSpeed(obj, v_max, BBal_fixed)
            % PlotBrakingTimeVsSpeed Calculates and plots the time to brake (T)
            % for both ideal and fixed brake bias as a function of initial speed.
            %
            % Inputs:
            %   v_max: The maximum initial velocity (m/s) to plot up to.
            %   BBal_fixed: The fixed front brake bias ratio (e.g., 0.65).
            
            % --- 1. Define the Sweep Range for Velocity (v) ---
            n_decel_points = 100; 
            v_decel_range = linspace(0.1, v_max, n_decel_points); % This brakes when set to 0 - so put something close, like 0.1. 
            
            % --- 2. Calculate Deceleration Curves d(v) ---
            d_ideal = zeros(1, n_decel_points);
            d_fixed = zeros(1, n_decel_points);
            
            for i = 1:n_decel_points
                v = v_decel_range(i);
                
                % Ideal Deceleration
                [d_ideal(i), ~] = obj.PeakDeceleration(v);
                
                % Fixed Deceleration
                d_fixed(i) = obj.PeakDecelerationFixedBBal(v, BBal_fixed);
            end
            
            % Handle NaNs to prevent integration errors
            d_ideal(isnan(d_ideal)) = 0.01; 
            d_fixed(isnan(d_fixed)) = 0.01; 

            % --- 3. Integrate to Find Braking Time T (v_initial) ---
            
            % The formula is T = integral(1 / d(v) dv) from 0 to v_initial.
            % Since a = dv/dt -> dt = dv/a
            
            integrand_ideal = 1 ./ d_ideal;
            integrand_fixed = 1 ./ d_fixed;
            
            % cumtrapz integrates the integrand array with respect to v_decel_range
            T_ideal = cumtrapz(v_decel_range, integrand_ideal);
            T_fixed = cumtrapz(v_decel_range, integrand_fixed);
            
            % --- 4. Plot the Braking Time Results ---
            figure; 
            hold on;
            
            % Plot Time with Ideal Bias
            plot(v_decel_range, T_ideal, 'r--', 'LineWidth', 1.5, 'DisplayName', 'Time to Stop (Ideal Bias)');
            
            % Plot Time with Fixed Bias
            BBal_percent = BBal_fixed * 100;
            display_name_fixed = sprintf('Time to Stop (Fixed %.1f%% Bias)', BBal_percent);
            plot(v_decel_range, T_fixed, 'b-', 'LineWidth', 2, 'DisplayName', display_name_fixed);
            
            % --- 5. Annotate the Plot ---
            xlabel('Initial Velocity, $v_{initial}$ (m/s)', 'Interpreter', 'latex');
            ylabel('Time to Stop, $T$ (s)', 'Interpreter', 'latex');
            title_str = 'Braking Time vs. Initial Speed';
            title(title_str);
            grid on;
            legend('Location', 'northwest');
            
            % Optional: Display the time difference at max speed
            time_diff = T_fixed(end) - T_ideal(end);
            text(v_max, T_fixed(end), sprintf('  +%.2fs lost', time_diff), ...
                'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'right');
            
            hold off;
        end

        function PlotRegenOptimizedBias(obj, v_max, max_regen_force)
            % PlotRegenOptimizedBias Calculates the optimal FIXED hydraulic bias 
            % such that at v_max, the regenerative braking hits exactly 'max_regen_force'.
            % It then plots how this fixed bias performs across the speed range.
            %
            % Inputs:
            %   v_max: The velocity at which to calibrate the system (m/s).
            %   max_regen_force: The peak force capability of the motor (N).

            % --- 1. Calculate the Design Point at v_max ---
            
            % Get ideal peak deceleration at max speed
            [d_top, ~] = obj.PeakDeceleration(v_max);
            
            % Get the ideal grip distribution at that deceleration
            [F_top_opt, R_top_opt] = obj.AxleGrips(v_max, d_top);
            
            % Determine the Rear Hydraulic contribution required at top speed
            % R_total = R_hyd + R_regen
            % R_hyd = R_total - R_regen
            R_hyd_top = R_top_opt - max_regen_force;
            
            % Safety clamp: If regen is stronger than the total tire grip, 
            % hydraulic rear is 0.
            if R_hyd_top < 0
                R_hyd_top = 0;
            end
            
            % Calculate the FIXED Hydraulic Bias required to achieve this split
            % Bias = Front_Hyd / (Front_Hyd + Rear_Hyd)
            % Note: Front_Hyd is simply F_top_opt (front does 100% of front work)
            Calibrated_Bias = F_top_opt / (F_top_opt + R_hyd_top);
            
            fprintf('<strong>System Calibration:</strong>\n');
            fprintf('To utilize %.0f N of Regen at %.0f m/s:\n', max_regen_force, v_max);
            fprintf('Set Fixed Hydraulic Bias to: <strong>%.2f%% Front</strong>\n', Calibrated_Bias * 100);

            % --- 2. Simulate across Speed Range ---
            n_points = 100;
            v_range = linspace(obj.ConstantVars.v_min, v_max, n_points);
            
            F_hyd_plot = zeros(1, n_points);
            R_hyd_plot = zeros(1, n_points);
            R_regen_plot = zeros(1, n_points);
            Regen_Saturation = zeros(1, n_points); % For checking limits
            
            for i = 1:n_points
                v = v_range(i);
                
                % A. Find the Limit of Adhesion (Target)
                [d_opt, ~] = obj.PeakDeceleration(v);
                [F_target, R_target] = obj.AxleGrips(v, d_opt);
                
                % B. Calculate Hydraulic Forces based on the FIXED Calibrated Bias
                % Front must take the full front load
                F_hyd = F_target;
                
                % Rear Hydraulic is mechanically tied to front via bias
                % F / R = Bias / (1-Bias)  =>  R = F * (1-Bias)/Bias
                R_hyd = F_hyd * (1 - Calibrated_Bias) / Calibrated_Bias;
                
                % C. Calculate Regen Required to fill the gap
                R_regen_needed = R_target - R_hyd;
                
                % Sanity check: Ensure non-negative regen 
                % (If bias is too rearward, R_hyd might exceed R_target at low speeds)
                if R_regen_needed < 0
                    R_regen_needed = 0; 
                    % In reality, this means the rear wheels would lock up.
                end
                
                F_hyd_plot(i) = F_hyd;
                R_hyd_plot(i) = R_hyd;
                R_regen_plot(i) = R_regen_needed;
                
                % Check if we exceed the motor limit at lower speeds
                Regen_Saturation(i) = R_regen_needed / max_regen_force * 100;
            end
            
            % --- 3. Visualization ---
            figure('Name', 'Regen Optimization Strategy', 'NumberTitle', 'off');
            
            subplot(2, 1, 1);
            hold on;
            
            % Stacked Area Plot: Shows where the braking force comes from
            % We stack [Rear_Hyd, Rear_Regen, Front_Hyd]
            area_data = [R_hyd_plot', R_regen_plot', F_hyd_plot'];
            h = area(v_range, area_data);
            
            % Style: Rear Hydraulic (Red)
            h(1).FaceColor = [0.8 0.3 0.3]; 
            h(1).DisplayName = 'Rear Hydraulic';
            h(1).EdgeColor = 'none';
            
            % Style: Rear Regen (Green)
            h(2).FaceColor = [0.3 0.8 0.3];
            h(2).DisplayName = 'Rear Regen';
            h(2).EdgeColor = 'none';
            
            % Style: Front Hydraulic (Blue)
            h(3).FaceColor = [0.3 0.3 0.8];
            h(3).DisplayName = 'Front Hydraulic';
            h(3).EdgeColor = 'none';
            h(3).FaceAlpha = 0.5;
            
            ylabel('Braking Force (N)', 'Interpreter', 'latex');
            title(sprintf('Brake Force Composition (Bias Fixed at %.1f%%)', Calibrated_Bias*100));
            legend('Location', 'northwest');
            grid on;
            
            subplot(2, 1, 2);
            hold on;
            plot(v_range, Regen_Saturation, 'k-', 'LineWidth', 2);
            yline(100, 'r--', 'Motor Limit');
            
            ylabel('Regen Motor Usage (%)', 'Interpreter', 'latex');
            xlabel('Velocity (m/s)', 'Interpreter', 'latex');
            title('Regen Saturation vs Speed');
            grid on;
            ylim([0 120]);
            
            % Add text annotation
            text(v_range(end), Regen_Saturation(end), ' \leftarrow Calibration Point (100%)', ...
                'HorizontalAlignment', 'left', 'VerticalAlignment', 'middle');
            
            hold off;
        end
    end
end