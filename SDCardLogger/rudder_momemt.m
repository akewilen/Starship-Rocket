% Rudder Moment Analysis
% Plots rudder angle vs force and performs linear curve fitting
% Data: rudder angle (degrees) vs force measurements


clear; clc; close all;
h = 0.4345;
% Data: [rudder_angle_deg, force_kg]
% Negative angles produce positive forces, positive angles produce negative forces
data_negative = [
    -5,  -0.06;
    -10, -0.11;
    -20, -0.21;
    -30, -0.30;
    -40, -0.39;
    -45, -0.41;
    -50, -0.39;
    -60, -0.33
];

% Positive angles (mirror the negative data with opposite force direction)
data_positive = [
    5,  0.06;
    10, 0.11;
    20, 0.21;
    30, 0.30;
    40, 0.39;
    45, 0.41;
    50, 0.39;
    60, 0.33
];

% Combine all data
data = [data_negative; data_positive];

% Extract rudder angles and forces
rudder_angle = data(:, 1);  % degrees
force_kg = data(:, 2);      % kg

% Convert kg to Newtons (1 kg = 9.81 N)
g = 9.81; % acceleration due to gravity (m/s²)
force_N = force_kg * g* h;

% Display the converted data
fprintf('Rudder Angle vs Force Data:\n');
fprintf('Angle (deg)\tForce (kg)\tForce (N)\n');
fprintf('----------------------------------------\n');
for i = 1:length(rudder_angle)
    fprintf('%8.1f\t%8.1f\t%8.2f\n', rudder_angle(i), force_kg(i), force_N(i));
end

% Filter data for linear fit between -45° to 45°
linear_range_mask = (rudder_angle >= -45) & (rudder_angle <= 45);
rudder_angle_linear = rudder_angle(linear_range_mask);
force_N_linear = force_N(linear_range_mask);

% Perform curve fitting on full data (quadratic and cubic only)
% Quadratic fit on all data
p2 = polyfit(rudder_angle, force_N, 2);
% Cubic fit on all data
p3 = polyfit(rudder_angle, force_N, 3);

% Linear fit only on -45° to 45° range
p1_limited = polyfit(rudder_angle_linear, force_N_linear, 1);

% Generate fitted curve data
angle_fit = linspace(min(rudder_angle), max(rudder_angle), 200);
angle_fit_linear = linspace(-45, 45, 100);
force_fit_linear_limited = polyval(p1_limited, angle_fit_linear);
force_fit_quad = polyval(p2, angle_fit);
force_fit_cubic = polyval(p3, angle_fit);

% Calculate R-squared for each fit
calculateRSquaredLimited = @(p, angles, forces) 1 - sum((forces - polyval(p, angles)).^2) / sum((forces - mean(forces)).^2);
calculateRSquared = @(p) 1 - sum((force_N - polyval(p, rudder_angle)).^2) / sum((force_N - mean(force_N)).^2);

R_squared_linear_limited = calculateRSquaredLimited(p1_limited, rudder_angle_linear, force_N_linear);
R_squared_quad = calculateRSquared(p2);
R_squared_cubic = calculateRSquared(p3);

% Display curve fitting results
fprintf('\nCurve Fitting Results:\n');
fprintf('================================\n');
fprintf('Linear Fit (-45° to 45° range only):\n');
fprintf('  F = %.4f * θ + %.4f\n', p1_limited(1), p1_limited(2));
fprintf('  R-squared: %.4f (for data in -45° to 45° range)\n\n', R_squared_linear_limited);

fprintf('Quadratic Fit (full data):\n');
fprintf('  F = %.4f * θ² + %.4f * θ + %.4f\n', p2(1), p2(2), p2(3));
fprintf('  R-squared: %.4f\n\n', R_squared_quad);

fprintf('Cubic Fit (full data):\n');
fprintf('  F = %.4f * θ³ + %.4f * θ² + %.4f * θ + %.4f\n', p3(1), p3(2), p3(3), p3(4));
fprintf('  R-squared: %.4f\n\n', R_squared_cubic);

% Determine best fit among polynomial fits
[~, best_fit_idx] = max([R_squared_quad, R_squared_cubic]);
fit_names = {'Quadratic', 'Cubic'};
fprintf('Best polynomial fit: %s (R² = %.4f)\n', fit_names{best_fit_idx}, max([R_squared_quad, R_squared_cubic]));

% Create the plot
figure('Name', 'Rudder Moment Analysis - Limited Linear + Polynomial Fits', 'Position', [100, 100, 1000, 700]);

% Plot original data points
plot(rudder_angle, force_N, 'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'red', 'LineWidth', 2);
hold on;

% Plot fitted curves
plot(angle_fit_linear, force_fit_linear_limited, 'r', 'LineWidth', 3);
plot(angle_fit, force_fit_cubic, 'g', 'LineWidth', 2);

% Add zero line for reference
plot([min(rudder_angle)-5, max(rudder_angle)+5], [0, 0], 'k-', 'LineWidth', 0.5, 'Color', [0.7, 0.7, 0.7]);

% Add vertical lines at -45° and +45° to show linear fit range
plot([-45, -45], [min(force_N)-1, max(force_N)+1], 'b:', 'LineWidth', 1.5, 'Color', [0.5, 0.5, 1]);
plot([45, 45], [min(force_N)-1, max(force_N)+1], 'b:', 'LineWidth', 1.5, 'Color', [0.5, 0.5, 1]);

% Formatting
xlabel('Rudder Angle (degrees)', 'FontSize', 12, 'FontWeight', 'bold');
ylabel('Moment (N/m)', 'FontSize', 12, 'FontWeight', 'bold');
title('Rudder Angle vs Mx & My - Limited Linear (-45° to 45°) + Polynomial Fits', 'FontSize', 14, 'FontWeight', 'bold');
grid on;

% Create legend
legend_str = {
    'All Data', 
    'Linear Fit Data (-45° to 45°)',
    sprintf('Cubic'),
};
legend(legend_str, 'Location', 'best', 'FontSize', 9);

% Add text box with linear fit equation
eq_str = sprintf('Linear Fit (-45° to 45°):\nF = %.4f θ + %.4f\nR² = %.4f', p1_limited(1), p1_limited(2), R_squared_linear_limited);

text(0.02, 0.98, eq_str, 'Units', 'normalized', 'FontSize', 10, ...
     'BackgroundColor', 'white', 'EdgeColor', 'blue', 'VerticalAlignment', 'top');

% Set axis limits with some padding
xlim([min(rudder_angle) - 5, max(rudder_angle) + 5]);
ylim([min(force_N) - 1, max(force_N) + 1]);

% Add grid and make plot look professional
set(gca, 'FontSize', 11);
box on;

fprintf('\nRudder plot created successfully!\n');

%% PROPELLER THRUST ANALYSIS
fprintf('\n========================================\n');
fprintf('PROPELLER THRUST ANALYSIS\n');
fprintf('========================================\n');

% Propeller thrust data: [throttle_%, thrust_kg]
propeller_data = [
    10, 0.11;
    20, 0.43;
    30, 0.95;
    40, 1.50;
    50, 1.91;
    60, 2.45;
    70, 3.10;
    80, 3.50;
    90, 3.90;
    100, 4.23
];

% Extract throttle and thrust data
throttle_percent = propeller_data(:, 1);  % %
thrust_kg = propeller_data(:, 2);         % kg

% Convert kg to Newtons (1 kg = 9.81 N)
thrust_N = thrust_kg * g;

% Display the converted propeller data
fprintf('Propeller Throttle vs Thrust Data:\n');
fprintf('Throttle (%%)\tThrust (kg)\tThrust (N)\n');
fprintf('----------------------------------------\n');
for i = 1:length(throttle_percent)
    fprintf('%8.0f\t%8.2f\t%8.2f\n', throttle_percent(i), thrust_kg(i), thrust_N(i));
end

% Filter data for linear fit from 0% to 100% throttle (full range)
linear_throttle_mask = (throttle_percent >= 0) & (throttle_percent <= 100);
throttle_linear = throttle_percent(linear_throttle_mask);
thrust_N_linear = thrust_N(linear_throttle_mask);

% Perform curve fitting on propeller data
% Quadratic fit on all propeller data
p2_prop = polyfit(throttle_percent, thrust_N, 2);
% Cubic fit on all propeller data
p3_prop = polyfit(throttle_percent, thrust_N, 3);

% Linear fit on full 0% to 100% throttle range
p1_prop_limited = polyfit(throttle_linear, thrust_N_linear, 1);

% Generate fitted curve data for propeller
throttle_fit = linspace(min(throttle_percent), max(throttle_percent), 200);
throttle_fit_linear = linspace(0, 100, 100);
thrust_fit_linear_limited = polyval(p1_prop_limited, throttle_fit_linear);
thrust_fit_quad = polyval(p2_prop, throttle_fit);
thrust_fit_cubic = polyval(p3_prop, throttle_fit);

% Calculate R-squared for propeller fits
R_squared_prop_linear_limited = calculateRSquaredLimited(p1_prop_limited, throttle_linear, thrust_N_linear);
R_squared_prop_quad = 1 - sum((thrust_N - polyval(p2_prop, throttle_percent)).^2) / sum((thrust_N - mean(thrust_N)).^2);
R_squared_prop_cubic = 1 - sum((thrust_N - polyval(p3_prop, throttle_percent)).^2) / sum((thrust_N - mean(thrust_N)).^2);

% Display propeller curve fitting results
fprintf('\nPropeller Curve Fitting Results:\n');
fprintf('================================\n');
fprintf('Linear Fit (0%% to 100%% throttle range):\n');
fprintf('  T = %.4f * θ + %.4f\n', p1_prop_limited(1), p1_prop_limited(2));
fprintf('  R-squared: %.4f (for full throttle range)\n\n', R_squared_prop_linear_limited);

fprintf('Quadratic Fit (full data):\n');
fprintf('  T = %.4f * θ² + %.4f * θ + %.4f\n', p2_prop(1), p2_prop(2), p2_prop(3));
fprintf('  R-squared: %.4f\n\n', R_squared_prop_quad);

fprintf('Cubic Fit (full data):\n');
fprintf('  T = %.4f * θ³ + %.4f * θ² + %.4f * θ + %.4f\n', p3_prop(1), p3_prop(2), p3_prop(3), p3_prop(4));
fprintf('  R-squared: %.4f\n\n', R_squared_prop_cubic);

% Determine best fit among propeller polynomial fits
[~, best_prop_fit_idx] = max([R_squared_prop_quad, R_squared_prop_cubic]);
prop_fit_names = {'Quadratic', 'Cubic'};
fprintf('Best propeller polynomial fit: %s (R² = %.4f)\n', prop_fit_names{best_prop_fit_idx}, max([R_squared_prop_quad, R_squared_prop_cubic]));

% Create the propeller thrust plot
figure('Name', 'Propeller Thrust Analysis - Limited Linear + Polynomial Fits', 'Position', [150, 150, 1000, 700]);

% Plot original propeller data points
plot(throttle_percent, thrust_N, 'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'red', 'LineWidth', 2);
hold on;

% Plot fitted curves for propeller
plot(throttle_fit_linear, thrust_fit_linear_limited, 'r', 'LineWidth', 3);
plot(throttle_fit, thrust_fit_cubic, 'g', 'LineWidth', 2);

% Add zero line for reference
plot([0, 105], [0, 0], 'k-', 'LineWidth', 0.5, 'Color', [0.7, 0.7, 0.7]);

% Add vertical lines at 0% and 100% to show linear fit range
plot([0, 0], [0, max(thrust_N)+2], 'b:', 'LineWidth', 1.5, 'Color', [0.5, 0.5, 1]);
plot([100, 100], [0, max(thrust_N)+2], 'b:', 'LineWidth', 1.5, 'Color', [0.5, 0.5, 1]);

% Formatting
xlabel('Throttle (%)', 'FontSize', 12, 'FontWeight', 'bold');
ylabel('Thrust (Newtons)', 'FontSize', 12, 'FontWeight', 'bold');
title('Propeller Throttle vs Thrust - Linear (0% to 100%) + Polynomial Fits', 'FontSize', 14, 'FontWeight', 'bold');
grid on;

% Create legend
legend_str_prop = {
    'All Data', 
    'Linear Fit Data (0% to 100%)',
    sprintf('Cubic Fit')
};
legend(legend_str_prop, 'Location', 'best', 'FontSize', 9);

% Add text box with linear fit equation
eq_str_prop = sprintf('Linear Fit (0%% to 100%%):\nT = %.4f θ + %.4f\nR² = %.4f', p1_prop_limited(1), p1_prop_limited(2), R_squared_prop_linear_limited);

text(0.02, 0.98, eq_str_prop, 'Units', 'normalized', 'FontSize', 10, ...
     'BackgroundColor', 'white', 'EdgeColor', 'blue', 'VerticalAlignment', 'top');

% Set axis limits with some padding
xlim([0, 105]);
ylim([0, max(thrust_N) + 2]);

% Add grid and make plot look professional
set(gca, 'FontSize', 11);
box on;

fprintf('\nPropeller plot created successfully!\n');
