% Filter Comparison Script
% Reads CSV data with format: Madgwick1,Madgwick2,EKF for roll,pitch,yaw
% Plots the data in 3 subplots with 10ms sampling interval

clear; clc; close all;

% Read the CSV file
filename = 'Flight_attitude_test.csv';
data = readmatrix(filename);

% Extract data columns
roll = data(:, 1);
roll1 = data(:, 2);
roll2 = data(:, 3);
pitch = data(:, 4);
pitch1 = data(:, 5);
pitch2 = data(:, 6);
yaw = data(:, 7);
yaw1 = data(:, 8);
yaw2 = data(:, 9);

% Create time vector (10ms between samples)
dt = 0.01; % 10ms in seconds
n_samples = length(roll);
time = (0:n_samples-1) * dt;

% Create figure with 3 subplots
figure('Name', 'Filter Comparison', 'Position', [100, 100, 1000, 800]);

% Subplot 1: Roll comparison
subplot(3, 1, 1);
plot(time, roll2, 'g-', 'LineWidth', 1.5, 'DisplayName', 'EKF');
hold on;
plot(time, roll, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Madgwick1');
plot(time, roll1, 'r--', 'LineWidth', 1.5, 'DisplayName', 'Madgwick2');
grid on;
xlabel('Time (s)');
ylabel('Roll (degrees)');
title('Roll Comparison');
legend('show', 'Location', 'best');

% Subplot 2: Pitch comparison
subplot(3, 1, 2);
plot(time, pitch2, 'g-', 'LineWidth', 1.5, 'DisplayName', 'EKF');
hold on;
plot(time, pitch, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Madgwick1');
plot(time, pitch1, 'r--', 'LineWidth', 1.5, 'DisplayName', 'Madgwick2');
grid on;
xlabel('Time (s)');
ylabel('Pitch (degrees)');
title('Pitch Comparison');
legend('show', 'Location', 'best');

% Subplot 3: Yaw comparison
subplot(3, 1, 3);
plot(time, yaw2, 'g-', 'LineWidth', 1.5, 'DisplayName', 'EKF');
hold on;
plot(time, yaw, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Madgwick1');
plot(time, yaw1, 'r--', 'LineWidth', 1.5, 'DisplayName', 'Madgwick2');
grid on;
xlabel('Time (s)');
ylabel('Yaw (degrees)');
title('Yaw Comparison');
legend('show', 'Location', 'best');

% Add overall title
sgtitle('Attitude Filter Comparison (10ms Sampling Rate)');

% Display some basic statistics
fprintf('Data loaded successfully!\n');
fprintf('Number of samples: %d\n', n_samples);
fprintf('Total time: %.2f seconds\n', time(end));
fprintf('Sampling rate: %.1f Hz\n', 1/dt);
fprintf('\nData ranges:\n');
fprintf('Roll Madgwick1: [%.2f, %.2f] degrees\n', min(roll), max(roll));
fprintf('Roll Madgwick2: [%.2f, %.2f] degrees\n', min(roll1), max(roll1));
fprintf('Roll EKF: [%.2f, %.2f] degrees\n', min(roll2), max(roll2));
fprintf('Pitch Madgwick1: [%.2f, %.2f] degrees\n', min(pitch), max(pitch));
fprintf('Pitch Madgwick2: [%.2f, %.2f] degrees\n', min(pitch1), max(pitch1));
fprintf('Pitch EKF: [%.2f, %.2f] degrees\n', min(pitch2), max(pitch2));
fprintf('Yaw Madgwick1: [%.2f, %.2f] degrees\n', min(yaw), max(yaw));
fprintf('Yaw Madgwick2: [%.2f, %.2f] degrees\n', min(yaw1), max(yaw1));
fprintf('Yaw EKF: [%.2f, %.2f] degrees\n', min(yaw2), max(yaw2));
