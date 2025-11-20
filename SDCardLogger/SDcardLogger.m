function SDcardLogger()
% SDCARDLOGGER Reads and plots flight data from FLIGHT.CSV
% This function reads the flight data CSV file and creates subplots
% showing timestamp (in ms) vs various flight parameters

    % Read the CSV file
    filename = 'FLIGHT.CSV';
    
    % Check if file exists
    if ~exist(filename, 'file')
        error('FLIGHT.CSV file not found in current directory');
    end
    
    % Read the data, skipping comment lines
    opts = detectImportOptions(filename, 'CommentStyle', '#');
    opts.VariableNamingRule = 'preserve';
    
    % Set proper variable names since the header gets skipped due to comments
    opts.VariableNames = {'Timestamp', 'Throttle', 'Pitch', 'Yaw', 'Roll', 'KillSwitch', 'Emergency'};
    
    % Read the data
    data = readtable(filename, opts);
    
    % Remove rows with missing timestamp data (empty rows)
    validRows = ~ismissing(data.Timestamp) & data.Timestamp > 0;
    data = data(validRows, :);
    
    % Extract data columns
    timestamp = data.Timestamp; % Already in milliseconds
    throttle = data.Throttle;
    pitch = data.Pitch;
    yaw = data.Yaw;
    roll = data.Roll;
    killSwitch = data.KillSwitch;
    emergency = data.Emergency;
    
    % Note: Battery_V and Temp_C columns exist but appear to be empty in the current dataset
    
    % Create figure with subplots
    figure('Name', 'Flight Data Analysis', 'Position', [100, 100, 1200, 800]);
    
    % Subplot 1: Throttle
    subplot(3, 2, 1);
    plot(timestamp, throttle, 'b-', 'LineWidth', 1.5);
    xlabel('Time (ms)');
    ylabel('Throttle (0-255)');
    title('Throttle');
    grid on;
    
    % Subplot 2: Pitch
    subplot(3, 2, 2);
    plot(timestamp, pitch, 'r-', 'LineWidth', 1.5);
    xlabel('Time (ms)');
    ylabel('Pitch (0-255, 127=center)');
    title('Pitch');
    grid on;
    
    % Subplot 3: Yaw
    subplot(3, 2, 3);
    plot(timestamp, yaw, 'g-', 'LineWidth', 1.5);
    xlabel('Time (ms)');
    ylabel('Yaw (0-255, 127=center)');
    title('Yaw');
    grid on;
    
    % Subplot 4: Roll
    subplot(3, 2, 4);
    plot(timestamp, roll, 'm-', 'LineWidth', 1.5);
    xlabel('Time (ms)');
    ylabel('Roll (0-255, 127=center)');
    title('Roll');
    grid on;
    
    % Subplot 5: Emergency and Kill Switch status
    subplot(3, 2, 5);
    hold on;
    plot(timestamp, emergency, 'r-', 'LineWidth', 2, 'DisplayName', 'Emergency');
    plot(timestamp, killSwitch, 'k--', 'LineWidth', 1, 'DisplayName', 'Kill Switch');
    xlabel('Time (ms)');
    ylabel('Status (0=normal, 1=active)');
    title('Emergency Status');
    legend('show');
    grid on;
    hold off;
    
    % Subplot 6: Combined control surfaces (normalized to show deviation from center)
    subplot(3, 2, 6);
    hold on;
    plot(timestamp, pitch - 127, 'r-', 'LineWidth', 1, 'DisplayName', 'Pitch');
    plot(timestamp, yaw - 127, 'g-', 'LineWidth', 1, 'DisplayName', 'Yaw');
    plot(timestamp, roll - 127, 'b-', 'LineWidth', 1, 'DisplayName', 'Roll');
    xlabel('Time (ms)');
    ylabel('Control Deviation from Center');
    title('Control Surfaces Deviation (Centered at 0)');
    legend('show');
    grid on;
    hold off;
    
    % Add overall title
    sgtitle('Starship Flight Data Analysis');
    
    % Display flight statistics
    fprintf('\n=== Flight Data Summary ===\n');
    fprintf('Flight Duration: %.2f seconds\n', (max(timestamp) - min(timestamp)) / 1000);
    fprintf('Data Points: %d\n', length(timestamp));
    fprintf('Throttle Range: %d - %d\n', min(throttle), max(throttle));
    max_throttle_idx = find(throttle == max(throttle), 1, 'first');
    fprintf('Max Throttle Time: %.2f ms\n', timestamp(max_throttle_idx));
    fprintf('Emergency Events: %d\n', sum(emergency == 1));
    fprintf('Kill Switch Activations: %d\n', sum(killSwitch == 1));
    
    % Calculate and display control surface statistics
    pitch_deviation = abs(pitch - 127);
    yaw_deviation = abs(yaw - 127);
    roll_deviation = abs(roll - 127);
    
    fprintf('\nControl Surface Analysis:\n');
    fprintf('Max Pitch Deviation: %d (%.1f%% from center)\n', max(pitch_deviation), max(pitch_deviation)/127*100);
    fprintf('Max Yaw Deviation: %d (%.1f%% from center)\n', max(yaw_deviation), max(yaw_deviation)/127*100);
    fprintf('Max Roll Deviation: %d (%.1f%% from center)\n', max(roll_deviation), max(roll_deviation)/127*100);
    
end

% Additional helper function to plot specific parameters
function plotCustomParameters(varargin)
% PLOTCUSTOMPARAMETERS Allows plotting specific parameters from flight data
% Usage: plotCustomParameters('throttle', 'pitch') - plots only specified parameters
% Usage: plotCustomParameters() - plots all parameters (same as plotFlightData)

    if nargin == 0
        SDcardLogger();
        return;
    end
    
    % Read the data
    filename = 'FLIGHT.CSV';
    if ~exist(filename, 'file')
        error('FLIGHT.CSV file not found in current directory');
    end
    
    opts = detectImportOptions(filename, 'CommentStyle', '#');
    opts.VariableNamingRule = 'preserve';
    opts.VariableNames = {'Timestamp', 'Throttle', 'Pitch', 'Yaw', 'Roll', 'KillSwitch', 'Emergency', 'Battery_V', 'Temp_C'};
    data = readtable(filename, opts);
    validRows = ~ismissing(data.Timestamp) & data.Timestamp > 0;
    data = data(validRows, :);
    
    timestamp = data.Timestamp;
    
    % Create figure
    figure('Name', 'Custom Flight Data Plot', 'Position', [100, 100, 800, 600]);
    
    colors = {'b', 'r', 'g', 'm', 'c', 'k'};
    hold on;
    
    for i = 1:length(varargin)
        param = lower(varargin{i});
        
        switch param
            case 'throttle'
                plot(timestamp, data.Throttle, colors{mod(i-1,6)+1}, 'LineWidth', 1.5, 'DisplayName', 'Throttle');
            case 'pitch'
                plot(timestamp, data.Pitch, colors{mod(i-1,6)+1}, 'LineWidth', 1.5, 'DisplayName', 'Pitch');
            case 'yaw'
                plot(timestamp, data.Yaw, colors{mod(i-1,6)+1}, 'LineWidth', 1.5, 'DisplayName', 'Yaw');
            case 'roll'
                plot(timestamp, data.Roll, colors{mod(i-1,6)+1}, 'LineWidth', 1.5, 'DisplayName', 'Roll');
            case 'emergency'
                plot(timestamp, data.Emergency, colors{mod(i-1,6)+1}, 'LineWidth', 1.5, 'DisplayName', 'Emergency');
            case 'killswitch'
                plot(timestamp, data.KillSwitch, colors{mod(i-1,6)+1}, 'LineWidth', 1.5, 'DisplayName', 'Kill Switch');
            otherwise
                warning('Unknown parameter: %s', param);
        end
    end
    
    xlabel('Time (ms)');
    ylabel('Parameter Values');
    title('Flight Data - Selected Parameters');
    legend('show');
    grid on;
    hold off;
end
