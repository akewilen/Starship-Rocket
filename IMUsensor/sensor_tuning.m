filename = "FLIGHT.csv";
%filename = "IMUdata.csv";
%filename = "more_data.csv";
data = readtable(filename);

%acc = data(:, 1:3); % Accelerometer readings
%gyro = data(:, 4:6);% Gyroscope readings
%mag = data(:, 7:end); %Magnetometer readings
acc = table2array(data(3:end, ["Var8" "Var9" "Var10"])).';
gyro = table2array(data(3:end, ["Var11" "Var12" "Var13"])).';
gyro = deg2rad(gyro);
N = size(gyro, 2);
acc_mean = mean(acc, 2);
gyro_mean = mean(gyro, 2);

bias_acc = acc_mean - [0;0;9.82]
bias_gyro = gyro_mean

cov_acc = ((acc - acc_mean) * (acc - acc_mean).')/(N-1);
cov_gyro = ((gyro - gyro_mean) * (gyro - gyro_mean).')/(N-1);

mean(gyro - gyro_mean, 2);