filename = "FLIGHT.csv";
%filename = "IMUdata.csv";
%filename = "more_data.csv";
data = readtable(filename);

%acc = data(:, 1:3); % Accelerometer readings
%gyro = data(:, 4:6);% Gyroscope readings
%mag = data(:, 7:end); %Magnetometer readings
acc = table2array(data(3:end, ["Var8" "Var9" "Var10"]));
gyro = table2array(data(3:end, ["Var11" "Var12" "Var13"]));
acc_mean = mean(acc).';
gyro_mean = mean(gyro).';
%mag_mean = mean(mag).';
expected_acc = [0; 0; 9.82];
bias_acc = (acc_mean - expected_acc).';
bias_gyro = gyro_mean;
N = size(acc, 1);
% Compute bias
mean(acc)
expected_acc = [0; 0; 9.82];
%bias_acc = [0.0277 0.7364 0.1344].';
%bias_gyro = [-0.5607 -0.0342 -0.3706].';   
bias_mag = [83.3796  220.8556  217.8227].'; 
% Correct sensor readings
acc_corr = acc - bias_acc ;
gyro_corr = (gyro.' - bias_gyro).' ;
%mag_corr = mag.' - bias_mag  ;
%mag_corr = mag_corr.';

gyro_error = bias_gyro ;


q0 = [1 0 0 0].';
dt = 0.01; %100 hz filter
sim_time = 0:dt:dt*N-dt; %Simulation time

q_mat = zeros(4, N);
q_mat(:, 1) = q0;
acc_corr = acc_corr.';
gyro_corr = gyro_corr.';
euler_angles = zeros(3, N);
euler_angles(:, 1) = Q_to_Euler(q0);
for k = 2:N
    q_next = madgewick_filter( acc_corr(:,k), gyro_corr(:,k), gyro_error,  q0, dt);
    q_mat(:, k) = q_next;
    q0 = q_next;
    [roll pitch yaw] = Q_to_Euler(q0);
    euler_angles(:, k) = [roll pitch yaw].';

end
euler_angles = rad2deg(euler_angles);


ax = acc_corr(1,:);
ay = acc_corr(2,:);
az = acc_corr(3,:);

%mx = mag(:, 1).';
%my = mag(:, 2).';
%mz = mag(:, 3).';


pitch_acc = atan2(-ax, sqrt(ay.^2 + az.^2 ));
roll_acc = atan2(ay, az);
%%
yaw_acc = -atan2( my .* cos(roll_acc) - mz .* sin(roll_acc), ...
                  mx .* cos(pitch_acc) + ...
                  my .* sin(pitch_acc) .* sin(roll_acc) + ...
                  mz .* sin(pitch_acc) .* cos(roll_acc) );

euler_angles_acc = [pitch_acc roll_acc yaw_acc.'].';
euler_angles_acc = rad2deg(euler_angles_acc);


%% Madgewick vs Accelerometer estimation of roll, pitch 
figure
subplot(2,1,1);
plot(sim_time, euler_angles(1,:))
hold on
plot(sim_time, pitch_acc)
legend("Madgewick", "Accelerometer")
xlabel("Time [s]")
ylabel("Roll [d]")
title('Madgewick vs accelerometer estimation of \phi');

subplot(2,1,2);
plot(sim_time,euler_angles(2,:))
hold on
plot(sim_time,roll_acc)
xlabel("Time [s]")
ylabel("Pitch [d]")
title('Madgewick vs accelerometer estimation of \theta');
legend("Madgewick", "Accelerometer")

%subplot(3,1,3);
%plot(sim_time,euler_angles(3,:))
%hold on
%plot(sim_time,euler_angles_acc(3,:))
%xlabel("Time [s]")
%ylabel("Yaw [d]")
%title('Madgewick vs accelerometer estimation of \psi');
%legend("Madgewick", "Accelerometer")


%% Estimate roll, pitch and yaw using gyroscope data
gx = gyro_corr(:, 1);
gy = gyro_corr(:, 2);
gz = gyro_corr(:, 3);

integrated_gyro = zeros(N, 3);
% Integrate angular velocity to approximate angles
for t = 1:N-1
    integrated_gyro(t+1, :) = integrated_gyro(t, :) + gyro_corr(t, :) * dt;
end
integrated_gyro = integrated_gyro.';
euler_angles_unwrapped = unwrap(euler_angles);

%% Madgewick vs Gyro numerical estimation of roll, pitch and yaw
figure
subplot(3,1,1);
plot(sim_time, unwrap(euler_angles(1,:)))
hold on
plot(sim_time, integrated_gyro(1,:))
xlabel("Time [s]")
ylabel("Roll [d]")
legend("Madgewick", "Gyroscope")
title('Madgewick vs numerical Gyroscope estimation of \phi');

subplot(3,1,2);
plot(sim_time, unwrap(euler_angles(2,:)))
hold on
plot(sim_time, integrated_gyro(2,:))
xlabel("Time [s]")
ylabel("Pitch [d]")
title('Madgewick vs numerical Gyroscope estimation of \theta');
legend("Madgewick", "Gyroscope")

subplot(3,1,3);
plot(sim_time, unwrap(euler_angles(3,:)))
hold on
plot(sim_time,integrated_gyro(3,:))
xlabel("Time [s]")
ylabel("Yaw [d]")
title('Madgewick vs numerical Gyroscope estimation of \psi');
legend("Madgewick", "Gyroscope")
