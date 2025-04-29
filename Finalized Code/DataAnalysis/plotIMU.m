% Select the CSV file
[file, path] = uigetfile('4_26_duration_test.csv', 'Select the CSV file');
if isequal(file, 0)
    disp('User selected Cancel');
    return;
else
    filepath = fullfile(path, file);
end

% Read the table
data = readtable(filepath);

% Extract time
time = data{:, 1};

% Extract IMU orientation and acceleration data
imu_orientation = data{:, 10:12}; % Orientation X, Y, Z
acceleration = data{:, 13:15};    % Accel X, Y, Z

% Extract variable names for legends
headers = data.Properties.VariableNames;

%% --------- First Figure: IMU Orientation ---------
figure;

subplot(3,1,1);
plot(time, imu_orientation(:,1));
xlabel('Time (ms)');
ylabel('Orientation X (deg)');
title('IMU Orientation - Roll (X Axis)');
grid on;

subplot(3,1,2);
plot(time, imu_orientation(:,2));
xlabel('Time (ms)');
ylabel('Orientation Y (deg)');
title('IMU Orientation - Pitch (Y Axis)');
grid on;

subplot(3,1,3);
plot(time, imu_orientation(:,3));
xlabel('Time (ms)');
ylabel('Orientation Z (deg)');
title('IMU Orientation - Yaw (Z Axis)');
grid on;

sgtitle('IMU Orientation Angles Over Time');

%% --------- Second Figure: Acceleration ---------
figure;

subplot(3,1,1);
plot(time, acceleration(:,1));
xlabel('Time (ms)');
ylabel('Accel X (m/s^2)');
title('IMU Acceleration - Forward/Backward (X Axis)');
grid on;

subplot(3,1,2);
plot(time, acceleration(:,2));
xlabel('Time (ms)');
ylabel('Accel Y (m/s^2)');
title('IMU Acceleration - Lateral (Y Axis)');
grid on;

subplot(3,1,3);
plot(time, acceleration(:,3));
xlabel('Time (ms)');
ylabel('Accel Z (m/s^2)');
title('IMU Acceleration - Vertical (Z Axis)');
grid on;

sgtitle('IMU Acceleration Signals Over Time');
