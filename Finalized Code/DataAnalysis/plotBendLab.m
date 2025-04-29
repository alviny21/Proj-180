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

% Extract time (first column) and data (columns 2 to 9)
time = data{:, 1};
sensorData = data{:, 2:9};
headers = data.Properties.VariableNames(2:9);

% Flip the sign of FarHipX (column 6) and FarKneeX (column 8)
sensorData(:, 5) = -sensorData(:, 5); % Column 6 (FarHipX)
sensorData(:, 7) = -sensorData(:, 7); % Column 8 (FarKneeX)

% Identify knee flexion (X) and abduction (Y) indices
idx_CloseKneeX = find(strcmp(headers, 'CloseKneeX'));
idx_FarKneeX = find(strcmp(headers, 'FarKneeX'));
idx_CloseKneeY = find(strcmp(headers, 'CloseKneeY'));
idx_FarKneeY = find(strcmp(headers, 'FarKneeY'));

%% ----- Apply Lowpass Filtering -----

% Filter settings
Fs = 100; % Assumed sample rate in Hz (you can adjust if known!)
Fc = 1.0; % Cutoff frequency in Hz (around 1 Hz keeps human movements, filters noise)
[b, a] = butter(4, Fc/(Fs/2), 'low'); % 4th-order lowpass Butterworth filter

% Apply the filter to knee angles only
sensorData(:, idx_CloseKneeX) = filtfilt(b, a, sensorData(:, idx_CloseKneeX));
sensorData(:, idx_FarKneeX) = filtfilt(b, a, sensorData(:, idx_FarKneeX));
sensorData(:, idx_CloseKneeY) = filtfilt(b, a, sensorData(:, idx_CloseKneeY));
sensorData(:, idx_FarKneeY) = filtfilt(b, a, sensorData(:, idx_FarKneeY));

% (Optionally filter hips too if needed — easy to add.)

%% --------- Re-Plot After Filtering ---------

% Flexion/Extension
figure;

subplot(2,1,1);
hold on;
plot(time, sensorData(:, find(strcmp(headers, 'CloseHipX'))), 'DisplayName', 'CloseHipX');
plot(time, sensorData(:, find(strcmp(headers, 'FarHipX'))), 'DisplayName', 'FarHipX');
hold off;
xlabel('Time (ms)');
ylabel('Bending Angle (degrees)');
title('Hip Flexion/Extension');
legend('show');
grid on;

subplot(2,1,2);
hold on;
plot(time, sensorData(:, idx_CloseKneeX), 'DisplayName', 'CloseKneeX');
plot(time, sensorData(:, idx_FarKneeX), 'DisplayName', 'FarKneeX');
hold off;
xlabel('Time (ms)');
ylabel('Bending Angle (degrees)');
title('Knee Flexion/Extension');
legend('show');
grid on;

sgtitle('Flexion/Extension Angles');

% Abduction/Adduction
figure;

subplot(2,1,1);
hold on;
plot(time, sensorData(:, find(strcmp(headers, 'CloseHipY'))), 'DisplayName', 'CloseHipY');
plot(time, sensorData(:, find(strcmp(headers, 'FarHipY'))), 'DisplayName', 'FarHipY');
hold off;
xlabel('Time (ms)');
ylabel('Bending Angle (degrees)');
title('Hip Adduction');
legend('show');
grid on;

subplot(2,1,2);
hold on;
plot(time, sensorData(:, idx_CloseKneeY), 'DisplayName', 'CloseKneeY');
plot(time, sensorData(:, idx_FarKneeY), 'DisplayName', 'FarKneeY');
hold off;
xlabel('Time (ms)');
ylabel('Bending Angle (degrees)');
title('Knee Adduction');
legend('show');
grid on;

sgtitle('Abduction/Adduction Angles');
