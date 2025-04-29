% Load data
[file, path] = uigetfile('4_26_duration_test.csv', 'Select the CSV file');
if isequal(file, 0)
    disp('User selected Cancel');
    return;
else
    filepath = fullfile(path, file);
end

data = readtable(filepath);
time = data{:,1};

% Define segment lengths (in cm)
thigh_length = 40; % cm
shank_length = 37; % cm

% Extract relevant angles
left_hip_flexion = data.CloseHipX;   % CloseHipX = Left Hip Flex/Ext
right_hip_flexion = data.FarHipX;    % FarHipX = Right Hip Flex/Ext
left_knee_flexion = data.CloseKneeX; % CloseKneeX = Left Knee Flex/Ext
right_knee_flexion = data.FarKneeX;  % FarKneeX = Right Knee Flex/Ext

% Limit data to first hour
max_time_ms = 1 * 3600 * 1000; % 1 hour = 3.6 million ms
valid_idx = find(time <= max_time_ms);

% Corrected Sampling Rate
Fs = 92.8; % ~93 Hz based on time axis
sampling_rate = 5; % 5 poses per second you want
step_size = round(Fs / sampling_rate); % ~18 or 19 samples

% Sample 30 seconds worth of data
window_samples = round(30 * Fs); % 30 seconds

% Pick a moving window (already restricted to first hour)
valid_idx = find(time <= 3600 * 1000); % 1 hour = 3600000 ms
window_energy = movvar(left_knee_flexion(valid_idx) + right_knee_flexion(valid_idx), 5000);

[~, max_idx_local] = max(window_energy);
start_idx = valid_idx(max_idx_local);
start_idx = max(1, min(start_idx, length(time) - window_samples));
end_idx = start_idx + window_samples - 1;

% Now downsample cleanly
sampled_time = time(start_idx:step_size:end_idx);
sampled_left_hip_flex = deg2rad(left_hip_flexion(start_idx:step_size:end_idx));
sampled_right_hip_flex = deg2rad(right_hip_flexion(start_idx:step_size:end_idx));
sampled_left_knee_flex = deg2rad(left_knee_flexion(start_idx:step_size:end_idx));
sampled_right_knee_flex = deg2rad(right_knee_flexion(start_idx:step_size:end_idx));

% Save start timestamp
start_timestamp = time(start_idx);

%% -------- Animate Both Legs --------
figure;
axis equal;
axis([-100 100 -100 100]);
xlabel('X (cm)');
ylabel('Z (cm)');
title(sprintf('Pose Estimation (Both Legs) starting at Timestamp: %d ms', start_timestamp));
grid on;
hold on;

for i = 1:length(sampled_time)
    cla; % Clear previous frame
    
    % Pelvis Origin
    pelvis_x = 0;
    pelvis_z = 0;
    
    % Left Leg Calculation
    left_thigh_x = thigh_length * sin(sampled_left_hip_flex(i));
    left_thigh_z = -thigh_length * cos(sampled_left_hip_flex(i));
    
    left_total_knee_angle = sampled_left_hip_flex(i) + sampled_left_knee_flex(i);
    left_shank_x = left_thigh_x + shank_length * sin(left_total_knee_angle);
    left_shank_z = left_thigh_z - shank_length * cos(left_total_knee_angle);
    
    % Right Leg Calculation
    right_thigh_x = thigh_length * sin(sampled_right_hip_flex(i));
    right_thigh_z = -thigh_length * cos(sampled_right_hip_flex(i));
    
    right_total_knee_angle = sampled_right_hip_flex(i) + sampled_right_knee_flex(i);
    right_shank_x = right_thigh_x + shank_length * sin(right_total_knee_angle);
    right_shank_z = right_thigh_z - shank_length * cos(right_total_knee_angle);

    % Draw Left Leg (Blue)
    plot([pelvis_x, left_thigh_x], [pelvis_z, left_thigh_z], 'b-', 'LineWidth', 3); % Left thigh
    plot([left_thigh_x, left_shank_x], [left_thigh_z, left_shank_z], 'b--', 'LineWidth', 3); % Left shank
    
    % Draw Right Leg (Red)
    plot([pelvis_x, right_thigh_x], [pelvis_z, right_thigh_z], 'r-', 'LineWidth', 3); % Right thigh
    plot([right_thigh_x, right_shank_x], [right_thigh_z, right_shank_z], 'r--', 'LineWidth', 3); % Right shank

    % Formatting
    xlabel('X (cm)');
    ylabel('Z (cm)');
    title(sprintf('Pose Estimation (Both Legs) starting at Timestamp: %d ms', start_timestamp));
    axis([-100 100 -100 100]);
    grid on;
    drawnow;
    
    pause(0.1); % 10 fps playback
end

hold off;
