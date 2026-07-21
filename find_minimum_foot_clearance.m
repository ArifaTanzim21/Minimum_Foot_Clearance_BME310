clc
clear all
% Load the .mat file from the imu sensor data
data = load('MFC_set2.mat');
 
% Extract the relevant data from the loaded file
acceleration = data.Acc_R;  %  acceleration data
gyroscope = data.Gyro_R;    % gyroscope data
quaternion = data.Quat_R;   %quaternion data
 
% Parameters
g = 9.81;  % Acceleration due to gravity (m/s^2)
 
% Convert quaternion to rotation matrix
rotationMatrix = quat2rotm(quaternion);
 
%  sampling rate
samplingRate = 100; % samples per second
samplePeriod = 1 / samplingRate;
numSamples = size(acceleration, 1);
 
% Generate time vector
time = (0:numSamples-1) * samplePeriod;
 
% Initialize position array
position = zeros(numSamples, 3);
 
% Initialize velocity array
velocity = zeros(numSamples, 3);
 
% Initialize orientation variables
orientation = zeros(numSamples, 3);  % Euler angles (yaw, pitch, roll)
angularVelocity = zeros(numSamples, 3);  % Angular velocity
 
% Process each sample of sensor data
for i = 1:numSamples
    % Rotate the gravity vector using the rotation matrix
    rotatedGravity = rotationMatrix(:,:,i) * [0; 0; g];
  
    % Subtract the rotated gravity vector from the acceleration vector
    correctedAcceleration = acceleration(i, :)' - rotatedGravity;
  
    % Integrate gyroscope to get angular velocity
    if i > 1
        angularVelocity(i, :) = angularVelocity(i-1, :) + gyroscope(i, :) * samplePeriod;
    end
  
    % Integrate angular velocity to get orientation
    if i > 1
        orientation(i, :) = orientation(i-1, :) + angularVelocity(i, :) * samplePeriod;
    end
 
    % Convert orientation to rotation matrix
    orientationMatrix = eul2rotm(orientation(i, :));
 
    % Rotate the corrected acceleration using the orientation matrix
    rotatedAcceleration = orientationMatrix * correctedAcceleration;
  
    % Integrate acceleration to get velocity
    if i > 1
        velocity(i, :) = velocity(i-1, :) + rotatedAcceleration' * samplePeriod;
    end
  
    % Integrate velocity to get position
    if i > 1
        position(i, :) = position(i-1, :) + velocity(i, :) * samplePeriod;
    end
end
 
% Apply a low-pass filter to position data to remove noise
[footClearance] = noise_removal(position,samplingRate);

 
% Apply additional smoothing to the foot clearance data
[smoothedFootClearance] = smoothed_FC(footClearance);
 
% Find the time index with the minimum foot clearance
[minClearance, minIndex] = min(smoothedFootClearance);
 
% Convert minimum foot clearance to millimeter
minClearance_mm = minClearance * 1000; % Conversion from meters to millimeter
 
% Display the minimum foot clearance in millimeter and its corresponding time
fprintf('Minimum foot clearance: %.2f mm\n', minClearance_mm);
fprintf('Time: %.2f seconds\n', minIndex * samplePeriod);
 
% Plot the raw and filtered foot clearance over time
figure;
subplot(2,1,1);
plot(time, footClearance, 'b', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Foot Clearance (mm)');
title('Raw Foot Clearance');
grid on;
 
subplot(2,1,2);
plot(time, smoothedFootClearance, 'r', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Foot Clearance (mm)');
title('Filtered and Smoothed Foot Clearance');
grid on;
