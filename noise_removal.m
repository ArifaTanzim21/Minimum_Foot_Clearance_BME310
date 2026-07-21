function [footClearance] = noise_removal(position,samplingRate)
cutoffFrequency = 2;
[b, a] = butter(2, cutoffFrequency / (samplingRate / 2));
filteredPosition = filtfilt(b, a, position);
footClearance = abs(filteredPosition(:, 3));
end