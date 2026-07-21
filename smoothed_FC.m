function [smoothedFootClearance] = smoothed_FC(footClearance)
smoothingWindow = 10; 
smoothedFootClearance = movmean(footClearance, smoothingWindow);
end