clc;
clf;

I = imread('square (1).png');
I = rgb2gray(I);

% Detect Harris corner points
cornerPoints1 = detectHarrisFeatures(I);
locations = cornerPoints1.Location;

% Sort by y-value
[~, sortedIndices] = sort(locations(:, 2));

% Take top and bottom subsets
numPoints = size(locations, 1);
topSubset = sortedIndices(1:round(0.05 * numPoints)); 
bottomSubset = sortedIndices(end-round(0.05 * numPoints)+1:end);

% Determine top-left, top-right, bottom-left, bottom-right
[~, idxTopLeft] = min(locations(topSubset, 1));
[~, idxTopRight] = max(locations(topSubset, 1));
[~, idxBottomLeft] = min(locations(bottomSubset, 1));
[~, idxBottomRight] = max(locations(bottomSubset, 1));

topLeft = locations(topSubset(idxTopLeft), :);
topRight = locations(topSubset(idxTopRight), :);
bottomLeft = locations(bottomSubset(idxBottomLeft), :);
bottomRight = locations(bottomSubset(idxBottomRight), :);

% Display the image and detected corners
subplot(1,2,1);
imshow(I);
hold on;
plot(cornerPoints1);
plot(topLeft(1), topLeft(2), 'ro');
plot(topRight(1), topRight(2), 'ro');
plot(bottomLeft(1), bottomLeft(2), 'ro');
plot(bottomRight(1), bottomRight(2), 'ro');


% Calculate orientation based on the top line
dy = topRight(2) - topLeft(2);
dx = topRight(1) - topLeft(1);
angleRadians = atan2(dy, dx);
angleDegrees = rad2deg(angleRadians);
rotationAngle = angleDegrees;
correctedImage = imrotate(I, rotationAngle, 'bicubic', 'crop');

title('Original Image');
hold on;

subplot(1,2,2);
imshow(correctedImage);
title('Corrected Image');