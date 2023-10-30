function [zoomedROI,  detect]= detectSquare(img)
    % Detect Harris corner points
    I = rgb2gray(img);
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

    % Initialize zoomedROI as an empty array
    zoomedROI = [];
    detect = false;
    % Check if top corner points (topLeft and topRight) are detected
    if ~isempty(topLeft) && ~isempty(topRight)
        % Calculate orientation based on the top line
        dy = topRight(2) - topLeft(2);
        dx = topRight(1) - topLeft(1);
        angleRadians = atan2(dy, dx);
        angleDegrees = rad2deg(angleRadians);
        rotationAngle = angleDegrees;
        correctedImage = imrotate(I, rotationAngle, 'bicubic', 'crop');

        % Calculate the bounding box of the detected corners
        minX = min([topLeft(1), topRight(1), bottomLeft(1), bottomRight(1)]);
        maxX = max([topLeft(1), topRight(1), bottomLeft(1), bottomRight(1)]);
        minY = min([topLeft(2), topRight(2), bottomLeft(2), bottomRight(2)]);
        maxY = max([topLeft(2), topRight(2), bottomLeft(2), bottomRight(2)]);

        % Define a region of interest (ROI) based on the bounding box
        roi = [minX, minY, maxX - minX, maxY - minY];

        % Extract the ROI from the corrected image
        zoomedROI = imcrop(correctedImage, roi);
        detect = true;
    end
end
