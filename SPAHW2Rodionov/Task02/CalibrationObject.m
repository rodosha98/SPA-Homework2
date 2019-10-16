
clear all; clc;
% Define images to process
imageFileNames = {'/Users/rodosha/Desktop/Semestr 1/Sensing/SPAHW2Rodionov/Task02/PhotosetChessboard/3.png',...
    '/Users/rodosha/Desktop/Semestr 1/Sensing/SPAHW2Rodionov/Task02/PhotosetChessboard/4.png',...
    '/Users/rodosha/Desktop/Semestr 1/Sensing/SPAHW2Rodionov/Task02/PhotosetChessboard/7.png',...
    '/Users/rodosha/Desktop/Semestr 1/Sensing/SPAHW2Rodionov/Task02/PhotosetChessboard/8.png',...
    '/Users/rodosha/Desktop/Semestr 1/Sensing/SPAHW2Rodionov/Task02/PhotosetChessboard/10.png',...
    '/Users/rodosha/Desktop/Semestr 1/Sensing/SPAHW2Rodionov/Task02/PhotosetChessboard/16.png',...
    '/Users/rodosha/Desktop/Semestr 1/Sensing/SPAHW2Rodionov/Task02/PhotosetChessboard/19.png',...
    '/Users/rodosha/Desktop/Semestr 1/Sensing/SPAHW2Rodionov/Task02/PhotosetChessboard/20.png',...
    '/Users/rodosha/Desktop/Semestr 1/Sensing/SPAHW2Rodionov/Task02/PhotosetChessboard/23.png',...
    '/Users/rodosha/Desktop/Semestr 1/Sensing/SPAHW2Rodionov/Task02/PhotosetChessboard/24.png',...
    '/Users/rodosha/Desktop/Semestr 1/Sensing/SPAHW2Rodionov/Task02/PhotosetChessboard/25.png',...
    '/Users/rodosha/Desktop/Semestr 1/Sensing/SPAHW2Rodionov/Task02/PhotosetChessboard/26.png',...
    '/Users/rodosha/Desktop/Semestr 1/Sensing/SPAHW2Rodionov/Task02/PhotosetChessboard/27.png',...
    '/Users/rodosha/Desktop/Semestr 1/Sensing/SPAHW2Rodionov/Task02/PhotosetChessboard/31.png',...
    '/Users/rodosha/Desktop/Semestr 1/Sensing/SPAHW2Rodionov/Task02/PhotosetChessboard/32.png',...
    '/Users/rodosha/Desktop/Semestr 1/Sensing/SPAHW2Rodionov/Task02/PhotosetChessboard/33.png',...
    '/Users/rodosha/Desktop/Semestr 1/Sensing/SPAHW2Rodionov/Task02/PhotosetChessboard/35.png',...
    '/Users/rodosha/Desktop/Semestr 1/Sensing/SPAHW2Rodionov/Task02/PhotosetChessboard/37.png',...
    '/Users/rodosha/Desktop/Semestr 1/Sensing/SPAHW2Rodionov/Task02/PhotosetChessboard/39.png',...
    '/Users/rodosha/Desktop/Semestr 1/Sensing/SPAHW2Rodionov/Task02/PhotosetChessboard/41.png',...
    };
% Detect checkerboards in images
[imagePoints, boardSize, imagesUsed] = detectCheckerboardPoints(imageFileNames);
imageFileNames = imageFileNames(imagesUsed);

% Read the first image to obtain image size
originalImage = imread(imageFileNames{1});
[mrows, ncols, ~] = size(originalImage);

% Generate world coordinates of the corners of the squares
squareSize = 33;  % in units of 'millimeters'
worldPoints = generateCheckerboardPoints(boardSize, squareSize);

% Calibrate the camera
[cameraParams, imagesUsed, estimationErrors] = estimateCameraParameters(imagePoints, worldPoints, ...
    'EstimateSkew', true, 'EstimateTangentialDistortion', true, ...
    'NumRadialDistortionCoefficients', 2, 'WorldUnits', 'millimeters', ...
    'InitialIntrinsicMatrix', [], 'InitialRadialDistortion', [], ...
    'ImageSize', [mrows, ncols]);

% View reprojection errors
h1=figure; showReprojectionErrors(cameraParams);

% Visualize pattern locations
h2=figure; showExtrinsics(cameraParams, 'CameraCentric');

% Display parameter estimation errors
displayErrors(estimationErrors, cameraParams);

% For example, you can use the calibration data to remove effects of lens distortion.
undistortedImage = undistortImage(originalImage, cameraParams);

% See additional examples of how to use the calibration data.  At the prompt type:
% showdemo('MeasuringPlanarObjectsExample')
% showdemo('StructureFromMotionExample')

%% Object Finding and distance estimation
magnification = 50;
%STEP1. Image Reading
imOrig = imread('/Users/rodosha/Desktop/Semestr 1/Sensing/SPAHW2Rodionov/Task02/ChessBox.png');
figure; imshow(imOrig, 'InitialMagnification', magnification);
title('Input Image');

%STEP2. Image Reading and Undistorting 

% Since the lens introduced little distortion, use 'full' output view to illustrate that
% the image was undistored. If we used the default 'same' option, it would be difficult
% to notice any difference when compared to the original image. Notice the small black borders.
[im, newOrigin] = undistortImage(imOrig, cameraParams, 'OutputView', 'full');
figure; imshow(im, 'InitialMagnification', magnification);
title('Undistorted Image');

% STEP3. Convert the image to the HSV color space.
imHSV = rgb2hsv(im);

% Get the saturation channel.
saturation = imHSV(:, :, 2);

% Threshold the image
t = graythresh(saturation);
imMybox = (saturation > t);

figure; imshow(imMybox, 'InitialMagnification', magnification);
title('Segmented Box');

% STEP4.Find connected components.
blobAnalysis = vision.BlobAnalysis('AreaOutputPort', true,...
    'CentroidOutputPort', false,...
    'BoundingBoxOutputPort', true,...
    'MinimumBlobArea', 200, 'ExcludeBorderBlobs', true);
[areas, boxes] = step(blobAnalysis, imMybox);

% Sort connected components in descending order by area
[~, idx] = sort(areas, 'Descend');

% Get the largest component.
boxes = double(boxes(idx(1), :));

% Reduce the size of the image for display.
scale = magnification / 100;
imDetectedCoins = imresize(im, scale);

% Insert labels for the Box.
imDetectedBox = insertObjectAnnotation(imDetectedCoins, 'rectangle', ...
    scale * boxes, 'box');
figure; imshow(imDetectedBox);
title('Detected Box');

% STEP 5 Compute Extrinsics parameters for our case

% Detect the checkerboard.
[imagePoints, boardSize] = detectCheckerboardPoints(im);
% Adjust the imagePoints so that they are expressed in the coordinate system
% used in the original image, before it was undistorted.  This adjustment
% makes it compatible with the cameraParameters object computed for the original image.
imagePoints = imagePoints + newOrigin; % adds newOrigin to every row of imagePoints

% Compute rotation and translation of the camera.
% Image 15 has min error

[R, t] = extrinsics(imagePoints, worldPoints, cameraParams);

%STEP 6 Assignment of the 4 points (corners od rectangle)

% Adjust upper left corners of bounding boxes for coordinate system shift 
% caused by undistortImage with output view of 'full'. This would not be
% needed if the output was 'same'. The adjustment makes the points compatible
% with the cameraParameters of the original image.

newboxes(1) = boxes(1) + newOrigin(1);
newboxes(2) = boxes(2) + newOrigin(2);
newboxes(3) = boxes(3)
newboxes(4) = boxes(4)
% zero padding is added for widht and height

% Get the top-left and the top-right corners.
box1 = double(newboxes(1, :));
% Define left up and right down corners
imagePoints1 = [box1(1:2); ...
                box1(1)+box1(3), box1(2)+box1(4)];

% Get the world coordinates of the corners            
worldPoints1 = pointsToWorld(cameraParams, R, t, imagePoints1);
worldPoints1
height = abs(worldPoints1(1,2) - worldPoints1(2,2))
width = abs(worldPoints1(1,1) - worldPoints1(2,1))
true_height = 130 %mm
true_width = 95 %mm 

% STEP 7 Distance estimation 
% Compute the center of the first coin in the image.
center1_image =[ box1(1)+box1(3)/2, box1(2) + box1(4)/2] ;

% Convert to world coordinates.
center1_world  = pointsToWorld(cameraParams, R, t, center1_image);

% Remember to add the 0 z-coordinate.
center1_world = [center1_world 0];

% Compute the distance to the camera.
[~, cameraLocation] = extrinsicsToCameraPose(R, t);
distanceToCamera = norm(center1_world - cameraLocation);
fprintf('Distance from the camera to the box plane = %0.2f mm\n', ...
    distanceToCamera);
