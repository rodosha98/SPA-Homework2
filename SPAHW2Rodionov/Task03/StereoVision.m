clear all; clc;
% Given
fl = 2.8 % mm Focal length 

baseline = 100 % mm
% Intrinsic matrix of the camera with which the images were taken
K = [10 0.000000e+00 0; ...
 0.000000e+00 10  0 ; ...
 0.000000e+00 0.000000e+00 1.000000e+00];
K1 = K;
K2 = K;

magnification = 100;
%STEP1. Image Reading
imOrigin1 = imread('/Users/rodosha/Desktop/Semestr 1/Sensing/Homework2/Task03/18left.png');
imOrigin2 = imread('/Users/rodosha/Desktop/Semestr 1/Sensing/Homework2/Task03/18right.png');

% STEP 2 Convert images in gray scale for SURF
im1 = rgb2gray(imOrigin1);
im2 = rgb2gray(imOrigin2);
[mrows, ncols, ~] = size(im1);

% Plot Images
figure(1); imshow(im1, 'InitialMagnification', magnification);
title('Input Left Image');

figure(2); imshow(im2, 'InitialMagnification', magnification);
title('Input Right Image');

% Step 3. SURF detection of keypoints

blobs1 = detectSURFFeatures(im1);
blobs2 = detectSURFFeatures(im2);
figure(3);
imshow(im1);
hold on;
plot(selectStrongest(blobs1, 10));
title('Ten strongest SURF features in Image 1 (Left)');

figure(4);
imshow(im2);
hold on;
plot(selectStrongest(blobs2, 10));

title('Thirty strongest SURF features in Image 2 (Right)');

% Extracting features (key points)

[features1, validpoints1] = extractFeatures(im1, blobs1);
[features2, validpoints2] = extractFeatures(im2, blobs2);

%STEP4  Matching points
indexPairs = matchFeatures(features1, features2);
matchedPoints1 = validpoints1(indexPairs(:,1),:);
matchedPoints2 = validpoints2(indexPairs(:,2),:);

% Plot matched Points
figure(5);
showMatchedFeatures(im1, im2, matchedPoints1, matchedPoints2);
legend('Putatively matched points in Image1', 'Putatively matched points in Image 2');
%N = 10;

N = matchedPoints1.Count;
%STEP5  Detecting Image coordinates of coreesponding points

for i=1:N
    xl(i,:)=matchedPoints1.Location(i,1);
    yl(i,:)=matchedPoints1.Location(i,2);
    xr(i,:)=matchedPoints2.Location(i,1);
    yr(i,:)=matchedPoints2.Location(i,2);
end
%

% 
% Show corresponding points on both images
figure(6) ; clf;

imshow(cat(2, im1, im2)) ;
axis image off ;
hold on ;
title('Both images in one figure');

%
xrr=xr+size(im1,2);% Because Origin of the second image moved in right on width of the first image
set=[1:matchedPoints1.Count];
h = line([xl(set)' ; xrr(set)'], [yl(set)' ; yr(set)']) ;

% STEP6 Fundamental Matrix F Estimation 

points1=[xl,yl];
points2=[xr,yr];

F=eightpoint(points1,points2);

%STEP7 Epipolar Line Estimation
k = 10
% Compute epipolar lines
epiLines1 = epipolarLine(F',points1(1: k, :));
pLine1 = lineToBorderPoints(epiLines1, size(im1));
epiLines2 = epipolarLine(F, points2(1: k, :));
pLine2 = lineToBorderPoints(epiLines2, size(im2));
hold off;

% Draw the epipolar lines
figure(7);
imshow(im1);
hold on;
line(pLine1(:, [1, 3])', pLine1(:, [2, 4])',...
    'LineWidth', 1.5, 'color', 'w');
plot(points1(1: k, 1), points1(1: k, 2), 'ro', 'LineWidth', 2);
title('Epipolar lines of the 1st image (LEFT)');
figure(8);
imshow(im2);
line(pLine2(:, [1, 3])', pLine2(:, [2, 4])',...
    'LineWidth', 1.5, 'color', 'w');
hold on;
plot(points2(1: k, 1), points2(1: k, 2), 'co', 'LineWidth', 2);
title('Epipolar lines of the 2nd image (RIGHT)');

%% Disparity map

%STEP8 Rectified stereo pair image
A = stereoAnaglyph(im1,im2);
figure(8)
imshow(A)
title('View of the rectified stereo pair image')
disparityRange = [0 48];
disparityMap = disparity(im1,im2,'DisparityRange',disparityRange,'UniquenessThreshold',20);
figure(9)
imshow(disparityMap,disparityRange)
title('Disparity Map')
colormap jet
colorbar

