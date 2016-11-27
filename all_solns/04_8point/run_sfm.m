clear all;
close all;
clc;

addpath('8point/');
addpath('triangulation/');
addpath('plot/');

% img = imread('../data/0001.jpg');
% img_2 = imread('../data/0002.jpg');
% K = [1379.74 0 760.35
%     0 1382.08 503.41
%     0 0 1 ];

img = rgb2gray(imread('../data/vfr/img0001_0.png'));
img_2 = rgb2gray(imread('../data/vfr/img0010_0.png'));
K = load('../data/vfr/K.txt');


%% Load outlier-free point correspondences

% p1 = load('../data/matches0001.txt');
% p2 = load('../data/matches0002.txt');

I1 = img;
I2 = img_2;

points1 = detectHarrisFeatures(I1, 'MinQuality', 0.2);
points2 = detectHarrisFeatures(I2, 'MinQuality', 0.2);

[features1,valid_points1] = extractFeatures(I1,points1);
[features2,valid_points2] = extractFeatures(I2,points2);

indexPairs = matchFeatures(features1,features2);

matchedPoints1 = valid_points1(indexPairs(:,1),:);
matchedPoints2 = valid_points2(indexPairs(:,2),:);

p1 = [matchedPoints1.Location ones(matchedPoints1.Count,1)]';
p2 = [matchedPoints2.Location ones(matchedPoints2.Count,1)]';

%% Estimate the essential matrix E using the 8-point algorithm

E = estimateEssentialMatrix(p1, p2, K, K);

%% Extract the relative camera positions (R,T) from the essential matrix

% Obtain extrinsic parameters (R,t) from E
[Rots,u3] = decomposeEssentialMatrix(E);

% Disambiguate among the four possible configurations
[R_C2_W,T_C2_W] = disambiguateRelativePose(Rots,u3,p1,p2,K,K);
% R_C2_W = Rots(:,:,2);
% T_C2_W = u3;

% Triangulate a point cloud using the final transformation (R,T)
M1 = K * eye(3,4);
M2 = K * [R_C2_W, T_C2_W];
P = linearTriangulation(p1,p2,M1,M2);

%% Visualize the 3-D scene
figure(1),
subplot(1,3,1)

% R,T should encode the pose of camera 2, such that M1 = [I|0] and M2=[R|t]

% P is a [4xN] matrix containing the triangulated point cloud (in
% homogeneous coordinates), given by the function linearTriangulation
plot3(P(1,:), P(2,:), P(3,:), 'o');

% Display camera pose

plotCoordinateFrame(eye(3),zeros(3,1), 0.8);
% text(-0.1,-0.1,-0.1,'Cam 1','fontsize',10,'color','k','FontWeight','bold');

center_cam2_W = -R_C2_W'*T_C2_W;
plotCoordinateFrame(R_C2_W',center_cam2_W, 0.8);
% text(center_cam2_W(1)-0.1, center_cam2_W(2)-0.1, center_cam2_W(3)-0.1,'Cam 2','fontsize',10,'color','k','FontWeight','bold');

axis equal
rotate3d on;
grid

% Display matched points
subplot(1,3,2)
imshow(img,[]);
hold on
plot(p1(1,:), p1(2,:), 'ys');
title('Image 1')

subplot(1,3,3)
imshow(img_2,[]);
hold on
plot(p2(1,:), p2(2,:), 'ys');
title('Image 2')
