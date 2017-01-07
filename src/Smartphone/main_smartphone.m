clear all;
close all;
clc

% Path to dataset
dataset_path = '/home/tonirv/Downloads/smartphone_dataset/';
debug = false;

load('/home/tonirv/Documents/visual-odometry-pipeline/src/Smartphone/data/cameraParams.mat');
if(isempty(dataset_path))
    % Get camera parameters
    cameraParams = calibrateSmartphone;
end

%Load images
imageNames = dir(fullfile(dataset_path,'a*.jpeg'));
imageNames = {imageNames.name}';

% Loop over frames
for i = 1:size(imageNames, 1)
    img_gray = rgb2gray(imread(fullfile(dataset_path,imageNames{i})));
    img_undistorted = undistortImage(img_gray, cameraParams);
    if (debug)
        figure();
        subplot(1, 2, 1);
        imshow(img_gray);
        title('Distorted image');
        subplot(1, 2, 2);
        imshow(img_undistorted);
        title('Undistorted image');
    end
    imwrite(img_undistorted,strcat(i,'.jpg'));
end
