

% Path to dataset
dataset_path = '/home/tonirv/Downloads/frame_set_3/';
debug = false;

load('/home/tonirv/Documents/visual-odometry-pipeline/src/Smartphone/data/matlab.mat');

%Load images
imageNames = dir(fullfile(dataset_path,'*.jpg'));
imageNames = {imageNames.name}';

% Loop over frames
for i = 1:size(imageNames, 1)
    img_gray = rgb2gray(imread(fullfile(dataset_path,imageNames{i})));
    img_res = imresize(img_gray, 0.5);
    img_undistorted = undistortImage(img_res, cameraParams, 'OutputView', 'valid');
    if (debug)
        figure();
        subplot(1, 2, 1);
        imshow(img_gray);
        title('Distorted image');
        subplot(1, 2, 2);
        imshow(img_undistorted);
        title('Undistorted image');
    end
    imwrite(img_undistorted, strcat(int2str(i), '.jpg'));
end
