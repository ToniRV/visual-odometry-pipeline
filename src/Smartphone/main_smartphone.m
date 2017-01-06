clear all;
close all;
clc

debug = true;

% Path to dataset
dataset_path = '/home/tonirv/Downloads/smartphone_dataset/';

% Define images to process for calibration
imageFileNames = {strcat(dataset_path, 'Foto 06.01.17, 20 04 45.jpg'),...
    strcat(dataset_path, 'Foto 06.01.17, 20 04 50.jpg'),...
    strcat(dataset_path, 'Foto 06.01.17, 20 04 54.jpg'),...
    strcat(dataset_path, 'Foto 06.01.17, 20 05 14.jpg'),...
    strcat(dataset_path, 'Foto 06.01.17, 20 05 20.jpg'),...
    strcat(dataset_path, 'Foto 06.01.17, 20 05 28.jpg'),...
    strcat(dataset_path, 'Foto 06.01.17, 20 05 31.jpg'),...
    strcat(dataset_path, 'Foto 06.01.17, 20 05 39.jpg'),...
    };

parameters = struct(...
    'imageFileNames', imageFileNames,...
    'debug', debug...
    );

% Get camera parameters
cameraParams = calibrateSmartphone(parameters);

% Create video reader
ReadFilename = strcat(dataset_path, 'video.mov');
videoFReader = vision.VideoFileReader(ReadFilename, 'ImageColorSpace', 'Intensity');

% Create video writer
WriteFilename = strcat(dataset_path, 'undistorted_video.avi');
videoFWriter = vision.VideoFileWriter(WriteFilename, 'FileFormat', 'AVI', 'FrameRate',videoFReader.info.VideoFrameRate);

% Create video player
videoPlayer = vision.VideoPlayer;

% Loop over frames
while ~isDone(videoFReader)
    I = step(videoFReader);
    undistorted_I = undistortImage(I, cameraParams);
    step(videoFWriter, undistorted_I);
    if (debug)
        step(videoPlayer, videoFrame);
    end
end
release(videoPlayer);
release(videoFReader);
release(videoFWriter);
