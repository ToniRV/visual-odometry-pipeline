clear all;
close all;
clc

debug = true;

parameters = struct(...
    'debug', debug...
    );

% Get camera parameters
cameraParams = calibrateSmartphone(parameters);

% Create video reader
ReadFilename = 'video.mov';
videoFReader = vision.VideoFileReader(ReadFilename, 'ImageColorSpace', 'Intensity');

% Create video writer
WriteFilename = 'undistorted_video.mov';
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