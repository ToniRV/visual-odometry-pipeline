function [img0, img1] = parseParkingImages(parking_path, bootstrap_frames)
% Parse pair of images from Parking dataset
    img0 = rgb2gray(imread([parking_path ...
        sprintf('/images/img_%05d.png',bootstrap_frames(1))]));
    img1 = rgb2gray(imread([parking_path ...
        sprintf('/images/img_%05d.png',bootstrap_frames(2))]));
end

