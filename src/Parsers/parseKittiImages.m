function [img0, img1] = parseKittiImages(kitti_path, bootstrap_frames, initialisation)
% Parse pair of images from Kitti dataset
    if (strcmp(initialisation,'Monocular'))
        img0 = imread([kitti_path '/00/image_left/' ...
            sprintf('%06d.png',bootstrap_frames(1))]);
        img1 = imread([kitti_path '/00/image_left/' ...
            sprintf('%06d.png',bootstrap_frames(2))]);
    elseif (strcmp(initialisation, 'Stereo'))
        % left image
        img0 = imread([kitti_path '/00/image_left/' ...
            sprintf('%06d.png',bootstrap_frames(1))]);
        % right image
        img1 = imread([kitti_path '/00/image_right/' ...
            sprintf('%06d.png',bootstrap_frames(2))]);
    else
        disp(['Not loading images for initialisation: ', initialisation]);
        img0 = 0;
        img1 = 0;
    end
end

