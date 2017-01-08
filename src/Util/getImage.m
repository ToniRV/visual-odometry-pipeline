function image = getImage(dataset, idx, kitti_path, malaga_path, parking_path)
   switch dataset
        case 'Kitti'
            image = imread([kitti_path '/00/image_0/' ...
                sprintf('%06d.png', idx)]);
        case 'Malaga'
            image = rgb2gray(imread([malaga_path ...
             '/malaga-urban-dataset-extract-07_rectified_800x600_Images/' ...
             left_images(idx).name]));
        case 'Parking'
            image = rgb2gray(imread([parking_path ...
               sprintf('/images/img_%05d.png', idx)]));
       otherwise
            disp(['Wrong dataset specified: ', dataset]);
            assert(false);
    end
end