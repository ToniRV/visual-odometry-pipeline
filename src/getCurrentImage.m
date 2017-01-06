function image = getImage(dataset, idx)
   switch dataset
        case 'Kitti'
            image = imread([kitti_path_ '/00/image_0/' ...
                sprintf('%06d.png',idx)]);
        case 'Malaga'
            image = rgb2gray(imread([malaga_path_ ...
             '/malaga-urban-dataset-extract-07_rectified_800x600_Images/' ...
             left_images(idx).name]));
        case 'Parking'
            image = rgb2gray(imread([parking_path_ ...
               sprintf('/images/img_%05d.png',idx)]));
        otherwise
            assert(false);
    end
end