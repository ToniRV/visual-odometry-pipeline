function [img] = parseSmartphoneImages(dataset_path, index)
    %Load images
    imageNames = dir(fullfile(dataset_path,'*.jpg'));
    imageNames = {imageNames.name}';
    img = rgb2gray(imread(fullfile(dataset_path,imageNames{index})));
end