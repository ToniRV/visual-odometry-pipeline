function cameraParams = calibrateSmartphone (parameters)
    smartphone_images_path = '/home/tonirv/Downloads/smartphone_dataset/';

    % Define images to process for calibration
    imageFileNames = {strcat(smartphone_images_path, 'Foto 06.01.17, 20 04 45.jpg'),...
        strcat(smartphone_images_path, 'Foto 06.01.17, 20 04 50.jpg'),...
        strcat(smartphone_images_path, 'Foto 06.01.17, 20 04 54.jpg'),...
        strcat(smartphone_images_path, 'Foto 06.01.17, 20 05 14.jpg'),...
        strcat(smartphone_images_path, 'Foto 06.01.17, 20 05 20.jpg'),...
        strcat(smartphone_images_path, 'Foto 06.01.17, 20 05 28.jpg'),...
        strcat(smartphone_images_path, 'Foto 06.01.17, 20 05 31.jpg'),...
        strcat(smartphone_images_path, 'Foto 06.01.17, 20 05 39.jpg'),...
        };

    % Detect checkerboards in images
    [imagePoints, boardSize, imagesUsed] = detectCheckerboardPoints(imageFileNames);
    imageFileNames = imageFileNames(imagesUsed);

    % Generate world coordinates of the corners of the squares
    squareSize = 20;  % in units of 'mm'
    worldPoints = generateCheckerboardPoints(boardSize, squareSize);

    % Calibrate the camera
    [cameraParams, imagesUsed, estimationErrors] = estimateCameraParameters(imagePoints, worldPoints, ...
        'EstimateSkew', false, 'EstimateTangentialDistortion', false, ...
        'NumRadialDistortionCoefficients', 2, 'WorldUnits', 'mm', ...
        'InitialIntrinsicMatrix', [], 'InitialRadialDistortion', []);
debug_with_figures = true;
    if (debug_with_figures)
        % View reprojection errors
        h1=figure; showReprojectionErrors(cameraParams, 'BarGraph');
        % Visualize pattern locations
        h2=figure; showExtrinsics(cameraParams, 'CameraCentric');
        % Display parameter estimation errors
        displayErrors(estimationErrors, cameraParams);
    end
end