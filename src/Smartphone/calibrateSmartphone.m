function cameraParams = calibrateSmartphone ()
    % Path to dataset
    dataset_path = '/home/tonirv/Downloads/smartphone_dataset/calibration/';
    
    % Parameters
    debug = true;
    squareSize = 20; % in units of 'mm' i.e 20mm

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
    
    % Detect checkerboards in images
    [imagePoints, boardSize, imagesUsed] = detectCheckerboardPoints(imageFileNames);
    %imageFileNames = imageFileNames(imagesUsed);

    % Generate world coordinates of the corners of the squares
    worldPoints = generateCheckerboardPoints(boardSize, squareSize);

    % Calibrate the camera
    [cameraParams, ~, estimationErrors] = estimateCameraParameters(imagePoints, worldPoints, ...
        'EstimateSkew', false, 'EstimateTangentialDistortion', false, ...
        'NumRadialDistortionCoefficients', 2, 'WorldUnits', 'mm', ...
        'InitialIntrinsicMatrix', [], 'InitialRadialDistortion', []);
    
    if (debug)
        % View reprojection errors
        figure; showReprojectionErrors(cameraParams, 'BarGraph');
        % Visualize pattern locations
        figure; showExtrinsics(cameraParams, 'CameraCentric');
        % Display parameter estimation errors
        displayErrors(estimationErrors, cameraParams);
    end
end
