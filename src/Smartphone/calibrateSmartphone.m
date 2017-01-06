function cameraParams = calibrateSmartphone (parameters)
    imageFileNames = parameters.imageFileNames;
    debug = parameters.debug;

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
    if (debug)
        % View reprojection errors
        h1=figure; showReprojectionErrors(cameraParams, 'BarGraph');
        % Visualize pattern locations
        h2=figure; showExtrinsics(cameraParams, 'CameraCentric');
        % Display parameter estimation errors
        displayErrors(estimationErrors, cameraParams);
    end
end
