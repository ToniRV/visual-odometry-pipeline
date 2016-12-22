function [ points_2D, points_3D ] = stereo_initialisation( img_left, img_right , K, baseline, ...
                                                                                        triangulation_algorithm)
%
%   INPUT:
    %
    %   - img_left: image from the left camera of the stereo cameras.
    %   - img_right: image from the right camera of the stereo cameras.
    %   - K: 3x3 matrix with the intrinsics of the cameras, assumed to be
    %   identical for both cameras.
    %   - baseline: scalar with the baseline lenght between left and right
    %   camera.
    %   - triangulation_algorithm: possible values= 'ex_5_triangulation',
    %   'matlab_triangulation', 'disparity_triangulation'. Default:
    %   'disparity_triangulation'
%
%   OUTPUT:
    %
    %   - points_2D: 2D points in the image (img_left) that have been defined
    %   as keypoints and triangulated to 3D points in the world. 2xN vector
        %       * first row: each value corresponds to the row of the pixel in the
        %       image matrix.
        %       * second row: each value corresponds to the column of the pixel in the image
        %       matrix.
    %   - points_3D: 3D points in the world that correspond to each point in
    %   points_2D. These are expressed in the left camera frame (right handed frame
    %   with z pointing in front of the camera, x pointing to where the right camera is located
    %   see figure 2 exercise 4 for an illustration). 3xN vector
        %       * first row: x coordinate in left camera frame.
        %       * second row: y coordinate in left camera frame.
        %       * third row: z coordinate in left camera frame (should be
        %       >0).
    
    % If you want debugging figures
    debug_with_figures = true;
    
    % Select triangulation algorithm
    if (strcmp(triangulation_algorithm,'ex_5_triangulation'))
        triangulation_algorithm = 1;
    elseif (strcmp(triangulation_algorithm,'matlab_triangulation'))
        triangulation_algorithm = 2;
    elseif(strcmp(triangulation_algorithm,'disparity_triangulation'))
        triangulation_algorithm = 3; % only useful for stereo
    else
        fprintf('No valid triangulation algorithm specified, defaulting to: "disparity_triangulation" ');
    end
    
    %% Detect and match features: find 2D-2D correspondences
    
    tic;
    
    % TODO
    % This way of finding 2d2d correspondences is actually pretty dumb since I am not using the
    % knowledge that my images are stereo images. Hence, if the images are rectified, I should be able
    % to match features by simply exploring the epipolar line instead of looking at all the descriptors to
    % see matches
    
    % !!!!!!!!!!!!!!!!WARNING keypoints are in (row, col) coordinates of the image which might differ from
    % (u, v) coordinates, depending on whether u representes rows or columns!!!!!!!!!!!!!!!!!!!!!!
    [keypoints_left, keypoints_right,~,~] = correspondences_2d2d(img_left, img_right);
    fprintf('It took %ds to compute correspondences \n', toc);

    if (debug_with_figures)
        %Matlab convention, I have to flip keypoints :O
        figure(10); showMatchedFeatures(img_left, img_right, flipud(keypoints_left)', flipud(keypoints_right)', 'montage');
    end

    % Homogeneous coordinates of 2d-2d correspondences
    homo_keypoints_left = [keypoints_left ; ones(1, size(keypoints_left,2))]; % TODO not sure if this zeros should instead 
    homo_keypoints_right = [keypoints_right ; ones(1, size(keypoints_right,2))]; %be a SCALE factor or something of the kind


    % FLIPED Homogeneous coordinates of 2d-2d correspondences
    % The flipping is to switch from (row, col, 1) coords to (u, v, 1) coords,
    % u and v are cols and rows respectively... IT DEPENDS ON THE CONVENTION
    % USED...
    homo_keypoints_left_fliped = [flipud(keypoints_left) ; ones(1, size(keypoints_left,2))]; % TODO not sure if this zeros should instead 
    homo_keypoints_right_fliped = [flipud(keypoints_right) ; ones(1, size(keypoints_right,2))]; %be a SCALE factor or something of the kind

%% Triangulate: find 2D-3D correspondences

    % ex_5 linear triangulation algorithm        
    if (triangulation_algorithm == 1)
        addpath('../triangulation');
        tic;
        % Assuming identical K for each camera, and world frame identical to
        % left camera frame.
        M_left = K * [eye(3), [0; 0; 0]];
        M_right = K * [eye(3), [-baseline; 0; 0]]; % TODO check that this is correct, it actually depends on the Kitti coords.
                                                                                    % According to the coords in this paper http://www.mrt.kit.edu/z/publ/download/2013/GeigerAl2013IJRR.pdf

        % These are 3D points in camera left frame
        P_est = linearTriangulation(homo_keypoints_left_fliped, homo_keypoints_right_fliped, M_left, M_right);
        fprintf('It took %ds to compute linear ex_5_triangulation \n', toc);

        % TODO we should maybe get rid of triangulation behind the camera at this
        % step? This makes the 18th entrance in P_est disappear since it's z
        % component is negative, mind that the one from P_est_official isn't
        % negative, and the reprojection error is 0.5... So maybe we shouldn't keep
        % it anyway...
        valid_indices = P_est(3,:)>0;
        P_est = P_est(:, valid_indices);

        points_3D = P_est(1:3,:);
        points_2D =  keypoints_left(:, valid_indices); % TODO should we send (u, v) coords instead of (row, col)?

        % Official matlab triangulation algorithm
    elseif (triangulation_algorithm == 2)
        % TODO official triangulation given in matlab, the results are different
        % the course implementation, which one do we choose?
        % I have also to flip upside down the keypoints...
        % Again these are wrt camera Left frame
        % TODO a possible optimisation would be to get rid of points that have a
        % great reprojectionERROR which are the ones where both triangulations (the
        % matlab one and the one given in the exercices) differ.
        tic;
        
        % Assuming identical K for each camera, and world frame identical to
        % left camera frame.
        M_left = K * [eye(3), [0; 0; 0]];
        M_right = K * [eye(3), [-baseline; 0; 0]]; % TODO check that this is correct, it actually depends on the Kitti coords.
                                                                                    % According to the coords in this paper http://www.mrt.kit.edu/z/publ/download/2013/GeigerAl2013IJRR.pdf

        [P_est_official, reprojectionErrors] = triangulate(flipud(keypoints_left)', flipud(keypoints_right)', M_left', M_right');
        fprintf('It took %ds to compute matlab_triangulation \n', toc);

        P_est_official = P_est_official';

        % TODO we should maybe get rid of triangulation behind the camera at this
        % step? This makes the 18th entrance in P_est disappear since it's z
        % component is negative, mind that the one from P_est_official isn't
        % negative, and the reprojection error is 0.5... So maybe we shouldn't keep
        % it anyway...
        valid_indices = P_est_official(3,:)>0;
        P_est_official = P_est_official(:, valid_indices);

        points_3D = P_est_official;
        points_2D =  keypoints_left(:, valid_indices); % TODO should we send (u, v) coords instead of (row, col)?

    % Disparity based algorithm
    elseif (triangulation_algorithm == 3)
        % Adapt disparity functions to compute 3D points only for the keypoints found

        % WARNING Switch from (row, col, 1) to (u, v, 1), check figure 2 of exercice4.
        tic;
        
        A_col1 = K^-1 * homo_keypoints_left_fliped;
        A_col2 = K^-1 * homo_keypoints_right_fliped;

        camera_points = zeros(3, size(homo_keypoints_left_fliped,2)); % Number of 3D points

        b = [baseline; 0; 0];
        for i = 1:size(homo_keypoints_left_fliped, 2) % for each keypoint compute the corresponding world point
            A = [A_col1(:,i), -A_col2(:,i)]; % Matrix A of one keypoint
            x = (A' * A) \ (A' * b); % solve lambdas
            camera_points(:, i) = x(1) * A_col1(:, i); % lambda_left*K^-1*homo_keypoint_left_fliped;
        end
        
        fprintf('It took %ds to compute disparity_triangulation \n', toc);
        
        points_3D = camera_points;
        points_2D =  keypoints_left; % TODO should we send (u, v) coords instead of (row, col)?
    else
        printf('No triangulation method precised!!!');
        points_2D = 0;
        points_3D  = 0;
    end

end

