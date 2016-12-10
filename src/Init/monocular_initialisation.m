function [state, T_cw] = moncular_initialisation(img0, img1, K)

%   INPUT:
    %   - img0: First image of monocular camera.
    %   - img1: Subsequent image of monocular camera (not necessarily second image!).
    %   - K: 3x3 matrix with the intrinsics of the camera
    
%   OUTPUT:
    %   - points_2D: 2D points in the image (img0) that have been defined
    %   as keypoints and triangulated to 3D points in the world. 2xN vector
        %       * first row: each value corresponds to the row of the pixel in the
        %       image matrix.
        %       * second row: each value corresponds to the column of the pixel in the image
        %       matrix.
    %   - points_3D: 3D points in the world that correspond to each point in
    %   points_2D. These are expressed in the first camera frame (right handed frame
    %   with z pointing in front of the camera, x pointing to where the right camera is located
    %   see figure 2 exercise 4 for an illustration). 3xN vector
        %       * first row: x coordinate in first camera frame.
        %       * second row: y coordinate in first camera frame.
        %       * third row: z coordinate in first camera frame (should be>0).

    % Get keypoints in both frames, descriptors and matches
    [keypoints1, keypoints2, descriptors1, descritors2, matches] = ...
        correspondences_2d2d(img0, img1);

    % Get matching keypoints
    [~, query_indices, match_indices] = find(matches);
    kp1 = keypoints1(:, match_indices);
    kp2 = keypoints2(:, query_indices);

    % Homogenous coordinates
    %p1 = [kp1; ones(1, size(kp1, 2))];
    %p2 = [kp2; ones(1, size(kp2, 2))];

    p1 = [flipud(kp1) ; ones(1, size(kp1,2))]; % TODO not sure if this zeros should instead 
    p2 = [flipud(kp2) ; ones(1, size(kp2,2))]; %be a SCALE factor or something of the kind


    %p1 = [keypoints1; ones(1, size(keypoints1, 2))];
    %p2 = [keypoints2; ones(1, size(keypoints2, 2))];

    % Estimate fundamental matrix
    % Call the 8-point algorithm on inputs x1,x2
    F = fundamentalEightPoint_normalized(p1,p2)
    E = estimateEssentialMatrix(p1, p2, K, K)

    % Decompose the matrix E by svd
    [u,s,v]=svd(E);


    %
    w = [0 -1 0; 1 0 0; 0 0 1];
    z = [0 1 0; -1 0 0; 0 0 1];

    % 
    % E = SR where S = [t]_x and R is the rotation matrix.
    % E can be factorized as:
    %s = u * z * u';

    % Two possibilities:
    rot1 = u * w  * v';
    rot2 = u * w' * v';

    % Two possibilities:
    t1 = u(:,3) ./max(abs(u(:,3)));
    t2 = -u(:,3) ./max(abs(u(:,3)));


    % 4 possible choices of the camera matrix P2 based on the 2 possible
    % choices of R and 2 possible signs of t.
    rot(:,:,1) = rot1; 
    t(:,:,1) = t1;

    rot(:,:,2) = rot2; 
    t(:,:,2) = t2;

    rot(:,:,3) = rot1; 
    t(:,:,3) = t2;

    rot(:,:,4) = rot2; 
    t(:,:,4) = t1;
    %%
    % Check if in front of camera
    % lambda1*[u v 1]^T = K1 * [Xw Yw Zw]^T
    % lambda2*[u v 1]^T = K1 * R1* [Xw Yw Zw]^T + T
    for i=1:size(rot,3)
        M1 = K*[eye(3) zeros(3,1)];
        M2 = K*[rot(:,:,i) t(:,:,i)];

        P_test = linearTriangulation(p1(:,1), p2(:,1), M1, M2);

        px1 = M1*P_test;
        px2 = M2*P_test;

        % Check if point in front of both cameras
        if ((px1(3) > 0) && (px2(3) > 0))
            R = rot(:,:,i); 
            T = t(:,:,i);
            P = linearTriangulation(p1, p2, M1, M2);
            break
        end

        %x3d1(:,i) = inv(K)*p1(:,2);
        %kr = K*rot(:,:,i);
        %x3d2(:,i) = inv(kr)*(p2(:,2) - t(:,:,i));

    end

    repro = reprojectPoints(transpose(P(1:3,:)), M1, K);
    repro2 = reprojectPoints(transpose(P(1:3,:)), M2, K);
    %inlier_mask = ransac(img0, img1, K, keypoints1, P(1:3,:));


    % Check the epipolar constraint x2(i).' * F * x1(i) = 0 for all points i.
    N = size(p1,2);
    cost_algebraic = norm( sum(p2.*(F*p1)) ) / sqrt(N)
    cost_dist_epi_line = distPoint2EpipolarLine(F,p1,p2)
    
    
    %% OUTPUT
    state = []
    T_cw = []
end
