function [keypoints_1, inlier_mask] = KLT(keypoints_0, img_1, img_0)
    % KLT DEMO Create the point tracker object. 
    pointTracker = vision.PointTracker('MaxBidirectionalError', 0.5);
    POINTS = flipud(keypoints_0)';
    initialize(pointTracker, POINTS, img_0);
    [NEW_POINTS, POINT_VALIDITY, SCORES] = step(pointTracker, img_1);
    % A more quantitative value for the quality of the tracking.
    SCORES = SCORES';
    % This guy gives us the needed threshold to determine if it is a valid
    % tracking or not
    POINT_VALIDITY = POINT_VALIDITY';
    keypoints_1 = flipud(NEW_POINTS');
    inlier_mask = POINT_VALIDITY > 0;
    
    figure(7);
    % Display tracked points.
    imshow(img_0);
    hold on;
    plot (keypoints_1(2, inlier_mask), keypoints_1(1, inlier_mask), 'yx');
    plot (keypoints_0(2, inlier_mask), keypoints_0(1, inlier_mask), 'gx');
    plotMatches(true(1, size(keypoints_1, 2)), keypoints_1, keypoints_0);
    hold off;
end