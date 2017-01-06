function plotMasked(inlier_mask, query_keypoints, database_keypoints)
    x_from = query_keypoints(1, inlier_mask);
    x_to = database_keypoints(1, inlier_mask);
    y_from = query_keypoints(2, inlier_mask);
    y_to = database_keypoints(2, inlier_mask);
    plot([y_from; y_to], [x_from; x_to], 'g-', 'Linewidth', 2);
end
