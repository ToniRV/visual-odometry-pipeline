function query_keypoints = harrisDetector (query_image)
    % Parameters form exercise 3.
    harris_patch_size = 9;
    harris_kappa = 0.08;
    nonmaximum_supression_radius = 8;
    
    % Divide image in matrix
    cols = 4;
    rows = 3;
    
    width = size(query_image, 2);
    height = size(query_image, 1);
    
    w_indices_limits = ones(1,cols+1);
    for i=1:cols
        w_indices_limits(i+1) = floor(width/cols*i);
    end
    h_indices_limits = ones(1,rows+1);
    for i=1:rows
        h_indices_limits(i+1) = floor(height/rows*i);
    end
    
    % TRY to not take keypoints close to the borders of the image, to do so
    % just crop the image...
    
    
    % Other parameters.
    num_keypoints = 16;
    
    % Detect new keypoints
    k = 0;
    query_keypoints = zeros(2, cols*rows*num_keypoints);
    for w = 1:cols
        for h = 1:rows
         query_harris = harris(query_image(h_indices_limits(h):h_indices_limits(h+1),...
                                                                w_indices_limits(w):w_indices_limits(w+1)), ...
                                                                harris_patch_size, harris_kappa);
          selected_kps = selectKeypoints(...
             query_harris, num_keypoints, nonmaximum_supression_radius);
         query_keypoints(:, k*(num_keypoints)+1:(k+1)*(num_keypoints)) = ...
             [selected_kps(1,:)+h_indices_limits(h); selected_kps(2,:)+w_indices_limits(w)];
         k = k+1;
        end
    end

end
