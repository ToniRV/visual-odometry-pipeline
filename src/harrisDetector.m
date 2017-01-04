function query_keypoints = harrisDetector (query_image, current_keypoints)
    debug_with_figures = false;

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
    query_keypoints = ones(2, cols*rows*num_keypoints); % I put ones instead of zeros just in case there is no match....
   
    scores = harris(query_image, harris_patch_size, harris_kappa);
    lean_scores = leanerScores(scores, current_keypoints, nonmaximum_supression_radius);
    
    if (debug_with_figures)
        figure(12);
        subplot(2, 2, [1 2]);
        imshow(query_image);
        hold on;
        plot(current_keypoints(2, :), current_keypoints(1, :), 'gx', 'Linewidth', 2);
        legend('Current Keypoints')
        hold off;
        subplot(2, 2, 3);
        imagesc(scores);
        subplot(2, 2, 4);
        imagesc(lean_scores);
    end
    
    k = 0;
    for w = 1:cols
        for h = 1:rows
         selected_kps = selectKeypoints(lean_scores(h_indices_limits(h):h_indices_limits(h+1),...
                                                                                w_indices_limits(w):w_indices_limits(w+1)), ...
                                                            num_keypoints, nonmaximum_supression_radius);
         query_keypoints(:, k*(num_keypoints)+1:(k+1)*(num_keypoints)) = ...
             [selected_kps(1,:)+h_indices_limits(h); selected_kps(2,:)+w_indices_limits(w)];
         k = k+1;
        end
    end

end

function new_scores = leanerScores (scores, tracked_keypoints, r)
    temp_scores = padarray(scores, [r r]);
    tracked_keypoints = ceil(tracked_keypoints)+r; % NOT sure whether to use floor or round, I
    % pick ceil to make sure I don't run into non-positive indices
    % afterwards..
    % Remove already followed keypoints
    for i = 1:size(tracked_keypoints, 2)
        temp_scores(tracked_keypoints(1,i)-r:tracked_keypoints(1,i)+r,...
                              tracked_keypoints(2,i)-r:tracked_keypoints(2,i)+r)...
                              = zeros(2*r+1, 2*r+1);
    end
    new_scores = temp_scores(r+1:end-r, r+1:end-r);
end

