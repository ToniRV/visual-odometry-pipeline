function keypoints = selectKeypoints(scores, num, r)
% Selects the pixels with the num best scores as keypoints and performs non-maximum 
% suppression of a (2r + 1)*(2r + 1) box around the current maximum.

keypoints = zeros(2, num);
temp_scores = padarray(scores, [r r]);

% Select the num points with the highest Harris score to avoid setting a
% static threshold.
for i = 1:num
    [~, kp] = max(temp_scores(:)); % Get linear index of the current maximum score
    [row, col] = ind2sub(size(temp_scores), kp); % Convert linear index to [row, col] coordinates
    kp = [row;col];
    keypoints(:, i) = kp - r;
    
    % Perform non-maximum suppression and remove current maximum score for next iteration
    temp_scores(kp(1)-r:kp(1)+r, kp(2)-r:kp(2)+r) = ...
        zeros(2*r + 1, 2*r + 1); 
end

end

