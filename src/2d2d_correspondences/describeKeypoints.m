function descriptors = describeKeypoints(img, keypoints, r)
% Returns a (2r+1)^2xK matrix of image patch vectors based on image
% img and a 2xK matrix containing the keypoint coordinates.
% r is the patch "radius".

K = size(keypoints, 2);
descriptors = uint8(zeros((2*r+1) ^ 2, K));
padded = padarray(img, [r, r]);
for i = 1:K
    kp = keypoints(:, i) + r;
    descriptors(:,i) = reshape(...
        padded(kp(1)-r:kp(1)+r, kp(2)-r:kp(2)+r), [], 1); 
        % Note: [] automatically calculates appropriate number of rows
end

end
