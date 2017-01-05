function descriptors = describeKeypoints(img, keypoints, r)
% Returns a (2r+1)^2xN matrix of image patch vectors based on image
% img and a 2xN matrix containing the keypoint coordinates.
% r is the patch "radius".

N = size(keypoints, 2);
descriptors = uint8(zeros((2*r+1) ^ 2, N));
padded = padarray(img, [r, r]);
for i = 1:N
    kp = keypoints(:, i) + r;
    descriptors(:,i) = reshape(...
        padded(kp(1)-r:kp(1)+r, kp(2)-r:kp(2)+r), [], 1);
end

end
