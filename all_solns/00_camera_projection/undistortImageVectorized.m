function undimg = undistortImageVectorized(img, K, D)
  [X, Y] = meshgrid(1:size(img, 2), 1:size(img, 1));
  px_locs = [X(:)-1, Y(:)-1, ones(nnz(X), 1)]';
  
  normalized_px_locs = K^-1 * px_locs;
  normalized_px_locs = normalized_px_locs(1:2, :);
  normalized_dist_px_locs = distortPoints(normalized_px_locs, D);
  dist_px_locs = K * [normalized_dist_px_locs; ...
      ones(1, size(normalized_dist_px_locs, 2))];
  dist_px_locs = dist_px_locs(1:2, :);
  
  intensity_vals = img(round(dist_px_locs(2, :)) + ...
      size(img, 1) * round(dist_px_locs(1, :)));
  undimg = uint8(reshape(intensity_vals, size(img)));
end
