function error = alignError(x, p_W_GT, p_W_estimate)

T_old_new = twist2HomogMatrix(x(1:6));
scale_old_new = x(7);

num_frames = size(p_W_estimate, 2);
aligned = scale_old_new * T_old_new(1:3, 1:3) * p_W_estimate ...
    + repmat(T_old_new(1:3, 4), [1 num_frames]);

errors = p_W_GT - aligned;

% Vectorize output error:
error = errors(:);

end

