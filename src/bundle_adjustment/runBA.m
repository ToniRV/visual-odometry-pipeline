function hidden_state = runBA(hidden_state, observations, K)

with_pattern = true;

if with_pattern
    num_frames = observations(1);
    % Factor 2, one error for each x and y direction.
    num_error_terms = 2 * (numel(observations)-2-num_frames)/3;
    % Each error term will depend on one pose (6 entries) and one landmark
    % position (3 entries):
    pattern = spalloc(num_error_terms, numel(hidden_state), ...
        num_error_terms * 9);
    % Pattern for each frame.
    observation_i = 3;
    error_i = 1;
    for frame_i = 1:num_frames
        num_keypoints_in_frame = observations(observation_i);
        % All errors of a frame are affected by its pose.
        pattern(error_i:error_i+2*num_keypoints_in_frame-1, ...
            (frame_i-1)*6+1:frame_i*6) = 1;
        
        % Each error is affected by the corresponding landmark.
        landmark_indices = observations(...
            observation_i+2*num_keypoints_in_frame+1:...
            observation_i+3*num_keypoints_in_frame);
        for kp_i = 1:numel(landmark_indices)
            pattern(error_i+(kp_i-1)*2:error_i+kp_i*2-1,...
                1+num_frames*6+(landmark_indices(kp_i)-1)*3:...
                num_frames*6+landmark_indices(kp_i)*3) = 1;
        end
        
        observation_i = observation_i + 1 + 3*num_keypoints_in_frame;
        error_i = error_i + 2 * num_keypoints_in_frame;
    end
    figure(4);
    spy(pattern);
end

error_terms = @(hidden_state) baError(hidden_state, observations, K);
options = optimoptions(@lsqnonlin, 'Display', 'iter', ...
    'MaxIter', 20);
if with_pattern
    options.JacobPattern = pattern;
    options.UseParallel = false;
end
hidden_state = lsqnonlin(error_terms, hidden_state, [], [], options);

end

 