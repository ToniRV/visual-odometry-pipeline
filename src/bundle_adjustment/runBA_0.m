function hidden_state = runBA_0(hidden_state, observations, K, num_frames)

with_pattern = true;

if with_pattern
    %%% num_frames = observations(1);
    % Factor 2, one error for each x and y direction.
    %%% num_error_terms = 2 * (numel(observations)-2-num_frames)/3;
    
    % Define the number of error terms in the error function passed to
    % lsqnonlin:
    num_error_terms = 2 * (numel(observations)-num_frames)/3;
    
    % Each error term will depend on one pose (6 entries) and one landmark
    % position (3 entries):
    pattern = spalloc(num_error_terms, numel(hidden_state), ...
        num_error_terms * 9);
    
    % Define the (sparse) Jacobian pattern for each frame:
    observation_i = 1;
    error_i = 1;
    for frame_i = 1:num_frames
        num_keypoints_in_frame = observations(observation_i);
        % All errors of a frame are affected by its pose.
        pattern(error_i:error_i+2*num_keypoints_in_frame-1, ...
            (frame_i-1)*6+1:frame_i*6) = 1;
        
        % Each error is affected by the corresponding landmark.
        % Extract the indices of the corresponding landmarks:
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

% Define the error function passed to lsqnonlin:
error_terms = @(hidden_state) baError(hidden_state, observations, K, num_frames);
options = optimoptions(@lsqnonlin, 'Display', 'iter', ...
    'MaxIter', 40);
if with_pattern
    options.JacobPattern = pattern;
    options.UseParallel = false;
end

% Return the optimized poses and landmark positions:
hidden_state = lsqnonlin(error_terms, hidden_state, [], [], options);

end

 