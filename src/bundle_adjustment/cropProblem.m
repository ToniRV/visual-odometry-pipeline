function [cropped_hidden_state, cropped_observations, ...
    cropped_ground_truth] = cropProblem(...
    hidden_state, observations, ground_truth, cropped_num_frames)

% Determine which landmarks to keep; assuming landmark indices increase
% with frame indices.
num_frames = observations(1);
assert(cropped_num_frames < num_frames);

observation_i = 3;
for i = 1:cropped_num_frames
    num_observations = observations(observation_i);
    if i == cropped_num_frames
        cropped_num_landmarks = max(observations(...
            observation_i+1+num_observations*2:...
            observation_i+num_observations*3));
    end
    observation_i = observation_i+num_observations*3+1;
end

cropped_hidden_state = [hidden_state(1:6*cropped_num_frames); ...
    hidden_state(6*num_frames+1:6*num_frames+3*cropped_num_landmarks)];
cropped_observations = [cropped_num_frames; cropped_num_landmarks; ...
    observations(3:observation_i-1)];

cropped_ground_truth = ground_truth(:, 1:cropped_num_frames);