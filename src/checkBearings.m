function checkBearings
last_kp = [0;0];
last_tf = [1, 0, 0, 0;
                0, 1, 0, 0;
                0, 0, 1, -10];
first_kp = [100000; 0];
first_tf = [1, 0, 0, ...
                0, 1, 0, ...
                0, 0, 1, ...
                0, 0, 0]';
            
K = [10, 0, 0;
        0, 10, 0;
        0, 0, 1];
checkTriangulability(last_kp, last_tf, first_kp, first_tf, K);


end






function is_triangulable = checkTriangulability(last_kps, last_tf, first_kps, first_tfs, K)
%%% last_tf: transformation from World to Camera of the last kps
%%% first_tf: transformation from World to Camera of the first kps
    %%% Tune this threshold
    threshold = 20;
    %1) Compute last bearing vector in the World frame
    bearing_vector_last_kps = computeBearing(last_kps, last_tf, K);
    %2) Compute first bearing vector in the World frame
    bearing_vector_first_kps = zeros(3, size(first_tfs, 2));
    for i = 1:size(first_kps,2)
        first_tf = reshape(first_tfs(:, i), 3, 4);
        bearing_vector_first_kps(:, i) = computeBearing(first_kps(:, i), first_tf, K);
    end
    %3) Check which current kps are triangulable
    angles = atan2d(norm(cross(bearing_vector_last_kps, bearing_vector_first_kps)), ...
        dot(bearing_vector_last_kps, bearing_vector_first_kps));
    is_triangulable = angles > threshold;
end

function bearing_vector = computeBearing(kps, tfs, K)
    % Get bearings orientation in cam frame assumig lambda equal to 1
    bearings = K\[kps; ones(1, size(kps,2))];
    % Get rot matrix from cam points to world
    R_C_W = tfs(:, 1:3);
    % Get bearings orientation in world frame
    bearings_in_world_frame = R_C_W*bearings;
    bearing_vector = bearings_in_world_frame;
end