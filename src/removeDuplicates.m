function unique_query_keypoints = removeDuplicates(...
    query_keypoints, current_keypoints, suppression_radius)
    % Remove current_keypoints from query_keypoints by clearing
    % query_keypoints close by suppression_radius pixels to other current_keypoints
    for k = 1:size(query_keypoints,2)
        x_lim = [query_keypoints(1,k)-suppression_radius, query_keypoints(1,k)+suppression_radius];
        y_lim = [query_keypoints(2,k)-suppression_radius, query_keypoints(2,k)+suppression_radius];
        inside_x_lim_mask = current_keypoints(1,:)>x_lim(1) &...
            current_keypoints(1,:)<x_lim(2);
        curr_kps_in_x_lim = current_keypoints(:,inside_x_lim_mask);
        if (size(curr_kps_in_x_lim, 2) ~=0)
            inside_y_lim_mask = curr_kps_in_x_lim(2,:)>y_lim(1) &...
                curr_kps_in_x_lim(2,:)<y_lim(2);
            curr_kps_in_x_lim_and_y_lim = curr_kps_in_x_lim(:,inside_y_lim_mask);
            if(size(curr_kps_in_x_lim_and_y_lim, 2) ~=0)
                query_keypoints(:,k) = [0,0];
            end
        end
    end
    unique_query_keypoints = query_keypoints(:, query_keypoints(1,:)~=0);
end

