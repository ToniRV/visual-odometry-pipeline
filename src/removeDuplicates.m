function unique_query_keypoints = removeDuplicates(query_keypoints, current_keypoints)
    % Remove current_keypoints from query_keypoints by clearing
    % query_keypoints close by r pixels to other current_keypoints
    r = 4;
    for k = 1:size(query_keypoints,2)
        x_lim = [query_keypoints(1,k)-r, query_keypoints(1,k)+r];
        y_lim = [query_keypoints(2,k)-r, query_keypoints(2,k)+r];
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

