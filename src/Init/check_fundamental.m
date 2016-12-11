function [score, inliers] = check_fundamental(kp_database, kp_query, ...
    F, num_matches, sigma)

    th = 1.0;%3.841;
    th_score = 5.991;
    inv_sigma_squared = 1.0/(sigma^2);
    score = 0.0;

    for i=1:num_matches
        bool_inlier = true;
        
        kp1 = kp_database(:,i);
        kp2 = kp_query(:,i);
        
        % Reprojection error in second image
        % l2 = F * x1
        l2 = F*kp1;
        num2 = dot(l2, kp2);
        square_dist1 = num2^2/(l2(1)^2+l2(2)^2);
        chi_square1 = square_dist1*inv_sigma_squared;
        
        if (chi_square1 > th)
           bool_inlier = false; 
        else
            score = score + th_score - chi_square1;
        end
        
        % Reprojection error in second image
        % l2 = F * x1
        l1 = kp2'*F;
        num1 = dot(l1, kp1);
        square_dist2 = num1^2/(l1(1)^2+l1(2)^2);
        chi_square2 = square_dist2*inv_sigma_squared;
        
        if (chi_square2 > th)
           bool_inlier = false; 
        else
            score = score + th_score - chi_square2;
        end
        
        if (bool_inlier)
            inliers(i) = true;
        else
            inliers(i) = false;
        end
    end
end

