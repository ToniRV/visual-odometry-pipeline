function [point_3d] = triangulate(kp1, kp2, M1, M2)

    A(:,1) = kp1(1)*M1(:,3) - M1(:,1);
    A(:,2) = kp1(2)*M1(:,3) - M1(:,2);
    A(:,3) = kp2(1)*M2(:,3) - M2(:,1);
    A(:,4) = kp2(2)*M2(:,3) - M2(:,2);

    [w, u, vt] = svd(A);
    point_3d_homo = vt(4,:)';
    point_3d_homo = point_3d_homo(1:4)/point_3d_homo(4); 
    %point_3d = point_3d_homo(1:3);
    point_3d = point_3d_homo;

end

