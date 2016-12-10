function [rot, t] = get_pose_hypotheses(E)
    % Decompose the matrix E by svd
    [u,s,v]=svd(E);

    % E = SR where S = [t]_x and R is the rotation matrix.
    % E can be factorized as:
    % s = u * z * u';
    w = [0 -1 0; 1 0 0; 0 0 1];
    z = [0 1 0; -1 0 0; 0 0 1];
    
    % Two possibilities:
    rot1 = u * w  * v';
    rot2 = u * w' * v';
    % Two possibilities:
    t1 = u(:,3) ./max(abs(u(:,3)));
    t2 = -u(:,3) ./max(abs(u(:,3)));

    % 4 possible choices of the camera matrix P2 based on the 2 possible
    % choices of R and 2 possible signs of t.
    rot(:,:,1) = rot1; 
    t(:,:,1) = t1;

    rot(:,:,2) = rot2; 
    t(:,:,2) = t2;

    rot(:,:,3) = rot1; 
    t(:,:,3) = t2;

    rot(:,:,4) = rot2; 
    t(:,:,4) = t1;
end

