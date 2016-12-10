% NORMALISE2DPTS - normalises 2D homogeneous points
%
% Function translates and normalises a set of 2D homogeneous points 
% so that their centroid is at the origin and their mean distance from 
% the origin is sqrt(2).
%
% Usage:   [newpts, T] = normalise2dpts(pts)
%
% Argument:
%   pts -  3xN array of 2D homogeneous coordinates
%
% Returns:
%   newpts -  3xN array of transformed 2D homogeneous coordinates.
%   T      -  The 3x3 transformation matrix, newpts = T*pts
%


function [newpts, T] = normalise2dpts(pts)

    num_points = length(pts);

    mu = mean(pts(1:2,:),2); % centroid
    sigma = mean(sqrt(sum((pts(1:2,:)-repmat(mu,1,num_points)).^2))); % mean standard deviation
    s = sqrt(2) / sigma;
    
    T = [[s 0 -s*mu(1)];
         [0 s -s*mu(2)];
         [0 0 1]];
    
    newpts = T*pts;

end
    
    
    