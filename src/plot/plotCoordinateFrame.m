function  [hline,hhead] = plotCoordinateFrame( rotation, origin, len, colors)
% PLOTCOORDINATEFRAME - plots a 3d coordinate frame.
%
% [hline,hhead] = plotCoordinateFrame( rotation, origin, len)
%
%  In the resulting plot x is red, y is green and z is blue.
%
% origin   - the origin of the frame (rho_0_j0)
% rotation - A rotation f -> i that rotates points from the coordinate
%            frame to the inertial frame. (C_0j)
% colors   - the colors of the three axis arrows.
%            If omitted, the default is ['r'; 'g'; 'b']
% Returns the handles to the lines and the heads of the arrows.
if nargin < 3
    len = 1;
end

if nargin < 4
    colors = ['r';'g';'b'];
end

if size(rotation,1) ~= 3 || size(rotation,2) ~= 3
    error('rotation must be a 3x3 matrix');
end

if size(origin,1) ~= 3 || size(origin,2) ~= 1
    error('origin must be a 3x1 vector');
end
R = rotation';

[hline,hhead] = arrow3d(repmat(origin',3,1), repmat(origin',3,1) + len*R,15);

set(hline(1,1),'facecolor',colors(1,:));
set(hhead(1,1),'facecolor',colors(1,:));
set(hline(1,2),'facecolor',colors(2,:));
set(hhead(1,2),'facecolor',colors(2,:));
set(hline(1,3),'facecolor',colors(3,:));
set(hhead(1,3),'facecolor',colors(3,:));


