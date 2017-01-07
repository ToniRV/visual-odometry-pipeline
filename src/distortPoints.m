function x_d = distortPoints(x, D)
% Applies lens distortion D(2x1) to 2D points x(2xN) on the image plane.

k1 = D(1); k2 = D(2);

xp = x(1,:); yp = x(2,:);

r2 = xp.^2 + yp.^2;
xpp = xp .* (1 + k1*r2 + k2*r2.^2);
ypp = yp .* (1 + k1*r2 + k2*r2.^2);

x_d = [xpp; ypp];

end

