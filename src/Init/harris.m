function scores = harris(img, patch_size, kappa)

sobel_para = [-1 0 1];
sobel_orth = [1 2 1];

Ix = conv2(sobel_orth', sobel_para, img, 'valid');
Iy = conv2(sobel_para', sobel_orth, img, 'valid');
Ixx = double(Ix .^ 2);
Iyy = double(Iy .^ 2);
Ixy = double(Ix .* Iy);

patch = ones(patch_size, patch_size) / (patch_size ^ 2);
pr = floor(patch_size / 2);  % patch radius
sIxx = conv2(Ixx, patch, 'valid');
sIyy = conv2(Iyy, patch, 'valid');
sIxy = conv2(Ixy, patch, 'valid');

scores = (sIxx .* sIyy - sIxy .^ 2) ... determinant
    - kappa * (sIxx + sIyy) .^ 2;  % square trace

scores(scores<0) = 0;

scores = padarray(scores, [1+pr 1+pr]);

end
