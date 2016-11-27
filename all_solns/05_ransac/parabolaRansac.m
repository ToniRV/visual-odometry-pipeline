function [best_guess_history, max_num_inliers_history] = ...
    parabolaRansac(data, max_noise)
% data is 2xN with the data points given column-wise, 
% best_guess_history is 3xnum_iterations with the polynome coefficients 
%   from polyfit of the BEST GUESS at each iteration columnwise and
% max_num_inliers_history is 1xnum_iterations, with the inlier count of the
%   BEST GUESS at each iteration.

% rng(2); For our figure.
num_iterations = 100;

best_guess_history = zeros(3, num_iterations);
max_num_inliers_history = zeros(1, num_iterations);

best_guess = zeros(3, 1);
max_num_inliers = 0;

rerun_on_inliers = true;

for i = 1:num_iterations
    samples = datasample(data, 3, 2, 'Replace', false);
    guess = polyfit(samples(1, :), samples(2, :), 2);
    errors = abs(polyval(guess, data(1, :)) - data(2, :));
    inliers = errors <= max_noise + 1e-5;
    num_inliers = nnz(inliers);
    if num_inliers > max_num_inliers
        if rerun_on_inliers
            guess = polyfit(data(1, inliers), data(2, inliers), 2);
        end
        best_guess = guess';
        max_num_inliers = num_inliers;
    end
    best_guess_history(:, i) = best_guess;
    max_num_inliers_history(i) = max_num_inliers;
end

end

