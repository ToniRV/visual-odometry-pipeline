function plotMap(hidden_state, range, num_frames)

T_W_frames = reshape(hidden_state(1:num_frames*6), 6, []);
p_W_landmarks = reshape(hidden_state(num_frames*6+1:end), 3, []);

p_W_frames = zeros(3, num_frames);

for i = 1:num_frames
   % Need current (homogeneous) transformation only temporarily to
   % calculate current p_W_estimate(:, i):
   T_W_frame = twist2HomogMatrix(T_W_frames(:, i));
   p_W_frames(:,i) = T_W_frame(1:3,4);
end

plot(p_W_landmarks(3, :), -p_W_landmarks(1, :), '.c');
hold on;
plot(p_W_frames(3, :), -p_W_frames(1, :), '-r', 'Linewidth', 3);
hold off;
axis equal;
axis(range);

end

