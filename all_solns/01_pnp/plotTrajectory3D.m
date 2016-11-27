function plotTrajectory3D(fps, transl,quats, pts3d)

% PLOTTRAJECTORY3D Given the (timestamped) poses from optitrack (time, translations, quaternions),
% draw the trajectory of the camera (3 colored axes, RGB).
%
%  -fps: framerate of the video
%  -transl(3,N): translations
%  -quats(4,N): orientations given by quaternions
%  -pts3d(3,N): additional 3D points to plot
%
% transl and quats refer to the tranformation T_W_C that maps points from
% the camera coordinate frame to the world frame, i.e. the transformation that expresses the
% camera position in the world frame.

decimationFactor = 1;
scaleFactorArrow = 0.05;
videoFilename = 'motion.avi';

numPoses = size(transl,2);

vidObj = VideoWriter(videoFilename,'Motion JPEG AVI'); % Prepare video file
vidObj.FrameRate = fps;
open(vidObj);

% Draw the trajectory
fig = figure('Color','w');

for k = 1:decimationFactor:numPoses
    pos = transl(:,k); % current position
    rotMat = quat2RotMatrix(quats(:,k)); % current orientation
    
    if (k==1)
        
        % Plot translation
        positionHandle = plot3(pos(1),pos(2),pos(3),'k.');
        
        hold on;
        axis equal;
        axis vis3d;
        grid on;
        xlabel('X');
        ylabel('Y');
        zlabel('Z');

%         % Uncomment this block to set a nice camera view for visualizing
%         % the camera motion in Exercise 2 - PnP
%
%         set(gca,'CameraPosition',...
%             [-2.11926427968706 -1.26339742475999 -2.74402484464086],'CameraUpVector',...
%             [0.249894865616035 -0.919346901850372 0.303897729831707],'CameraViewAngle',...
%             6.14709233376306,'DataAspectRatio',[1 1 1],'PlotBoxAspectRatio',...
%             [1.81087823370581 1 1.68269573663027]);
%         xlim([-0.3 0.2]);
%         ylim([-0.1 0.28]);
%         zlim([-0.6 0]);

        % Plot the 3D points
        scatter3(pts3d(1,:), pts3d(2,:), pts3d(3,:));
        
        % Plot orientation using axes (X=red, Y=green, Z=blue) at current location
        axisX = quiver3(pos(1),pos(2),pos(3), rotMat(1,1),rotMat(2,1),rotMat(3,1), 'r', 'ShowArrowHead', 'on', 'AutoScale', 'on', 'AutoScaleFactor', scaleFactorArrow);
        axisY = quiver3(pos(1),pos(2),pos(3), rotMat(1,2),rotMat(2,2),rotMat(3,2), 'g', 'ShowArrowHead', 'on', 'AutoScale', 'on', 'AutoScaleFactor', scaleFactorArrow);
        axisZ = quiver3(pos(1),pos(2),pos(3), rotMat(1,3),rotMat(2,3),rotMat(3,3), 'b', 'ShowArrowHead', 'on', 'AutoScale', 'on', 'AutoScaleFactor', scaleFactorArrow);

    end
    
    % Draw only the current moving frame, not the past ones

    % Plot translation
    set(positionHandle, 'xdata', pos(1), 'ydata', pos(2), 'zdata', pos(3));

    % Plot orientation using axes (X=red, Y=green, Z=blue) at current location
    set(axisX, 'xdata', pos(1), 'ydata', pos(2), 'zdata', pos(3), 'udata', rotMat(1,1), 'vdata', rotMat(2,1), 'wdata', rotMat(3,1));
    set(axisY, 'xdata', pos(1), 'ydata', pos(2), 'zdata', pos(3), 'udata', rotMat(1,2), 'vdata', rotMat(2,2), 'wdata', rotMat(3,2));
    set(axisZ, 'xdata', pos(1), 'ydata', pos(2), 'zdata', pos(3), 'udata', rotMat(1,3), 'vdata', rotMat(2,3), 'wdata', rotMat(3,3));

    drawnow;
    
    pause(0.005); % seconds
    writeVideo(vidObj, getframe(fig)); % Write each frame to the file
    
end

close(vidObj); % Close file; movie is written to disk

end