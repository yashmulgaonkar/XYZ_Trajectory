function plots(traj, threeD, states, time)
% This script written by Justin Thomas to analyze trajectories in dynamic
% simulations as well as actual trajectories.  It requires EulToRot() and
% load_params().

% If states are provided, then we have simulated or actual stuff to plot
if nargin > 2
    sim = true;
else
    sim = false;
end

params = load_params();

scrsz = get(0,'ScreenSize');
scrsz = scrsz + 100*[1 1 -2 -2];

%% Positions
figure
hold on
if sim
    % Determine a stepsize that will give us 100 datapoints on our plots
    step = round(length(time)/75);
    % Plot actual positions
    plot(time(1:step:end),states(1:step:end,1),'.r',time(1:step:end),states(1:step:end,2),'.g',time(1:step:end),states(1:step:end,3),'.b');
end

% Plot the desireds
plot(traj.time,traj.pos(:,1),'-.r');
plot(traj.time,traj.pos(:,2),'-.g');
plot(traj.time,traj.pos(:,3),'-.b');

if sim
    % Determine the closest achieved point to the pickup point
    diffs = bsxfun(@minus,traj.pickup(1:3), states(:,1:3));
    errors = sqrt(sum(diffs.^2,2));
    [C, I] = min(errors); %#ok<ASGLU>
    disp(['Error at pickup: ', num2str(errors(I)), ' meters']);
    
    % Plot a vertical line at the pickup point
    yL = get(gca,'YLim');
    plot([time(I) time(I)], yL, '-.k');
end

% Set plot parameters
set(gcf,'position',scrsz);
title('Position')
xlabel('time (s)')
ylabel('Position (m)')
legend('x','y','z');
axis tight
hold off
pause
close

%% Roll, Pitch, and Yaw

if sim
    figure
    hold on
    
    % Plot actuals
    plot(time(1:step:end),states(1:step:end,7),'.r');
    plot(time(1:step:end),states(1:step:end,8),'.g');
    plot(time(1:step:end),states(1:step:end,9),'.b');
    
    % Plot constraints
    plot([time(1) time(end)],[params.maxAngle params.maxAngle],'-.k');
    plot([time(1) time(end)],[-params.maxAngle -params.maxAngle],'-.k');
    
    % Plot desireds
    plot(traj.time,traj.phi,'-.r')
    plot(traj.time,traj.theta,'-.g')
    plot(traj.time,traj.psi,'-.b')
    
    % Set plot parameters
    set(gcf,'position',scrsz);
    xlabel('time (s)')
    ylabel('Angle (rad)')
    title('phi, theta, psi')
    axis tight
    
    % Plot a vertical line at the pickup point
    yL = get(gca,'YLim');
    plot([time(I) time(I)], yL, '-.k');
    
    hold off
    pause
    close
end

%% 3D plot of actual, desired, and angles

if threeD
    
    figure
    hold on
    if sim
        plot3(states(:,1),states(:,2),states(:,3));
    end
    title(['x, y, z position;  Total Time: ', num2str(traj.time(end)), ' seconds']);
    xlabel('x')
    ylabel('y')
    zlabel('z')
    
    view(-161,16);
    
    %% Overlay orientation
    
    if sim
        
        ctimes = 0:time(end)/20:time(end);
        
        % x, y, z as rows so that we can rotate them
        a = params.armlength;
        square = [...
            a  0 -a  0;
            0 -a  0  a;
            0  0  0  0];
        
        % Define our colormap
        cmap = colormap(jet);
        
        pts = zeros(4,3,length(ctimes));
        attitude_functional = zeros(length(ctimes),1);
        for idx = 1:length(ctimes)
            keyidx = find(time >= ctimes(idx),1,'first');
            angles = states(keyidx,7:9);
            
            Rot = EulToRot(angles(1),angles(2),angles(3));
            Rotlevel = EulToRot(0, 0, angles(3));
            
            % Columns of x, y, and z
            vertices = (Rot*square)';
            
            % Now we need to shift to the trajectory
            shift = ones(4,1)*[states(keyidx,1) states(keyidx,2) states(keyidx,3)];
            
            % Store the points
            pts(:,:,idx) = vertices + shift;
            attitude_functional(idx) = 1/2*trace(eye(3) - Rotlevel'*Rot);
        end

        % Now plot the patches representing the quadrotor
        for idx = 1:length(ctimes)
            cindex = max(1,round(attitude_functional(idx)/max(attitude_functional)*64));
            patch(pts(:,1,idx),pts(:,2,idx),pts(:,3,idx),cmap(cindex,:));
        end
    end
    
    % Plot the desired trajectory
    plot3(traj.pos(:,1),traj.pos(:,2),traj.pos(:,3),'-.k');
    
    % Set plot parameters
    set(gcf,'position',scrsz);
    axis equal
    grid on
    rotate3d on
    pause
    close
end

% Use this if we want to save images as pngs by default
% print(fig,'-dpng','image.png');

end