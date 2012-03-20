function traj = QuadProg_Trajectory_Generator(saferegion, start, pickup, finish, speed)
% This function is used to determine an appropriate trajectory for a
% grasping quadrotor

% ColorSet = distinguishable_colors(iterations);

%% Parameters and Thresholds

% Set the desired thresholds here:
velocity_threshold = 2*speed;
acc_threshold = 5;
maxPsidot = pi/4;

% Parameters
tstep = .005;
params = load_params();

% s is our state vector in our differentially flat space for each keyframe.
keyframes = 3;
keytimes = zeros(1,keyframes);

%% States

s = zeros(4,3);
s(1:4,:) = [start' pickup' finish'];

sdot = zeros(4,3);   % xdot, ydot, zdot, psidot
% sddot = zeros(4,3);  % ddots

% A first guess at keytimes.  Obviously, they are too fast.
keytimes(1) = 0;
keytimes(2) = .1;
keytimes(3) = .2;

% Let's use the pickup velocity and orientation to determine our boundary
% conditions
attackangle = atan2(finish(2)-start(2), finish(1)-start(1));
sdot(1,2) = speed*cos(attackangle); % xdot at pickup
sdot(2,2) = speed*sin(attackangle); % ydot at pickup

%% Now we define the Optimization problem

% Polynomial order
n = 13;

% Initalize the order in the basis generator
basisgen(0,n);

% Initalize the basis matrix
basis = zeros(n+1,keyframes);

% Trajectory segments
m = keyframes - 1;

%% Generate our linear differential operator
LinDeriv = zeros(n + 1);
for idx = 1:n
    LinDeriv(idx,:) = [zeros(1,idx) (n - idx + 1) zeros(1,n-idx)];
end
D = cell(4,1);
D{1} = LinDeriv;
D{2} = LinDeriv^2;
D{3} = LinDeriv*D{2};
D{4} = LinDeriv*D{3};

% coeffs has the coefficients along the rows, segments along columns, and
% flat output in the 3rd dimension.
coeffs = cell(4, m);
fo = cell(4,1);
D1fo = fo;
D2fo = fo;
D3fo = fo;
D4fo = fo;

% Segment Loop
for seg = 1:m
    
    % Optimization loop
    while (1)
        
        times = 0:tstep:(keytimes(seg+1) - keytimes(seg) - tstep);
        
        % Our first point on the segment
        t1 = keytimes(seg);
        basis(:,seg) = basisgen(0);
        
        % The second point on the segment
        t2 = keytimes(seg + 1);
        basis(:,seg+1) = basisgen(t2-t1);

        % Quadratic Optimization for each element for the current segment in our flat space
        for flat_out = 1:4
            
            % Determine H based on our two times.  Note: we have to add a
            % slight offset to ensure that H is positive semidefinite.
            if flat_out < 4
                numderiv = 4;
            else
                numderiv = 2;
            end
            
            d_quad_prime = D{numderiv}*basisgen(t2-t1);
            problem.H = d_quad_prime*d_quad_prime' + (1/10^4)*eye(n+1);
            
            diag_incr_count = 0;
            while min(eig(problem.H)) < 0
%                 disp('Increasing diagional of H to ensure convexity.');
                problem.H = problem.H + (1/10^4)*eye(n+1);
                diag_incr_count = diag_incr_count + 1;
            end
            
            if diag_incr_count > 2
                warning(['The diagional was increased ', num2str(diag_incr_count), ' times']); %#ok<WNTAG>
            end
            
            problem.options = optimset('MaxIter',1000,'Display','off');
            problem.solver = 'quadprog';
            
            % I need to make sure z doesn't hit the ground.  Add it as a constraint
            % with A and b.
            
            % A*x <= b
            problem.Aineq = [];
            problem.bineq = [];
            
%             % Establish our safe region constraints
%             if flat_out < 4
%                 t = 0:5*tstep:t2-t1;
%                 Aineq = -basisgen(t')';
%                 bineq = -saferegion(flat_out)*ones(length(t),1);
%                 
%                 Aineq = [Aineq; basisgen(t')'];
%                 bineq = [bineq; saferegion(flat_out+3)*ones(length(t),1)];
%                 
%                 problem.Aineq = Aineq;
%                 problem.bineq = bineq;
%                 
%                 problem.options = optimset('Algorithm', 'active-set','MaxIter',1000);
%                 
%             end
            
            % Setup our boundary conditions based on the coefficients and
            % our basis
            problem.Aeq = [...
                basis(:,seg)';              % Initial position
                basis(:,seg+1)';            % Final Position
                (D{1}*basis(:,seg))';       % Initial velocity
                (D{1}*basis(:,seg+1))';     % Final velocity
                (D{2}*basis(:,seg))';       % Initial Acceleration
                (D{2}*basis(:,seg+1))'];     % Final Acceleration
            
            % Here we actually specify the constraint
            problem.beq = [...
                s(flat_out,seg);        % Initial Position
                s(flat_out,seg+1);      % Final Position
                sdot(flat_out,seg);     % Initial Velocity
                sdot(flat_out,seg+1);   % Final Velocity
                zeros(2,1);];
            
            % min 0.5*x'*H*x + f'*x   subject to:  A*x <= b and Aeq*x = beq
            coeffs{flat_out,seg} = quadprog(problem);
            
            fo{flat_out,seg} = polyval(coeffs{flat_out,seg},times);
            D1fo{flat_out,seg} = polyval(polyder(coeffs{flat_out,seg}),times);
            D2fo{flat_out,seg} = polyval(polyder(polyder(coeffs{flat_out,seg})),times);
            D3fo{flat_out,seg} = polyval(polyder(polyder(polyder(coeffs{flat_out,seg}))),times);
            D4fo{flat_out,seg} = polyval(polyder(polyder(polyder(polyder(coeffs{flat_out,seg})))),times);
            
        end
        
        % Use Differential Flatness to determine the states required to achieve
        % the trajectory and then increase the time until we have a feasible
        % trajectory.
        
        xdd = D2fo{1,seg}';
        ydd = D2fo{2,seg}';
        zdd = D2fo{3,seg}';
        
        % Each row of zbody is a unit vector at each timestep
        % [xworld, yworld, zworld]
        zb = [xdd, ydd, zdd + params.g];
        norm_zb = rownorm(zb);
        
        % Determine the feedforward control for the thrust
        traj.u1{seg} = norm_zb*params.m;

        % Normalize the zb vector
        % #### Can I use unit?
        zb_unit = zb ./ (rownorm(zb)*ones(1,3));
        
        % Store psi
        traj.psi{seg} = fo{4,seg}';
        %
        %         % Determine xc, the x vector of the frame that accounts only for
        %         % yaw angle.
        %         xc = [cos(traj.psi{key}), sin(traj.psi{key}), zeros(length(traj.psi{key}),1)];
        %
        %         % Determine the unit vectors representing the body frame y axis in
        %         % the world coordinates
        %         xb_cross_xc = cross(zb_unit, xc);
        %         yb = xb_cross_xc ./ (rownorm(xb_cross_xc)*ones(1,3));
        %
        %         % And the body frame x axis in world coordinates
        %         xb = cross(yb, zb);
        
        % We can determine the euler angles from the Z - X - Y
        % representation.
        traj.phi{seg} = asin(-zb_unit(:,2));
        traj.theta{seg} = atan2(zb_unit(:,1),zb_unit(:,3));
        %
        %     if max(abs(traj.theta)) > thresholds.theta || max(traj.u1) > thresholds.F || max(traj.u2) > thresholds.M
        % %         warning('The current trajectory is exceeding the thresholds.');
        %     end
        
        %% Check if our trajectory is too aggressive
        
        t = keytimes(seg+1)-keytimes(seg);
        
        checkthresh = [...
            max(abs(traj.phi{seg})) > params.maxAngle ...
            max(abs(traj.theta{seg})) > params.maxAngle ...
            max(traj.u1{seg}) > .5*params.maxForce ...
            max(rownorm([D1fo{1,seg}' D1fo{2,seg}' D1fo{3,seg}'])) > velocity_threshold ...
            max(rownorm([D2fo{1,seg}' D2fo{2,seg}' D2fo{3,seg}'])) > acc_threshold ...
            max(D1fo{4,seg}) > maxPsidot];
        %             max(u2{key}) > maxMoment ...
        %             max(u3{key}) > maxMoment ...
        %             max(u4{key}) > params.maxPsiMom ...
        
        if  any(checkthresh)
            lastcheckthresh = checkthresh;
            
            % Shift all successive keytimes appropriately
            keytimes(seg + 1:end) = keytimes(seg + 1:end) + .3;
            
        else
            factors = {'Phi', 'Theta', 'Thrust', 'Velocity', 'Acceleration', 'Psidot'};
            % factors = {'Phi', 'Theta', 'Thrust', 'M1', 'M2', 'Velocity', 'Acceleration', 'Psi Moment', 'Psidot'};
            disp(['Segment ', num2str(seg), ' Duration: ', num2str(t), ' seconds']);
            disp(['Limiting Factor(s):', factors(lastcheckthresh)]);
            
            break;
        end
        
        if t > 5
            disp('The trajectory segment is longer than 5 seconds.  You may want to just quit now and change your parameters.');
            keyboard
        end
    end
end

%% Save the trajectory
traj.tstep = tstep;
traj.keytimes = keytimes;

traj.x = [fo{1,:}]';
traj.y = [fo{2,:}]';
traj.z = [fo{3,:}]';

traj.time = (0:tstep:length(traj.x)*tstep - tstep)';

traj.phi = cell2mat(traj.phi');
traj.theta = cell2mat(traj.theta');
traj.psi = cell2mat(traj.psi');

traj.start = start;
traj.pickup = pickup;
traj.finish = finish;

% traj.p = []';
% traj.q = []';
% traj.r = []';

% traj.u = [[u1{:}]' [u2{:}]' [u3{:}]' [u4{:}]'];
traj.u1 = cell2mat(traj.u1');
% traj.u4 = I(3,3)*

traj.pos = [traj.x traj.y traj.z];
traj.vel = [[D1fo{1,:}]' [D1fo{2,:}]' [D1fo{3,:}]'];
traj.acc = [[D2fo{1,:}]' [D2fo{2,:}]' [D2fo{3,:}]'];

traj.coeffs.x = [coeffs{1,:}];
traj.coeffs.y = [coeffs{2,:}];
traj.coeffs.z = [coeffs{3,:}];
traj.coeffs.psi = [coeffs{4,:}];

save('traj','traj');

disp(['Total Time: ', num2str(keytimes(end)), ' seconds'])

if any(any(traj.pos < ones(length(traj.pos),1)*saferegion(1:3))) || any(any(traj.pos > ones(length(traj.pos),1)*saferegion(4:6)))
    error('The trajectory does not fall within the safe region');
end

%% Results

subplot(2,2,1)
hold on
plot3(traj.x,traj.y,traj.z,'-.b*')
plot3(traj.x(1),traj.y(1),traj.z(1),'og','MarkerFaceColor','g')
plot3(traj.x(end),traj.y(end),traj.z(end),'or','MarkerFaceColor','r')
xlabel('x');
ylabel('y');
zlabel('z');
grid on

subplot(2,2,2)
plot(traj.time,traj.x,'o');
xlabel('t (s)');
ylabel('x');

subplot(2,2,3)
plot(traj.time,traj.y,'o');
xlabel('t (s)');
ylabel('y');

subplot(2,2,4)
plot(traj.time,traj.z,'o');
xlabel('t (s)');
ylabel('z');

% print(fig,'-dpng','image.png');

end

function vec = basisgen(t,order)
% This function generates a basis vector of order n at times t.  It has a
% dimension of n+1 by size(t,1) and takes the form [t.^n t.^(n-1) ... t 1]'

persistent n

if nargin > 1
    n = order;
end

if ~isequal(size(t,2),1)
    error('t must be a column vector or scalar.');
end

vec = zeros(n+1,length(t));
for idx = 1:n+1
    vec(idx,:) = t.^(n+1-idx);
end

end

function norms = rownorm(vec)
norms = sqrt(sum(vec.^2,2));
end