function Dynamic_Sim(varargin)

% We only want to run the simulation if true is provided as an argument
if (nargin > 0) && varargin{1}
    
    global log
    
    disp('Running Dynamic Simulation...');
    
    % Initalize params
    params = load_params();
    
    % Load the trajectory into the Global Workspace
    % hover
    % Trajectory_Generator
    
    % Load the trajectory
    load('traj')
    
    % logs
    % log.w_des = [];
    log.phi_des = [];
    log.theta_des = [];
    log.F = [];
    log.w = [];
    % log.pos_e = [];
    log.pos = [];
    log.time = [];
    log.u = [];
    
    % Initalize the controller
    feval(params.controlHandle,[],[],params,traj);
    
    s0 = [traj.pos(1,:)'; zeros(3,1); traj.phi(1); traj.theta(1); traj.psi(1); zeros(3,1); params.w_hover*ones(4,1)];
    tf = traj.keytimes(end);
    
    % We can now determine the output of the system
    [time states] = ode45(@(t,s)Simulator(t,s,params,traj),[0,tf],s0);
    
    plots(traj,true,states,time)
    
end

end