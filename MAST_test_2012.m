clear
close all
clear states
addpath(genpath('/Users/yashm/Desktop/GRASP/state_control'));

Midi = midictrl('init');
fclose('all');

sim_on = 0;
height_above_pad1 = .2;
height_above_pad2 = .01;
height_above_takeoff1 = .01;
height_above_takeoff2 = .01;
cruising_height = 2;
freq=100;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
end_traj_position = [1, 0, cruising_height];
crawler_pickup = [0 0 1];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


if sim_on
    quadnames = {'1'};
    objnames = {'crawler_mat'};
    
    takeoff = [-1 -1 0];
    init_pos1 = [takeoff pi/2];
    
    charging_location = [0 0 1];
    init_pos2 = [charging_location deg2rad(45)];
    
    charge_angle = deg2rad(4);
    
    log_file = '/Users/yashm/Desktop/GRASP/state_control/logs/Sim_MAST_test_2012';
    
    axis equal
    axis([-2 2 -2 2 0 3]);
    %     view(0,0)
    patch('XData',[-2 2 2 -2],'YData',[2 2 -2 -2],'FaceColor',[0.4 0.4 0.4],'EdgeColor','none', 'FaceAlpha', 0.5);
    grid on
    xlabel('x')
    ylabel('y')
else
    quadnames = {'kilo'};
    objnames = {};
    log_file = '/Users/yashm/Desktop/GRASP/state_control/logs/MAST_test_2012';
    
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if sim_on
    for i=1:length(quadnames)
        quads(i) = start_quad(quadnames{i});
        curr_state{i} = init_state_tester(quads(i),init_pos1(i,:));
    end
    
    for i=1:length(objnames)
        objects(i) = start_object(objnames{i});
        obj_state{i} = init_state_tester(objects(i),init_pos2(i,:));
    end
else
    vicon = start_vicon;
    for i=1:length(quadnames)
        quads(i) = start_quad(quadnames{i},vicon);
        curr_state{i} = init_curr_state_vicon(quads(i),vicon);
    end
    
    for i=1:length(objnames)
        objects(i) = start_object(objnames{i},vicon);
        obj_state{i} = init_object_vicon(objects(i),vicon);
    end
    
end



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

setgains

speed = .9;
accel = .7;

if sim_on
    
else
    takeoff = [curr_state{1}.x_est curr_state{1}.y_est curr_state{1}.z_est];
    %     obj_state{1} = vicon_object_update(objects(1),obj_state{1}, vicon);
    %     crawler_pickup = [obj_state{1}.x_est obj_state{1}.y_est obj_state{1}.z_est obj_state{1}.psi];
end

start_slide = @(curr_state,states,Midi_idx, signal,state) midi_select(curr_state,states,Midi,4,'on');
advance_slide = @(curr_state,states,Midi_idx, signal,state) midi_select(curr_state,states,Midi,3,'on');
phi_45 = @(curr_state) (curr_state.phi>pi/4) || (curr_state.phi<-pi/4);
psi_error = @(curr_state,states,target,max_err,operator)angle_error(curr_state, states,target, max_err, operator);
t_inf = 1e6;
opengripperpos = 5700;
closegripperpos = 10350;

gripper_open = [1 opengripperpos t_inf ];
gripper_close = [1 closegripperpos t_inf ];



for i=1:length(quads)
    
    %do nothing while waiting for start slide
    states{i}(1) = create_state(@do_nothing,gains,'servo',gripper_open,@n_plus,1, start_slide,1,@n_plus,1);
    
    %ascend to point right above takeoff point and hover
    states{i}(end+1) = create_state(@xyz_vel, gains, [takeoff(1:2) (takeoff(3)+height_above_takeoff1)],'servo',gripper_open, 'speed',speed,'accelrate',accel,@n_plus,1);
    states{i}(end+1) = create_state(@xyz_hover, gains, [],'servo',gripper_open,@n_plus,1, @ec_timer , 1,@n_plus,1);
    
    %ascend to cruising height and rotate to ZERO yaw
    states{i}(end+1) = create_state(@xyz_vel, gains, [takeoff(1:2) cruising_height], 'speed',speed,'accelrate',accel,@n_plus,1);
%     states{i}(end+1) = create_state(@rotate_at_xyz, gains, 0,'yawrate',1,'servo',gripper_open,@n_plus,1,psi_error,-pi/2,deg2rad(5),'<=',@n_plus,1);
    %     states{i}(end+1) = create_state(@xyz_hover, gains, [],'servo',gripper_open,@n_plus,1, @ec_timer , 3,@n_plus,1);
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % Trajectory Parameters
    saferegion = [-2 -2 .4 2 2 3];
    attackangle = atan2(end_traj_position(2)-takeoff(2),end_traj_position(1)-takeoff(1)) - pi/2;
    end_traj_position = [1, -.5, cruising_height, attackangle];
    startpos = [takeoff(1:2) cruising_height attackangle];
    crawler_pickup = [crawler_pickup attackangle];
    
    % Establish correct yaw
    states{i}(end+1) = create_state(@rotate_at_xyz, gains, attackangle, ...
        'servo',gripper_open,'yawrate', 1, @n_plus,1,psi_error,attackangle,deg2rad(5),'<=',@n_plus,1);
    
    % Generate the Trajectory
    global traj
    traj = Trajectory_Generator(saferegion, startpos, crawler_pickup, end_traj_position, speed);
    
    % Run a dynamic simulation
    Dynamic_Sim(sim_on);
    
    % Create the Swooping Trajectory state
    states{i}(end+1) = create_state(@xyz_traj_J, gains, 'traj', traj, 'feedforward',true, @n_plus, 1);
    
    % Use some soft gains to stabilize
    states{i}(end+1) = create_state(@xyz_hover, soft_gains, [], 'servo',gripper_open,@n_plus,1, @ec_timer , 2,@n_plus,1);
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %move to right in front of window
    %         states{i}(end+1) = create_state(@xyz_vel, gains, [takeoff(1:2) cruising_height],'speed',speed,'accelrate',accel,'servo',gripper_close,@n_plus,1);
    %
    %     states{i}(end+1) = create_state(@xyz_hover, gains, [],'servo',gripper_close,@n_plus,1, @ec_timer , 1,@n_plus,1);
    %
    %     %move to point on other side of window
    %     states{i}(end+1) = create_state(@xyz_vel, gains, [takeoff(1:2) (takeoff(3)+height_above_takeoff2)],'servo',gripper_open,'speed',speed,'accelrate',accel,@n_plus,1);
    states{i}(end+1) = create_state(@xyz_hover, gains, [],'servo',gripper_open,@n_plus,1,advance_slide,1,@n_plus,1);
    
    states{i}(end+1) = create_state(@xyz_vel, gains, [takeoff(1:2) (takeoff(3)+height_above_takeoff2)],'servo',gripper_open, 'speed',speed,'accelrate',accel,@n_plus,1);
    states{i}(end+1) = create_state(@xyz_hover, gains, [],'servo',gripper_open,@n_plus,1, @ec_timer , 3,@n_plus,1);
    
    
    states{i}(end+1) = create_state(@do_nothing,gains,'servo',gripper_open,@n_no,1);
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

graph_handle = [];

tic
while (midi_select(curr_state,states,Midi,9,'off'))
    toc
    tic
    obj_exist = exist('objects','var');
    if sim_on
        if obj_exist
            [entity_state states]=run_states({quads objects},{curr_state obj_state}, states, {@sim_update @sim_object_update}, {1/freq 1/freq}, log_file);
            curr_state = entity_state{1};
            obj_state = entity_state{2};
        else
            [curr_state states]=run_states({quads []},curr_state, states, @sim_update, 1/freq, log_file);
        end
    else
        if obj_exist
            [entity_state states]=run_states({quads objects},{curr_state obj_state}, states, {@vicon_update @vicon_object_update}, {vicon vicon}, log_file);
            curr_state = entity_state{1};
            obj_state = entity_state{2};
        else
            [curr_state states]=run_states({quads []},curr_state, states, @vicon_update, vicon, log_file);
        end
    end
    [curr_state{1}.x_est curr_state{1}.y_est curr_state{1}.z_est curr_state{1}.psi curr_state{1}.theta curr_state{1}.phi]
    %     curr_state{1}.z_est - obj_state{1}.z_est
    %      [obj_state{1}.x_est obj_state{1}.y_est obj_state{1}.z_est obj_state{1}.psi]
end

clean_states
if ~sim_on
    for i=1:length(quads)
        kill_thrust_quad(quads(i))
        
        kill_thrust_quad(quads(i))
        stop_quad(quads(i))
    end
    stop_vicon(vicon)
end

midictrl('close',Midi);