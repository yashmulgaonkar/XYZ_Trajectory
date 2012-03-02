function process(binaryfile, trajfile)
% This function takes a binary file as a result of a flown trajectory and
% the corresponding trajectory .mat file and plots general analysis on the
% results.  If no arguments are provided, the function looks for a
% tests.mat file which is a structure containing the following fields:
% tests{idx}.bin, tests{idx}.traj that contain the binary filename and the
% traj filename.  Note: the associated txt file must be named the same as
% the binary file with the exception of the extension.

if ~isequal(nargin,0)
    tests{1}.bin = binaryfile;
    tests{1}.traj = trajfile;
else
    load('tests.mat');
end

% Loop through the tests if there are multiple
for idx = 1:length(tests)
    
    % Get the log
    log = quad_read_log(tests{idx}.bin);
    % disp(['Using log: ', tests{idx}.bin]);
    
    % Correct the time
    [time,~]=fix_log_time(log);
    
    % Determine the start and finish times by processing the txt file
    txt_file = regexprep(tests{idx}.bin, '.bin','.txt');
    h = fopen(txt_file);
    idx2 = 1;
    while ~feof(h)
        tline{idx2} = fgetl(h); %#ok<AGROW>
        if ~isempty(regexp(tline{idx2},'xyz_traj_J', 'once'))
            start_time_line = idx2 - 3;
            end_time_line = idx2 + 3;
        end
        idx2 = idx2 + 1;
    end
    
    % Extract the desired strings
    [~, ~, ~, matchstring] = regexp(tline{start_time_line},'total_time=[0123456789.]+','once');
    starttime = str2double(matchstring(12:end));
    
    [~, ~, ~, matchstring] = regexp(tline{end_time_line},'total_time=[0123456789.]+','once');
    endtime = str2double(matchstring(12:end));
    
    % Determine the boundary indices
    startidx = find(time >= starttime,1,'first');
    endidx = find(time <= endtime,1,'last');
    
    % disp(['Start time: ', num2str(tests{idx}.starttime), ' seconds']);
    % disp(['End time: ', num2str(tests{idx}.endtime), ' seconds']);
    disp(['Trajectory Duration: ', num2str(endtime-starttime), ' seconds']);
    
    % Extract the interesting data
    indicies = startidx:endidx;
    time = time(indicies) - time(startidx);
    xpos = log(indicies,8);
    ypos = log(indicies,9);
    zpos = log(indicies,10);
    phi = log(indicies,1);
    theta = log(indicies,2);
    psi = log(indicies,11);
    
    % Create the states matrix
    states = [xpos, ypos, zpos, zeros(length(log(indicies,8)),3), phi, theta, psi];
    
    % Load the trajectory
    load(tests{idx}.traj);
    
    % disp(['Using trajectory: ', tests{idx}.traj]);
    traj.pos = [log(indicies,12), log(indicies,13), log(indicies,14)];
    traj.phi = log(indicies,20);
    traj.theta = log(indicies,19);
    traj.psi = log(indicies,15);
    traj.time = time;
    
    plots(traj, true, states, time)
    
end

end