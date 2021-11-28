clear all
%% Parameter Definition
% (initial) Number of points per trajectory:
m = 500;
% in range [-pi*m_range, pi*m_range]
m_range = 1;

% max/min angles:
min_roll = -20;
max_roll = 20;
min_pitch = -15;
max_pitch = 15;
min_yaw = -5;
max_yaw = 5;

% Difference of sums between points:
max_dist = 0.5;
max_fixed_waypoints = 50;
SEED = 43632;

% Number of trajectories to be generated
num_of_trajectories = 100;



%generate_static_drone_static_wind(SEED, num_of_trajectories, m, min_roll, max_roll, min_pitch, max_pitch, min_yaw, max_yaw, m_range, max_dist,max_fixed_waypoints)

generate_static_drone_dynamic_wind(SEED, num_of_trajectories, m, min_roll, max_roll, min_pitch, max_pitch, min_yaw, max_yaw, m_range, max_dist,max_fixed_waypoints)
generate_dynamic_drone_static_wind(SEED, num_of_trajectories, m, min_roll, max_roll, min_pitch, max_pitch, min_yaw, max_yaw, m_range, max_dist,max_fixed_waypoints)
generate_dynamic_drone_dynamic_wind(SEED, num_of_trajectories, m, min_roll, max_roll, min_pitch, max_pitch, min_yaw, max_yaw, m_range, max_dist,max_fixed_waypoints)
generate_dynamic_force_torque(SEED, num_of_trajectories, m, torque_min, torque_max, torque_min, torque_max, torque_min, torque_max, force_min, force_max, force_min, force_max, force_min, force_max, m_range, max_change_torque, max_change_force, max_fixed_waypoints,interval);

%torque_min = -3.9;
%torque_max = 3.9;
%max_change_torque=2;

%force_min = -28;
%force_max = 28;
%max_change_force = 10;
%interval=0.05;

% (soft) cutoff time
%cutoff_time = 10;

%generate_long_dynamic_force_torque(SEED,num_of_trajectories, m, torque_min, torque_max, torque_min, torque_max, torque_min, torque_max, force_min, force_max, force_min, force_max, force_min, force_max, m_range, max_change_torque, max_change_force, cutoff_time,interval);
%generate_long_static_force_torque(SEED, force_min, force_max, torque_min, torque_max, 2, 2,cutoff_time)

%cutoff_time = 20;
%SEED = 48734;
%generate_long_dynamic_force_torque(SEED,num_of_trajectories, m, torque_min, torque_max, torque_min, torque_max, torque_min, torque_max, force_min, force_max, force_min, force_max, force_min, force_max, m_range, max_change_torque, max_change_force, cutoff_time,interval);
%generate_long_static_force_torque(SEED, force_min, force_max, torque_min, torque_max, 2, 2,cutoff_time)

%cutoff_time = 40;
%SEED = 4875345;
%generate_long_dynamic_force_torque(SEED,num_of_trajectories, m, torque_min, torque_max, torque_min, torque_max, torque_min, torque_max, force_min, force_max, force_min, force_max, force_min, force_max, m_range, max_change_torque, max_change_force, cutoff_time,interval);
%generate_long_static_force_torque(SEED, force_min, force_max, torque_min, torque_max, 2, 2,cutoff_time)


function [] = generate_static_drone_dynamic_wind(SEED, num_of_trajectories, m, min_roll, max_roll, min_pitch, max_pitch, min_yaw, max_yaw, m_range, max_dist,max_fixed_waypoints)

    rng(SEED);
    folder_name = 'static_drone_dynamic_wind';
    if ~exist(folder_name, 'dir')
           mkdir(folder_name)
    end

    remain_static_for = 50;

    for i=1:num_of_trajectories
        arrays = lhsdesign(max_fixed_waypoints,7);

   
        arrays(:,1) = normalize_to_vals(arrays(:,1), min_roll, max_roll);
        arrays(:,2) = normalize_to_vals(arrays(:,2), min_pitch, max_pitch);
        arrays(:,3) = normalize_to_vals(arrays(:,3), min_yaw, max_yaw);

        arrays = [zeros(1,7);arrays;zeros(1,7)];

        pitch = zeros(0,1);
        roll = zeros(0,1);
        yaw = zeros(0,1);
        strided_pitch = zeros(0,1);
        strided_roll = zeros(0,1);
        strided_yaw = zeros(0,1);

        wind = zeros(0,1);
        for i=1:max_fixed_waypoints-1
            [new_r, new_p, new_y] = connect_x_to_y(arrays(i,:), arrays(i+1,:), max_dist);

            roll = [roll; new_r; ones(remain_static_for,1)*arrays(i+1,1)];
            pitch = [pitch; new_p; ones(remain_static_for,1)*arrays(i+1,2)];
            yaw = [yaw; new_y; ones(remain_static_for,1)*arrays(i+1,3)];

            new_r(2:length(new_r)-1) = 0;
            strided_roll = [strided_roll; new_r;ones(remain_static_for,1)*arrays(i+1,1)];

            new_p(2:length(new_p)-1) = 0; 
            strided_pitch = [strided_pitch; new_p;ones(remain_static_for,1)*arrays(i+1,2)];

            new_y(2:length(new_y)-1) = 0; 
            strided_yaw = [strided_yaw; new_y; ones(remain_static_for,1)*arrays(i+1,3)];


            windrange = min(remain_static_for, 20);
            if isempty(wind)
                firstelem = 0;
                new_walk = create_random_walk(remain_static_for, 10, 50);

            else
                firstelem = wind(end);
                % increase or decrease start of new walk by at least 10%, cap at 50 and 90
                start = max(min(wind(end) + (5*(arrays(i,4)-0.5)),100-windrange),50);
                sum(arrays(:,4)-0.5)
                new_walk = create_random_walk(remain_static_for, windrange, start);
            end

            connect = linspace(firstelem,new_walk(1),length(new_p));

            wind = [wind; connect';new_walk'];
        end
        
    

        fprintf('Number of elements in second trajectory: %f\n', nnz(strided_roll))

        figure(1)
        plot(wind)

        figure(2)
        scatter3(roll, pitch, yaw, 'b', 'filled')
        hold on
        scatter3(strided_roll, strided_pitch, strided_yaw, 'g', 'filled')
        hold off
        grid on
        legend('Normal', 'Subsampled')

        id = string(round(rand*1000));
        filename = strcat('attitude_inputs_lissajous_',string(i),'_',id,'.csv');
        output_as_robot_csv(roll, pitch, yaw, fullfile(folder_name,filename));
        filename = strcat('attitude_inputs_lissajous_',string(i),'_',id,'_subsampled.csv');
        output_as_robot_csv(strided_roll, strided_pitch, strided_yaw, fullfile(folder_name,filename));
        writematrix(wind,fullfile(folder_name, strcat('wind_',string(i),'_',id,'.csv')));
        fprintf('Saved with file id: '+id+'\n')
    end
end

 


function [] = generate_dynamic_drone_static_wind(SEED, num_of_trajectories, m, min_roll, max_roll, min_pitch, max_pitch, min_yaw, max_yaw, m_range, max_dist,max_fixed_waypoints)

    rng(SEED);
    folder_name = 'dynamic_drone_static_wind';
    if ~exist(folder_name, 'dir')
           mkdir(folder_name)
    end
    sample_arrays = lhsdesign(num_of_trajectories, 7);
   
    for i=1:num_of_trajectories
        [roll, pitch, yaw] = generate_angles(m, min_roll, max_roll, min_pitch, max_pitch, min_yaw, max_yaw, m_range, sample_arrays(i,:));

        [roll, pitch, yaw] = interpolate_until_small(roll, pitch, yaw, max_dist);

        difference = summed_differences(roll, pitch, yaw);

        fprintf('Max distance between points: %f\n', max(difference))
        fprintf('Min distance between points: %f\n', min(difference))

        n_nonlinear_points = size(roll,1);

        [roll, pitch, yaw] = connect_to_zero(roll, pitch, yaw, max_dist);
        fprintf('Final max distance between points (with trajectory to 0): %f\n', max(difference))

        n_linear_points = size(roll,1)-n_nonlinear_points;

        random_walk = static_wind(n_linear_points, n_nonlinear_points, 50,50);

        figure(1)
        plot(random_walk)
        stride = ceil(size(roll)/max_fixed_waypoints);

        strided_roll = roll;
        strided_pitch = pitch;
        strided_yaw = yaw;

        idx = setdiff(1:size(roll),1:stride:size(roll));
        strided_roll(idx) = 0;
        strided_pitch(idx) = 0;
        strided_yaw(idx) = 0;

        fprintf('Number of elements in second trajectory: %f\n', nnz(strided_roll))

        figure(2)
        scatter3(roll, pitch, yaw, 'b', 'filled')
        hold on
        scatter3(strided_roll, strided_pitch, strided_yaw, 'g', 'filled')
        hold off
        grid on
        legend('Normal', 'Subsampled')

        id = string(round(rand*1000));
        filename = strcat('attitude_inputs_lissajous_',string(i),'_',id,'.csv');
        output_as_robot_csv(roll, pitch, yaw, fullfile(folder_name,filename));
        filename = strcat('attitude_inputs_lissajous_',string(i),'_',id,'_subsampled.csv');
        output_as_robot_csv(strided_roll, strided_pitch, strided_yaw, fullfile(folder_name,filename));
        writematrix(random_walk',fullfile(folder_name, strcat('wind_',string(i),'_',id,'.csv')));
        fprintf('Saved with file id: '+id+'\n')
    end
end


function [] = generate_dynamic_drone_dynamic_wind(SEED, num_of_trajectories, m, min_roll, max_roll, min_pitch, max_pitch, min_yaw, max_yaw, m_range, max_dist,max_fixed_waypoints)

    rng(SEED);
    folder_name = 'dynamic_drone_dynamic_wind';
    if ~exist(folder_name, 'dir')
           mkdir(folder_name)
    end
    sample_arrays = lhsdesign(num_of_trajectories, 7);

    for i=1:num_of_trajectories
        [roll, pitch, yaw] = generate_angles(m, min_roll, max_roll, min_pitch, max_pitch, min_yaw, max_yaw, m_range, sample_arrays(i,:));

        [roll, pitch, yaw] = interpolate_until_small(roll, pitch, yaw, max_dist);

        difference = summed_differences(roll, pitch, yaw);

        fprintf('Max distance between points: %f\n', max(difference))
        fprintf('Min distance between points: %f\n', min(difference))

        n_nonlinear_points = size(roll,1);

        [roll, pitch, yaw] = connect_to_zero(roll, pitch, yaw, max_dist);
        fprintf('Final max distance between points (with trajectory to 0): %f\n', max(difference))


        n_linear_points = size(roll,1)-n_nonlinear_points;

        random_walk = create_random_walk_zero(n_linear_points, n_nonlinear_points, 50,50);

        figure(1)
        plot(random_walk)
        stride = ceil(size(roll)/max_fixed_waypoints);

        strided_roll = roll;
        strided_pitch = pitch;
        strided_yaw = yaw;

        idx = setdiff(1:size(roll),1:stride:size(roll));
        strided_roll(idx) = 0;
        strided_pitch(idx) = 0;
        strided_yaw(idx) = 0;

        fprintf('Number of elements in second trajectory: %f\n', nnz(strided_roll))

        figure(2)
        scatter3(roll, pitch, yaw, 'b', 'filled')
        hold on
        scatter3(strided_roll, strided_pitch, strided_yaw, 'g', 'filled')
        hold off
        grid on
        legend('Normal', 'Subsampled')

        id = string(round(rand*1000));
        filename = strcat('attitude_inputs_lissajous_', string(i), '_', id, '.csv');
        output_as_robot_csv(roll, pitch, yaw, fullfile(folder_name, filename));
        filename = strcat('attitude_inputs_lissajous_', string(i), '_', id, '_subsampled.csv');
        output_as_robot_csv(strided_roll, strided_pitch, strided_yaw, fullfile(folder_name,filename));
        writematrix(random_walk', fullfile(folder_name, strcat('wind_',string(i),'_',id,'.csv')));
        fprintf('Saved with file id: '+id+'\n')
    end
end
%% Wind/Trajectory generation components

%function[x,y,z] = generate_static_trajectory(SEED, )


%% Force-Torque generation

function [] = generate_dynamic_force_torque(SEED, num_of_trajectories, m, min_torque_x, max_torque_x, min_torque_y, max_torque_y, min_torque_z, max_torque_z, min_force_x, max_force_x, min_force_y, max_force_y, min_force_z, max_force_z, m_range, max_dist_torque, max_dist_force,max_fixed_waypoints,interval);

    rng(SEED);
    folder_name = 'dynamic_force_torque';
    if ~exist(folder_name, 'dir')
           mkdir(folder_name);
    end
    sample_arrays = lhsdesign(num_of_trajectories, 14);

    for i=1:num_of_trajectories
        [torque_x, torque_y, torque_z] = generate_angles(m, min_torque_x, max_torque_x, min_torque_y, max_torque_y, min_torque_z, max_torque_z, m_range, sample_arrays(i,:));
        [torque_x, torque_y, torque_z] = interpolate_until_small(torque_x, torque_y, torque_z, max_dist_torque);

        [force_x, force_y, force_z] = generate_angles(m, min_force_x, max_force_x, min_force_y, max_force_y, min_force_z, max_force_z, m_range, sample_arrays(i,7:end));

        [force_x, force_y, force_z] = interpolate_until_small(force_x, force_y, force_z, max_dist_force);
        

        % Ensure both have same length
        max_len = max(length(torque_x),length(force_x));
        [torque_x, torque_y, torque_z] = interpolate_trajectory(torque_x, torque_y, torque_z, max_len);
        [force_x, force_y, force_z] = interpolate_trajectory(force_x, force_y, force_z, max_len);


        [torque_x, torque_y, torque_z] = pivot_to_smallest(torque_x, torque_y, torque_z);
        [force_x, force_y, force_z] = pivot_to_smallest(force_x, force_y, force_z);

        

        [fx_to_zero, fy_to_zero, fz_to_zero] = interpolate_until_small(transpose(linspace(0, force_x(1),10)),transpose(linspace(0, force_y(1),10)),transpose(linspace(0, force_z(1),10)),max_dist_force);
        [tx_to_zero, ty_to_zero, tz_to_zero] = interpolate_until_small(transpose(linspace(0, torque_x(1),10)),transpose(linspace(0, torque_y(1),10)),transpose(linspace(0, torque_z(1),10)),max_dist_torque);

        len_zero_traj = max(length(fx_to_zero), length(tx_to_zero));

        [fx_to_zero, fy_to_zero, fz_to_zero] = interpolate_until_small(transpose(linspace(0, force_x(1),len_zero_traj)),transpose(linspace(0, force_y(1),len_zero_traj)),transpose(linspace(0, force_z(1),len_zero_traj)),max_dist_force);
        [tx_to_zero, ty_to_zero, tz_to_zero] = interpolate_until_small(transpose(linspace(0, torque_x(1),len_zero_traj)),transpose(linspace(0, torque_y(1),len_zero_traj)),transpose(linspace(0, torque_z(1),len_zero_traj)),max_dist_torque);

        size(tx_to_zero)
        size(torque_x)

        torque_x = [tx_to_zero;torque_x;flip(tx_to_zero)];
        torque_y = [ty_to_zero;torque_y;flip(ty_to_zero)];
        torque_z = [tz_to_zero;torque_z;flip(tz_to_zero)];

        force_x = [fx_to_zero;force_x;flip(fx_to_zero)];
        force_y = [fy_to_zero;force_y;flip(fy_to_zero)];
        force_z = [fz_to_zero;force_z;flip(fz_to_zero)];


%        fprintf('Final max distance between points (with trajectory to 0): %f\n', max(difference))
        figure(1)

        scatter3(force_x, force_y, force_z, 'b', 'filled')
        hold on
        scatter3(torque_x, torque_y, torque_z, 'g', 'filled')
        hold off
        grid on
       
        filename = strcat('dynamic_', string(i), '.csv');

        time = ones(size(torque_z)) * interval;
        time(1) = 0;
        time = cumsum(time);

        output_force_torque_csv(force_x,force_y,force_z,torque_x,torque_y,torque_z, time,fullfile(folder_name, filename))

    end
end

function [] = generate_long_dynamic_force_torque(SEED,num_of_trajectories, m, min_torque_x, max_torque_x, min_torque_y, max_torque_y, min_torque_z, max_torque_z, min_force_x, max_force_x, min_force_y, max_force_y, min_force_z, max_force_z, m_range, max_dist_torque, max_dist_force,timecut,interval);
    timecut = timecut*60;

    rng(SEED);
    folder_name = 'dynamic_force_torque';
    if ~exist(folder_name, 'dir')
           mkdir(folder_name);
    end
    sample_arrays = lhsdesign(num_of_trajectories, 14);

    [fx,fy,fz,tx,ty,tz,t] = deal([]);

    for i=1:num_of_trajectories
        [torque_x, torque_y, torque_z] = generate_angles(m, min_torque_x, max_torque_x, min_torque_y, max_torque_y, min_torque_z, max_torque_z, m_range, sample_arrays(i,:));
        [torque_x, torque_y, torque_z] = interpolate_until_small(torque_x, torque_y, torque_z, max_dist_torque);

        [force_x, force_y, force_z] = generate_angles(m, min_force_x, max_force_x, min_force_y, max_force_y, min_force_z, max_force_z, m_range, sample_arrays(i,7:end));

        [force_x, force_y, force_z] = interpolate_until_small(force_x, force_y, force_z, max_dist_force);
        

        % Ensure both have same length
        max_len = max(length(torque_x),length(force_x));
        [torque_x, torque_y, torque_z] = interpolate_trajectory(torque_x, torque_y, torque_z, max_len);
        [force_x, force_y, force_z] = interpolate_trajectory(force_x, force_y, force_z, max_len);


        [torque_x, torque_y, torque_z] = pivot_to_smallest(torque_x, torque_y, torque_z);
        [force_x, force_y, force_z] = pivot_to_smallest(force_x, force_y, force_z);

        [old_fx,old_fy, old_fz, old_tx, old_ty, old_tz] = deal(0);

        if  ~isempty(fx)
            old_fx = fx(end);
            old_fy = fy(end); 
            old_fz = fz(end); 
            old_tx = tx(end);
            old_ty = ty(end);
            old_tz = tz(end);
        end



        [fx_to_zero, fy_to_zero, fz_to_zero] = interpolate_until_small(transpose(linspace(old_fx, force_x(1),10)),transpose(linspace(old_fy, force_y(1),10)),transpose(linspace(old_fz, force_z(1),10)),max_dist_force);
        [tx_to_zero, ty_to_zero, tz_to_zero] = interpolate_until_small(transpose(linspace(old_tx, torque_x(1),10)),transpose(linspace(old_ty, torque_y(1),10)),transpose(linspace(old_tz, torque_z(1),10)),max_dist_torque);

        len_zero_traj = max(length(fx_to_zero), length(tx_to_zero));

        [fx_to_zero, fy_to_zero, fz_to_zero] = interpolate_until_small(transpose(linspace(old_fx, force_x(1),len_zero_traj)),transpose(linspace(old_fy, force_y(1),len_zero_traj)),transpose(linspace(old_fz, force_z(1),len_zero_traj)),max_dist_force);
        [tx_to_zero, ty_to_zero, tz_to_zero] = interpolate_until_small(transpose(linspace(old_tx, torque_x(1),len_zero_traj)),transpose(linspace(old_ty, torque_y(1),len_zero_traj)),transpose(linspace(old_tz, torque_z(1),len_zero_traj)),max_dist_torque);


        torque_x = [tx_to_zero;torque_x];
        torque_y = [ty_to_zero;torque_y];
        torque_z = [tz_to_zero;torque_z];

        force_x = [fx_to_zero;force_x];
        force_y = [fy_to_zero;force_y];
        force_z = [fz_to_zero;force_z];


        figure(1)

        scatter3(force_x, force_y, force_z, 'b', 'filled')
        hold on
        scatter3(torque_x, torque_y, torque_z, 'g', 'filled')
        hold off
        grid on
       

        time = ones(size(torque_z)) * interval;
        time(1) = 0;
        time = cumsum(time);

        if ~isempty(t)
            time = t(end)+time;
        end

        fx = [fx;force_x];
        fy = [fy;force_y];
        fz = [fz;force_z];

        tx = [tx;torque_x];
        ty = [ty;torque_y];
        tz = [tz;torque_z];
        t = [t;time];
        if t(end) > timecut
            break
        end
    end
    [fx_to_zero, fy_to_zero, fz_to_zero] = interpolate_until_small(transpose(linspace(0, fx(end),10)),transpose(linspace(0, fy(end),10)),transpose(linspace(0, fz(end),10)),max_dist_force);
    [tx_to_zero, ty_to_zero, tz_to_zero] = interpolate_until_small(transpose(linspace(0, tx(end),10)),transpose(linspace(0, ty(end),10)),transpose(linspace(0, tz(end),10)),max_dist_torque);

    len_zero_traj = max(length(fx_to_zero), length(tx_to_zero));
    len_zero_traj
    [fx_to_zero, fy_to_zero, fz_to_zero] = interpolate_until_small(transpose(linspace(fx(end), 0,len_zero_traj)),transpose(linspace(fy(end), 0,len_zero_traj)),transpose(linspace(fz(end),0,len_zero_traj)),max_dist_force);
    [tx_to_zero, ty_to_zero, tz_to_zero] = interpolate_until_small(transpose(linspace(tx(end), 0,len_zero_traj)),transpose(linspace(ty(end), 0,len_zero_traj)),transpose(linspace(tz(end),0,len_zero_traj)),max_dist_torque);

    fx = [fx;fx_to_zero];
    fy = [fy;fy_to_zero];
    fz = [fz;fz_to_zero];

    tx = [tx;tx_to_zero];
    ty = [ty;ty_to_zero];
    tz = [tz;tz_to_zero];

    time = ones(size(fx_to_zero)) * interval;
    time(1) = 0;
    time = cumsum(time);
    time = t(end)+time;
    t = [t;time];

    filename = strcat('dynamic_',int2str(round(timecut/60)),'.csv');
    output_force_torque_csv(fx,fy,fz,tx,ty,tz, t,fullfile(folder_name, filename))


end

function [] = generate_long_static_force_torque(SEED, min_f, max_f, min_t, max_t, static_time,move_time,cutoff)
    rng(SEED)

    folder_name = 'static_force_torque';
    if ~exist(folder_name, 'dir')
           mkdir(folder_name);
    end

    runtime = cutoff*60;
    N = floor(runtime/(static_time+move_time));

    pts = repelem(lhsdesign(N,6),2,1);
    pts(:,1:3) = pts(:,1:3)*(min_f-max_f)-min_f;
    pts(:,4:6) = pts(:,4:6)*(min_t-max_t)-min_t;

    scatter3(pts(:,1),pts(:,2),pts(:,3))
    time = repmat([move_time;static_time],N,1);
    time = cumsum(time);
    
    index = (0:length(time)-1)';
    
    % Add 2 seconds of zeros
    output = [index pts time+2];
    output = [zeros(1,8);output];
    output = [zeros(1,8);output];
    output(2,8) = 2;

    colnames = {'index','fx','fy','fz','tx','ty','tz','time'};

    index = compose("%d",((0:length(output)-1)'));

    table = array2table(output,'RowNames',index,'VariableNames',colnames);
    filename = strcat('static_',int2str(round(cutoff)),'.csv');
    filename = fullfile(folder_name,filename);
    writetable(table,filename);
end


%% Helper Functions
function [roll, pitch, yaw] = generate_angles(m, min_roll, max_roll, min_pitch, max_pitch, min_yaw, max_yaw, m_range, sample_arrays)    
    % Adjust range
    t = linspace(-pi*m_range, pi*m_range, m)';
    
    % Normalize to largest degree interval
    max_deg = max([(max_roll-min_roll),(max_pitch-min_pitch),(max_yaw-min_yaw)]);
    
    a = sample_arrays(:,1)*(max_roll - min_roll)/max_deg;
    b = sample_arrays(:,2)*(max_pitch - min_pitch)/max_deg;
    c = sample_arrays(:,3)*(max_yaw - min_yaw)/max_deg;
    
    delta = pi./sample_arrays(:,4);
    phi = pi./sample_arrays(:,5);
    
    n = sample_arrays(:,6)*5+5;
    m1 = sample_arrays(:,7)*5+5;
    
    roll = normalize_to_vals(a.*sin(round(min([n,m1]))*t + delta), min_roll, max_roll);
    pitch = normalize_to_vals(b.*sin(round(max([n,m1]))*t), min_pitch, max_pitch);
    yaw = normalize_to_vals(c.*sin(t+phi), min_yaw, max_yaw);
end

function x = normalize_to_vals(x, min_val, max_val)
    x = x - min(x);
    x = x/max(x);
    x = x * (max_val - min_val);
    x = x + min_val;
end 

function [x, y, z] = interpolate_trajectory(pitch, roll, yaw, m)
    t = [0;cumsum(summed_differences(pitch, roll, yaw))*0.5];
    t = t/t(end);
    ti = linspace(0,1,m);
    x = spline(t,roll,ti)';
    y = spline(t,pitch,ti)';
    z = spline(t,yaw,ti)';
end

function [sum_differece] = summed_differences(roll, pitch, yaw)
    sum_differece = abs(diff(roll))+ abs(diff(pitch))+abs(diff(yaw));
end

function[] = output_force_torque_csv(fx,fy,fz,tx,ty,tz,time,filename)
colnames = {'index','fx','fy','fz','tx','ty','tz','time'};

index = (0:length(fx)-1)';

output = [index fx fy fz tx ty tz time+2];
output = [zeros(1,8);output];

index = compose("%d",((0:length(fx))'));
table = array2table(output,'RowNames',index,'VariableNames',colnames);
writetable(table,filename);

end

function[] = output_as_robot_csv(roll, pitch, yaw, filename)
    pitch=90-pitch; roll=90-roll;
    m = size(roll);
    
    spacer=zeros(m);
    x_pos=zeros(m)-50.000000;
    y_pos=zeros(m)-1218.223000; 
    z_pos=zeros(m)+242.834000;

    output=[spacer x_pos y_pos z_pos pitch yaw roll];
  
  
    writematrix(output,filename)
end

function[x, y, z] = pivot_to_smallest(x,y,z)
    % returns xyz such that closest element is at start
   distance_to_zero = sqrt(x.^2+y.^2+z.^2);
   [~, idx] = min(distance_to_zero);

   x = [x(idx:size(x)); x(1:idx-1)];
   y = [y(idx:size(y)); y(1:idx-1)];
   z = [z(idx:size(z)); z(1:idx-1)];

end

function[pitch, roll, yaw] = connect_to_zero(pitch, roll, yaw, max_dist)
    % (for closed loops: connect points to the starting point (zero))

    [pitch, roll, yaw] = pivot_to_smallest(pitch, roll, yaw);
   
    [pitch_to_zero, roll_to_zero, yaw_to_zero] = interpolate_until_small(transpose(linspace(0, pitch(1),10)),transpose(linspace(0, roll(1),10)),transpose(linspace(0, yaw(1),10)),max_dist);
   
    pitch = [pitch_to_zero;pitch; flip(pitch_to_zero)];
    roll = [roll_to_zero; roll; flip(roll_to_zero)];
    yaw = [yaw_to_zero; yaw; flip(yaw_to_zero)];
end

function[x,y,z] = connect_x_to_y(x,y, max_dist)
    % connect x (3x point) to y (3x point)
    [x, y, z] = interpolate_until_small(transpose(linspace(x(1), y(1),10)),transpose(linspace(x(2), y(2),10)),transpose(linspace(x(3), y(3),10)),max_dist);
end

function[pitch,roll,yaw]= interpolate_until_small(pitch, roll, yaw, max_dist)
    m = size(pitch, 1);
    max_dist_measured = max(summed_differences(pitch, roll, yaw));
    
    while max_dist_measured >= max_dist
        m = m * (max(summed_differences(pitch, roll, yaw))/max_dist);
       [pitch, roll, yaw] = interpolate_trajectory(roll, pitch, yaw, m);
        max_dist_measured = max(summed_differences(roll, pitch, yaw));
    end

end

function [random_walk] = create_random_walk_zero(n_linear_points, n_nonlinear_points, range, minimum)
    random_walk = create_random_walk(n_nonlinear_points, range,minimum);

    linear_increase = linspace(0,random_walk(1),n_linear_points/2);

    linear_decrease = linspace(random_walk(end),0,n_linear_points/2);

    random_walk = [linear_increase,random_walk,linear_decrease];
end

function [random_walk] = create_random_walk(n_nonlinear_points, range, minimum)
    random_walk = smoothdata(cumsum(randn(1,n_nonlinear_points)));

    random_walk = random_walk - min(random_walk);
    random_walk = random_walk/max(random_walk)*range;
    random_walk = random_walk + minimum;

end



function [static_w] = static_wind(n_linear_points, n_nonlinear_points, range, minimum)
    static_w = rand * ones(1,n_nonlinear_points);
    static_w = static_w * range;
    static_w = static_w + minimum;
    
    linear_increase = linspace(0,static_w(1),n_linear_points/2);

    linear_decrease = linspace(static_w(end),0,n_linear_points/2);

    static_w = [linear_increase,static_w,linear_decrease];
    
end
