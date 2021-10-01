clear all
%% Parameter Definition
% Number of points per trajectory:
m = 300;
% in range [-pi*m_range, pi*m_range]
m_range = 1;

% max/min angles:
min_roll = -20;
max_roll = 20;
min_pitch = -15;
max_pitch = 15;
min_yaw = -5;
max_yaw = 5;

% Sum between points:
max_dist = 0.5;
max_fixed_waypoints = 500;

rng(48548795);

[roll, pitch, yaw] = generate_angles(m, min_roll, max_roll, min_pitch, max_pitch, min_yaw, max_yaw, m_range);

[roll, pitch, yaw] = interpolate_until_small(roll, pitch, yaw, max_dist);

difference = summed_differences(roll, pitch, yaw);
fprintf('Max distance between points: %f\n', max(difference))
fprintf('Min distance between points: %f\n', min(difference))

[roll, pitch, yaw] = connect_to_zero(roll, pitch, yaw, max_dist);
fprintf('Final max distance between points (with trajectory to 0): %f\n', max(difference))

stride = ceil(size(roll)/max_fixed_waypoints);

strided_roll = roll;
strided_pitch = pitch;
strided_yaw = yaw;

idx = setdiff(1:size(roll),1:stride:size(roll));
strided_roll(idx) = 0;
strided_pitch(idx) = 0;
strided_yaw(idx) = 0;

fprintf('Number of elements in second trajectory: %f\n', nnz(strided_roll))

figure(1)
scatter3(roll, pitch, yaw, 'b', 'filled')
hold on
scatter3(strided_roll, strided_pitch, strided_yaw, 'g', 'filled')
hold off
grid on
legend('Normal', 'Subsampled')

difference = summed_differences(roll, pitch, yaw);
id = string(round(rand*1000));
filename = strcat('attitude_inputs_lissajous_',id,'.csv');
output_as_robot_csv(roll, pitch, yaw, filename);
filename = strcat('attitude_inputs_lissajous_',id,'_subsampled.csv');
output_as_robot_csv(strided_roll, strided_pitch, strided_yaw, filename)
fprintf('Saved with file id: '+id+'\n')
%% Functions
function [roll, pitch, yaw] = generate_angles(m, min_roll, max_roll, min_pitch, max_pitch, min_yaw, max_yaw, m_range)
    sample_arrays = lhsdesign(1, 7);
    
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

function[pitch, roll, yaw] = connect_to_zero(pitch, roll, yaw, max_dist)
    % (for closed loops: connect points to the starting point (zero))
   distance_to_zero = sqrt(pitch.^2+roll.^2+yaw.^2);
   [~, idx] = min(distance_to_zero);
   
   pitch = [pitch(idx:size(pitch)); pitch(1:idx-1)];
   roll = [roll(idx:size(roll)); roll(1:idx-1)];
   yaw = [yaw(idx:size(yaw)); yaw(1:idx-1)];
   
   [pitch_to_zero, roll_to_zero, yaw_to_zero] = interpolate_until_small(transpose(linspace(0, pitch(1),10)),transpose(linspace(0, roll(1),10)),transpose(linspace(0, yaw(1),10)),max_dist);
   
   pitch = [pitch_to_zero;pitch; flip(pitch_to_zero)];
   roll = [roll_to_zero; roll; flip(roll_to_zero)];  
   yaw = [yaw_to_zero; yaw; flip(yaw_to_zero)];
end

function[pitch,roll,yaw]= interpolate_until_small(pitch, roll, yaw, max_dist)
    m = size(pitch, 1);
    max_dist_measured = max(summed_differences(pitch, roll, yaw));
    
    while max_dist_measured >= max_dist
        m = m * (max(summed_differences(pitch, roll, yaw))/max_dist);
       [pitch, roll, yaw] = interpolate_trajectory(roll, pitch, yaw, m);
        max_dist_measured = max(summed_differences(roll, pitch, yaw));
    end
    min_dist_measured = min(summed_differences(pitch, roll, yaw));

end

