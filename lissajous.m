clear all
%% Parameter Definition
% Number of points per trajectory:
m = 2500;
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
max_dist = 1;

rng(428341);

[roll, pitch, yaw] = generate_angles(m, min_roll, max_roll, min_pitch, max_pitch, min_yaw, max_yaw, m_range);
[roll, pitch, yaw] = interpolate_trajectory(roll, pitch, yaw, m);
difference = summed_differences(roll, pitch, yaw);
fprintf('Max difference between points: %f\n', max(difference))
fprintf('Min difference between points: %f\n', min(difference))


scatter3(roll, pitch, yaw);

pitch=90-pitch; roll=90-roll;

spacer=zeros(m,1);
x_pos=zeros(m,1)-50.000000;
y_pos=zeros(m,1)-1218.223000; 
z_pos=zeros(m,1)+242.834000;

output=[spacer x_pos y_pos z_pos pitch yaw roll];
writematrix(output,'attitude_inputs_lissajous.csv')

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
    
    n = sample_arrays(:,6)*5 +5;
    m1 = sample_arrays(:,7)*5 +5;
    
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
