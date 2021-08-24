%% Parameter Definition
% Number of points per trajectory:
clear all
m = 500;

% max/min angles:
min_roll = -20;
max_roll = 20;
min_pitch = -5;
max_pitch = 15;
min_yaw = -10;
max_yaw = 10;

rng(4281529)

[roll, pitch, yaw] = generate_angles(m, min_roll, max_roll, min_pitch, max_pitch, min_yaw, max_yaw);

%% Functions
function [roll, pitch, yaw] = generate_angles(m, min_roll, max_roll, min_pitch, max_pitch, min_yaw, max_yaw)
    sample_arrays = LHS_sampling(m, 5, m);
    t = linspace(-pi, pi, m);

    a = sample_arrays(1,:);
    b = sample_arrays(2,:);
    c = sample_arrays(3,:);

    delta = pi./sample_arrays(4,:);
    phi = pi./sample_arrays(5,:);
    
    roll = standardize_to_vals(sin(a.*t + delta), min_roll, max_roll);
    pitch = standardize_to_vals(sin(b.*t), min_pitch, max_pitch);
    yaw = standardize_to_vals(sin(c.*t+phi), min_yaw, max_yaw);
end

function x = LHS_sampling(N, dims, max_val)
    a = linspace(1,max_val, N*dims);
    a = a(randperm(length(a)));
    
    x = reshape(a, dims,[]);
    
end

function x = standardize_to_vals(x, min_val, max_val)
    x = x - min(x);
    x = x * (1/max(x));
    x = x * (max_val - min_val);
    x = x + min_val;
    
end 






