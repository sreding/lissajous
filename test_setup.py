import pandas as pd
import numpy as np
from src import generator

SEED = 40960242
np.random.seed(SEED)

# Number of trajectories:
number_of_trajectories = 8
# Number of points per trajectory:
number_of_points = 300

# average time spend at one point
avg_freq = 0.1


min_torque_x = -12
max_torque_x = 12
min_torque_y = -12
max_torque_y = 12
min_torque_z = -12
max_torque_z = 12

min_force_x = -100
max_force_x = 100
min_force_y = -100
max_force_y = 100
min_force_z = -100
max_force_z = 100

SEED_torque = 429471
SEED_force = 7234

torque = generator.DynamicTrajectory(min_torque_x, max_torque_x, min_torque_y, max_torque_y, min_torque_z, max_torque_z,seed=SEED_torque,sequence_len=number_of_points)
force = generator.DynamicTrajectory(min_force_x, max_force_x, min_force_y, max_force_y, min_force_z, max_force_z,seed=SEED_force,sequence_len=number_of_points)
torque.frequency=avg_freq

for i in range(number_of_trajectories):
    t = torque.generate_next()
    f = force.generate_next()
    force_torque_traj = np.hstack((f,t))
    pd.DataFrame(force_torque_traj, columns=['force_x','force_y', 'force_z', 'torque_x', 'torque_y', 'torque_z','time']).to_csv('dynamic_v1.0_'+str(i)+'.csv')

torque = generator.StaticTrajectory(min_torque_x, max_torque_x, min_torque_y, max_torque_y, min_torque_z, max_torque_z,seed=SEED_torque,sequence_len=number_of_points)
force = generator.StaticTrajectory(min_force_x, max_force_x, min_force_y, max_force_y, min_force_z, max_force_z,seed=SEED_force,sequence_len=number_of_points)
torque.frequency=avg_freq

for i in range(number_of_trajectories):
    t = torque.generate_next()
    f = force.generate_next()
    force_torque_traj = np.hstack((f,t))
    pd.DataFrame(force_torque_traj, columns=['force_x','force_y', 'force_z', 'torque_x', 'torque_y', 'torque_z','time']).to_csv('static_v1.0_'+str(i)+'.csv')