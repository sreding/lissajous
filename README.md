# lissajous
Basic angle generation in Matlab and Python using 3D Lisssajous figures. Includes:
- Matlab file to generate static and dynamic trajectory generation including wind tunnel values as a random walk between 50% and 100%
- Python source code to generate trajectories
- Jupyter notebook example to show python usage

## Matlab
All the matlab code is contained in a single file, `lissajous.m`. The code is currently set up to generate angles for a drone experiment.
#### Note on static trajectories:
They are sill under development and should not be used currently.
## Python
Python code is in `src/generator.py`. Scipy is required for LHS sampling and numpy provides the basic data structures.
Matplotlib is required only for visualization in the notebook. To install everything in a new conda environment, run
`conda env create --file envname.yml`.
### Dynamic Trajectories
Dynamic trajectories can be created with the `DynamicTrajectories` class. See comments for more detail on how to instantiate the class.
### Static Trajectories