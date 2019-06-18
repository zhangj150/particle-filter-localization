# Particle Filter Implementation For Localization

In robotics and autonomous systems, localization is the problem of figuring out where on a known map the robot is currently located. It is important to be able to do this because the robot cannot always keep track of where it is. For example, if the robot is picked up and placed down somewhere else, or if the robot is turned off and transported to a new location, it won't be able to infer where it is based on the movements it performed alone.

A particle filter is used to address this problem by simulating robots ("particles") at random places on the map and comparing what these particles would see against what the real robot is seeing. It then weights the particles by how similar its sensor observations are to those of the real robot. Then the particles are sampled with replacement based on the weights and then the process is repeated, with each particle being reweighted. When all the particles converge to roughly the same spot, that is our estimate of where the robot is.

#### There are two key parts to the algorithm, motion update and measurement update.

Motion update occurs when the robot moves its motors to get around the map. All the particles will perform the same motor inputs.

Measurement update occurs when the robot compares its sensor readings to those of the simulated particles. It then reweights the probability of each particle being the real robot location by taking the difference between the actual sensor data and the sensor data of the simulated particle and applying a probability distribution, in this case a Gaussian. After that, if there were say 100 particles, we would resample 100 times with replacement from the current particle locations, where the probability of each particle being selected is its weight. That way, particles that have small weight are less likely to be included in the next iteration.

Motion updates must keep occuring to give the robot and its particles new observations to compare against.

To run the particle filter simulation,  run ```python3 pf_gui.py``` in the root directory.

After a few iterations, the particles have not yet agreed on a position. The red dots are the particles, with their attached lines showing their orientation. The blue squares on the edges of the grid are the landmarks used for (visual) observations. 

![alt text](https://github.com/zhangj150/particle-filter-localization/blob/master/unconverged.png "unconverged")

The estimated robot location is the average position of all the particles. When all the particles converge to roughly the same spot, the estimated location is close to the ground truth location of the robot.

![alt text](https://github.com/zhangj150/particle-filter-localization/blob/master/converged.png "converged")
