# Particle Filter Implementation For Localization

In robotics and autonomous systems, localization is the problem of figuring out where on a known map the robot is currently located. It is important to be able to do this because the robot cannot always keep track of where it is. For example, if the robot is picked up and placed down somewhere else, or if the robot is turned off and transported to a new location, it won't be able to infer where it is based on the movements it performed alone.

A particle filter is used to address this problem by simulating robots ("particles") at random places on the map and comparing what these particles would see against what the real robot is seeing. It then weights the random simulations by how similar its sensor observations are to those of the real robot. Then the particles are sampled with replacement based on the weights and then the process is repeated, with each particle being reweighted. When all the particles converge to roughly the same spot, that is our estimate of where the robot is.


To run the particle filter simulation,  run ```python3 pf_gui.py``` in the root directory.

![alt text](https://github.com/zhangj150/particle-filter-localization/blob/master/unconverged.png "unconverged")
