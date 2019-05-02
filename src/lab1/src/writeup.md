Title         : Lab1 Writeup

Doc class     : [10pt]article
Heading Depth : 0

[TITLE]

# 6.1 Motion Model

* **What assumptions is this model making?**

    The kinematic car model assumes no acceleration and the wheel rolls on flat ground without slipping.

* **Why are so many particles in a 10cm radius of the groud truth for test 1 as opposed to test 2/3?**

# 6.2 Sensor Model

* **What are the drawbacks of viewing each ray as conditionally independent?**

* **What assumptions is this model making?**

* **Why does the hallway heatmap not converge on one point?**

    Because the hallway has less distinct features to distinguish laser readings from different locations. Many spots on the hallway have similar laser scan and thus about the same probability of being the ground truth.

# 6.3 Re-Sampler

* **Why do the samples close to k get sampled more?**

    Because the samples close to k are assigned higher probability masses, they correspond to larger proportion in the rectangle as in the visualization of low-variance re-sampling. Thus, they cover more points of sampling (where the arrows point to).


# 6.4 Particle Filter

* **Where does the particle filter do well? Where does it fail? Why?**
