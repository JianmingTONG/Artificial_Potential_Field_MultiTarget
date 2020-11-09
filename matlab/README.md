# This repo contains matlab version of Multi-robot Multi-target Potential Field
MMPF: Cluster detected targets into clusters and only keep the center of each cluster -- Corresponding infoGain is the number of targets of the cluster. Under this case, the mmpf automatically chooses the cluster with the best scores.

# Features
1. Potential function of targets is ```k*Q/r```
- Q is information gain which equals to the number of frontiers in eacy cluster. A self-proposed cluster method is proposed to cluster all frontiers into different clusters based on the layout feature. Specifically, frontiers belonging to a cluster tend to lie together into a continunous line or curver. Thus all frontiers lying in the same continous line or curver will be clustered into the same group with the center of it selected as the centroid. Only potential of the centroid of each cluster needs to be calculated. 
- r is the Hamilton Distance (a.k.a. city blocks distance), which contains information of all obstacles and thus eliminating the calculation of obstacles' potentials.
- k is the coefficience which is used to balance potential of targets with potential of multiple robots.
2. Potential of robots is ``` k(d_{sensor} - r) ```
- ``` d_{sensor} ``` is the range of lidar sensor.
- ```r``` is the coordinate difference between robots.
- Such potential is set to repel different robots different places.  
