# This repo stores all matlab version Aritificial Potential Field

Three Versions are avaliable:
APF:
Use original euclidean distance and kq/(r^2)

MRPF :Cluster detected targets into clusters and only keep the center of each cluster -- Corresponding infoGain is the number of targets of the cluster. Under this case, the apf turns to find the center point of a cluster of frontiers.
Speed: Version 3 < Version 1 < Version 2
Version 1: Maintain all distance maps of all detected targets. The APF turns to find the nearest unexplored frontier.
Version 2: Cluster detected targets into clusters and only keep the center of each cluster -- Corresponding infoGain is the number of targets of the cluster. Under this case, the apf turns to find the center point of a cluster of frontiers.
Version 3: Maintain distance from the current place (0 at curr and expand to all free places based on wave-front)

# The following versions are also tested in this repo.
1. Use eluclidean distance as the "r" --- increase the possibility of getting into local minimum. (see files computNewPotentialMultiGoal_elu.m)
2. Currently Hamilton (a.k.a. city blocks distance) has been used here to calculate obstacles since only obstacles which are near current place contribute to the potentials. Under this case, hamilton distance is always the sqrt(2) times of eluclidean distance, but significantly reduce the overall calcualtion.
3. Using distance generated from wave-front (start at targets) to measure the distance between targets and current places. --- Solve local minimum.
