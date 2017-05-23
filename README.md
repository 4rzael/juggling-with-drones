# juggling-with-drones
Juggling with drones. An experiment for a juggling performance


## Wat ?

This repository contains the code done by the Maxime Agor, in the team POTIOC, from INRIA Bordeaux, for a part of the spectacle named "Les spheres curieuses", by Antoine Clee

The objective is to be able to "juggle" with drones instead of balls, removing limitations given by the laws of nature (gravity, for example), and adding... AERIAL CRASHES ! MWAHAHAHAHAHA


## Prerequisites

* Linux environment (tested on Ubuntu )

* ROS kinetic installed

* numpy, sklearn, rospy installed

* crazyflie_ros installed

* razer_hydra package for ROS installed

* having crazyflie drones (and loco positioning systems), duh


## Code architecture

* juggling_swarm_manager : The orchestra master, controlling the small swarm of drones (I named it swarmy, because it's cute)

* juggling_controller : Handles the razer hydra / glove and the way the artist is going to interact with swarmy

* crazyflie_trajectory_manager : Handles the trajectories for a single drone

* juggling_demo : Unify all above packages into one launch file


## Wiki / API doc ?

* Maybe... Someday... **runs away**


## Notes :

* This code should require [only minimal configuration and tweaking](https://xkcd.com/1742/) in order to make it work
