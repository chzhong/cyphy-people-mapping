This ros stack contains packages to:
(a) follow people using a rgbd camera and the openni framework
(b) cluster the trajectories obtained while following people online into a skeletal representation of all trajectories
(c) engineer a map from the clustered trajectories that better reflects the way humans operate in the environment and can thus better inform a robot operating in the same environment.  The robot is able to respond to changes in the environment (like spilled water on the floor) that people easily detect and navigate around, but are difficult for a robot to pick up on using its onboard sensors.