"""RL route navigation for the S10: the walking actor (official slot) and the stairs actor
(stairs_stable slot) driven along route_v2 with perception-gated policy switches.

Everything here is ROS-free and runs identically in the MuJoCo harness and on the robot:

* ``edge_tracker``     step edges from the 13x9 height grid (RANSAC line), fused with a map prior
* ``maneuvers``        offline: where along the route the stairs actor is needed, with edge priors
* ``local_astar``      A* on the follower's local grid to get back onto the route
* ``maneuver_router``  the state machine that owns the command and the policy request
"""
