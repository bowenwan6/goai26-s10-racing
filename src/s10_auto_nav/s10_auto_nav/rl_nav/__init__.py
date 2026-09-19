"""RL route navigation for the S10: the walking actor (official slot) and the stairs actor
(stairs_stable slot) driven along route_v2.

Everything here is ROS-free and runs identically in the MuJoCo harness and on the robot:

Offline, on the map (``prepare``):

* ``route_prep``    centre the route on the structure it crosses, re-plan where it crosses a hazard
* ``maneuvers``     where along the route the stairs actor is needed
* ``capability``    what each policy can take (the numbers the two steps above use)

On the robot:

* ``route_runner``  who drives and with what command: the first version's controller, plus recovery
                    that engages only on a measured deviation
* ``map_check``     is what the LiDAR sees terrain the map knows, or something new?
* ``map_planner``   A* on the map from where the robot is back onto the route
* ``edge_tracker``  step edges from the 13x9 height grid (a field-review tool; not in the loop)
"""
