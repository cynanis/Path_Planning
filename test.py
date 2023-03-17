from src.sampling_based_planners.rrt import RRT
from src.sampling_based_planners.rrt_star import RRTStar
from src.sampling_based_planners.informed_rrt_star import InformedRRTStar
from src.map import Map
import argparse
import numpy as np
import cv2 as cv




parser = argparse.ArgumentParser(description='Process some integers.')
parser.add_argument('-t', "--type", type=str, default='rrt',
                    help='rrt or rrt_star')
parser.add_argument('-r', "--raduis", type=int, default=50,
                    help='initial search raduis')
args = parser.parse_args()

# Initializing surface
map_size = (330,340,3)

#draw star and goal points
q_start={"x":100,"y":35,"theta":0.3,"delta":0.1,"beta":0.01}
q_goal={"x":250,"y":250,"theta":0,"delta":0,"beta":0}

map = Map(size = map_size, q_start= q_start, q_goal=q_goal,
                        obstacles_color=(0, 255, 0), end_points_colors= (255, 0, 255), 
                        tree_color = (0, 0, 0), path_color = (255, 0, 0))

# Initializing RTT
rrt = None
if args.type == "rrt":
    rrt = RRT(map=map, goal_threshold=7,name="RRT")
elif args.type == "rrt_star":
    rrt = RRTStar(map=map,goal_threshold=7,rewire_radius=args.raduis,name="RRT*")
elif args.type == "informed_rrt_star":
    rrt = InformedRRTStar(map=map,goal_threshold=7,rewire_radius=args.raduis,name="Informed RRT*")
else:
    print("incorrect algorithm name")
    exit()
#Build RRT
rrt.build(int(1e6))

input('Press ENTER to exit')