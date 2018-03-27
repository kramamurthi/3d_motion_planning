import argparse
import time
import msgpack
from enum import Enum, auto

import numpy as np
import re

from planning_utils import a_star, heuristic, create_grid, prune_path
from planning_utils import create_grid_and_edges, extract_graph_start_goal, a_star_g
from udacidrone import Drone
from udacidrone.connection import MavlinkConnection
from udacidrone.messaging import MsgID
from udacidrone.frame_utils import global_to_local


class States(Enum):
    MANUAL = auto()
    ARMING = auto()
    TAKEOFF = auto()
    WAYPOINT = auto()
    LANDING = auto()
    DISARMING = auto()
    PLANNING = auto()


class MotionPlanning(Drone):

    def __init__(self, connection):
        super().__init__(connection)

        self.target_position = np.array([0.0, 0.0, 0.0])
        self.waypoints = []
        self.in_mission = True
        self.check_state = {}

        # initial state
        self.flight_state = States.MANUAL

        # register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

    def local_position_callback(self):
        if self.flight_state == States.TAKEOFF:
            if -1.0 * self.local_position[2] > 0.95 * self.target_position[2]:
                # Add protection for empty paths
                if len(self.waypoints) > 0:
                    self.waypoint_transition()
                else:
                    self.landing_transition()
        elif self.flight_state == States.WAYPOINT:
            if np.linalg.norm(self.target_position[0:2] - self.local_position[0:2]) < 1.0:
                if len(self.waypoints) > 0:
                    self.waypoint_transition()
                else:
                    if np.linalg.norm(self.local_velocity[0:2]) < 1.0:
                        self.landing_transition()

    def velocity_callback(self):
        if self.flight_state == States.LANDING:
            if self.global_position[2] - self.global_home[2] < 0.1:
                if abs(self.local_position[2]) < 0.01:
                    self.disarming_transition()

    def state_callback(self):
        if self.in_mission:
            if self.flight_state == States.MANUAL:
                self.arming_transition()
            elif self.flight_state == States.ARMING:
                if self.armed:
                    self.plan_path()                 
                    self.flight_state == States.DISARMING
            elif self.flight_state == States.PLANNING:
                self.takeoff_transition()
            elif self.flight_state == States.DISARMING:
                if ~self.armed & ~self.guided:
                    self.manual_transition()

    def arming_transition(self):
        self.flight_state = States.ARMING
        print("arming transition")
        self.arm()
        self.take_control()

    def takeoff_transition(self):
        self.flight_state = States.TAKEOFF
        print("takeoff transition")
        print("self.target_position")
        self.takeoff(self.target_position[2])

    def waypoint_transition(self):
        self.flight_state = States.WAYPOINT
        print("waypoint transition")
        self.target_position = self.waypoints.pop(0)
        print('target position', self.target_position)
        self.cmd_position(self.target_position[0], self.target_position[1], self.target_position[2], self.target_position[3])

    def landing_transition(self):
        self.flight_state = States.LANDING
        print("landing transition")
        self.land()

    def disarming_transition(self):
        self.flight_state = States.DISARMING
        print("disarm transition")
        self.disarm()
        self.release_control()

    def manual_transition(self):
        self.flight_state = States.MANUAL
        print("manual transition")
        self.stop()
        self.in_mission = False

    def send_waypoints(self):
        print("Sending waypoints to simulator ...")
        data = msgpack.dumps(self.waypoints)
        self.connection._master.write(data)

    def plan_path(self):
        self.flight_state = States.PLANNING
        print("Searching for a path ...")
        TARGET_ALTITUDE = 10
        SAFETY_DISTANCE = 5

        self.target_position[2] = TARGET_ALTITUDE

        with open('colliders.csv', 'r') as f:
            header = f.readline()
        
        home_lat = np.float64(re.search('lat0 (-?\d+\.\d+)', header).group(1))
        home_lon = np.float64(re.search('lon0 (-?\d+\.\d+)', header).group(1))

        print("Setting Global Home from colliders.csv to latitude = {}, longitude = {}".format(home_lat, home_lon))
        self.set_home_position(home_lon,
                               home_lat,
                               0.0)

        global_pos =  self.global_position
        local_pos =  global_to_local(global_pos, global_home=self.global_home)
        
        
        print('global home {0}, position {1}, local position {2}'.format(self.global_home, self.global_position,
                                                                         self.local_position))
        # Read in obstacle map
        data = np.loadtxt('colliders.csv', delimiter=',', dtype='Float64', skiprows=2)
        

        # In case there is no path possible, adjust the target altitude.
        # Also, if someone manually parked drone on top of a building, make sure to raise the target altitude
        TARGET_ALTITUDE = max(TARGET_ALTITUDE, int(5*np.ceil(self.global_position[2]/5)))
        path = []
        while len(path) == 0:

            # Define a grid for a particular altitude and safety margin around obstacles
            print('Extracting Grid and admissable Vornoi Edges for Altitude = {}'.format(TARGET_ALTITUDE))
            grid, north_offset, east_offset, edges = create_grid_and_edges(data, TARGET_ALTITUDE, SAFETY_DISTANCE)
            print('Done !')
            
            print("North offset = {0}, east offset = {1}".format(north_offset, east_offset))
            # Define starting point on the grid as the current drone position.
            N,E,D = self.local_position
            grid_start = (int(N-north_offset), int(E-east_offset))
            
            # Set goal as some arbitrary position on the grid
            grid_goal = (grid_start[0] + np.random.randint(50),
                         grid_start[1] + np.random.randint(50))
            # Will use second line in colliders.csv to set goal.
            target_lat = np.float64(re.search('lat1 (-?\d+\.\d+)', header).group(1))
            target_lon = np.float64(re.search('lon1 (-?\d+\.\d+)', header).group(1))
            target_pos = global_to_local(np.array([target_lon, target_lat, TARGET_ALTITUDE]), global_home=self.global_home)
            print('target_pos =', target_pos)
            N,E,D = target_pos
            grid_goal = (int(N-north_offset), int(E-east_offset))

            # Run A* to find a path from start to goal using a Graph approach
            print('Local Start and Goal: ', grid_start, grid_goal)
            #path, _ = a_star(grid, heuristic, grid_start, grid_goal)

            # Extract a graph representation from Edges and find the closest start and goal to nodes
            G, start_node, goal_node = extract_graph_start_goal(edges, grid_start, grid_goal)
            
            # Run Graph A*
            path, cost = a_star_g(G, heuristic, start_node, goal_node)

            if len(path) > 0:
                print('Found a path of length = ', len(path))
            else:
                print('Trying a 5m higher altitude')
                TARGET_ALTITUDE += 5
         
        #prune path to minimize number of waypoints
        t0 = time.time()
        pruned_path = prune_path(path, grid, use_bresenham=True) #Use Bresenham
        print('Reduced Path length to = ', len(pruned_path))
        print('Pruned Path: ', pruned_path)
        print('Time taken to Prune =', time.time()-t0)

        # TODO (if you're feeling ambitious): Try a different approach altogether!

        # Convert path to waypoints
        waypoints = [[int(p[0]) + north_offset, int(p[1]) + east_offset, TARGET_ALTITUDE, 0] for p in pruned_path]
        # Set self.waypoints
        self.waypoints = waypoints
        print('Way Points: ', waypoints)
        self.send_waypoints()

    def start(self):
        self.start_log("Logs", "NavLog.txt")

        print("starting connection")
        self.connection.start()

        # Only required if they do threaded
        # while self.in_mission:
        #    pass

        self.stop_log()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    args = parser.parse_args()

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), timeout=60)
    drone = MotionPlanning(conn)
    time.sleep(1)

    drone.start()
