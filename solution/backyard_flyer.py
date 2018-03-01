import argparse
import time
from enum import Enum

import numpy as np

import utm

from udacidrone import Drone
from udacidrone.connection import MavlinkConnection, WebSocketConnection  # noqa: F401
from udacidrone.messaging import MsgID


class States(Enum):
    MANUAL = 0
    ARMING = 1
    TAKEOFF = 2
    WAYPOINT = 3
    LANDING = 4
    DISARMING = 5


class BackyardFlyer(Drone):

    def __init__(self, connection):
        super().__init__(connection)
        self.target_position = np.array([0.0, 0.0, 0.0])
        self.all_waypoints = []
        self.in_mission = True
        self.check_state = {}
        self.current_waypoint = -1
        self.distance_threshold = 0.05

        # initial state
        self.flight_state = States.MANUAL

        # TODO: Register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

        self.calculate_box()

        print(self.all_waypoints)

    def local_position_callback(self):
        """
        TODO: Implement this method

        This triggers when `MsgID.LOCAL_POSITION` is received and self.local_position contains new data
        """
        if self.flight_state == States.TAKEOFF:
            # coordinate conversion
            altitude = -1.0 * self.local_position[2]

            if altitude >= 0.95 * self.target_position[2]:
                self.waypoint_transition()
        elif self.flight_state == States.WAYPOINT:
            """
            If you wish to use the code below to determine whether landing should occur, uncomment the code and
            indent the self.waypoint_transition() statement so it is part of the else logic. Also, follow the
            instructions in the waypoint_transition function.
            """
            # check if altitude is within 99% of target
            # print("Global Home: {}".format(self.global_home))
            # if self.current_waypoint == 4 and \
            #         ((self.local_position[0] < 0 and self.local_position[0] - self.all_waypoints[self.current_waypoint][0] >= -self.distance_threshold) or \
            #          (self.local_position[0] >= 0 and self.local_position[0] - self.all_waypoints[self.current_waypoint][0] <= self.distance_threshold)) and \
            #         ((self.local_position[1] < 0 and self.local_position[1] - self.all_waypoints[self.current_waypoint][1] >= -self.distance_threshold) or \
            #           (self.local_position[1] >= 0 and self.local_position[1] - self.all_waypoints[self.current_waypoint][1] <= self.distance_threshold)):
            #     print("Landing Waypoint: {}".format(self.local_position))
            #     self.landing_transition()
            # else:
            self.waypoint_transition()

    def velocity_callback(self):
        """
        TODO: Implement this method

        This triggers when `MsgID.LOCAL_VELOCITY` is received and self.local_velocity contains new data

        NOTE: We need to determine when it is appropriate to disarm the drone so as not to have it
              crash to the ground. Removed the transition from the state_callback function and
              add this logic to determine when we are close enough to the ground to release control.
        """
        if self.flight_state == States.LANDING:
            if ((self.global_position[2] - self.global_home[2] < self.distance_threshold) and
                    abs(self.local_position[2]) < self.distance_threshold):
                self.disarming_transition()

    def state_callback(self):
        """
        TODO: Implement this method

        This triggers when `MsgID.STATE` is received and self.armed and self.guided contain new data
        """
        if not self.in_mission:
            return
        if self.flight_state == States.MANUAL:
            self.arming_transition()
        elif self.flight_state == States.ARMING:
            self.takeoff_transition()
        elif self.flight_state == States.DISARMING:
            self.manual_transition()

    def calculate_box(self):
        """TODO: Fill out this method
        
        1. Return waypoints to fly a box
        """
        self.all_waypoints.append([self.local_position[0], self.local_position[1]])
        self.all_waypoints.append([10.0, 0.0])
        self.all_waypoints.append([10.0, 10.0])
        self.all_waypoints.append([0.0, 10.0])
        self.all_waypoints.append([self.local_position[0], self.local_position[1]])

    def arming_transition(self):
        """TODO: Fill out this method
        
        1. Take control of the drone
        2. Pass an arming command
        3. Set the home location to current position
        4. Transition to the ARMING state
        """
        print("arming transition")
        self.take_control()
        self.arm()

        self.flight_state = States.ARMING

    def takeoff_transition(self):
        """TODO: Fill out this method
        
        1. Set target_position altitude to 3.0m
        2. Command a takeoff to 3.0m
        3. Transition to the TAKEOFF state
        """
        print("takeoff transition")
        target_altitude = 3.0

        self.target_position[0] = self.local_position[0]
        self.target_position[1] = self.local_position[1]
        self.target_position[2] = target_altitude

        self.current_waypoint = 0

        self.takeoff(target_altitude)
        self.flight_state = States.TAKEOFF

    def waypoint_transition(self):
        """TODO: Fill out this method
    
        1. Command the next waypoint position
        2. Transition to WAYPOINT state
        """
        print("waypoint transition")

        print("Local Position: {}".format(self.local_position))
        print("Global Position: {}".format(self.global_position))
        print("Current Waypoint: {}".format(self.current_waypoint))

        """
        Need to validate we are within the distance threshold of the current waypoint (target position) before
        moving to the next waypoint. Until then, we'll keep issuing the cmd_position to get there.
        
        NOTE: There may be a better way but based on my understanding of the API, it was easier to calculate the
              difference between the desired position and the current local position, and, therefore, I had to
              adjust based on +/- positions.
              
              I also decided to perform the landing transition here. There is code that allows it to occur in the
              local_position_callback but since I'm already doing the calculations to update the current_waypoint
              value, it made sense to minimize the code. This still could be cleaner.
        """
        if ((self.local_position[0] < 0 and self.local_position[0] - self.all_waypoints[self.current_waypoint][0] >= -self.distance_threshold) or \
            (self.local_position[0] >= 0 and ((self.all_waypoints[self.current_waypoint][0] > 0 and \
                                               self.all_waypoints[self.current_waypoint][0] - self.local_position[0] <= self.distance_threshold) or \
                                              (self.all_waypoints[self.current_waypoint][0] <= 0 and \
                                               self.local_position[0] - self.all_waypoints[self.current_waypoint][0] <= self.distance_threshold)))) and \
            ((self.local_position[1] < 0 and self.local_position[1] - self.all_waypoints[self.current_waypoint][1] >= -self.distance_threshold) or \
             (self.local_position[1] >= 0 and ((self.all_waypoints[self.current_waypoint][1] > 0 and \
                                                self.all_waypoints[self.current_waypoint][1] - self.local_position[1] <= self.distance_threshold) or \
                                               (self.all_waypoints[self.current_waypoint][1] <= 0 and \
                                                self.local_position[1] - self.all_waypoints[self.current_waypoint][1] <= self.distance_threshold)))):
            print("Updating Waypoint")
            self.current_waypoint += 1

            if self.current_waypoint <= 4:
                self.target_position[0] = self.all_waypoints[self.current_waypoint][0]
                self.target_position[1] = self.all_waypoints[self.current_waypoint][1]

            print("Target Position: {}".format(self.target_position))

        """
        If you wish to use the landing logic in local_position_callback, comment the if/else below and uncomment the
        next line here. Remember, indents matter and should be at the same level as the if statement.
        
        self.cmd_position(self.target_position[0],
                          self.target_position[1],
                          self.target_position[2],
                          0)

        self.flight_state = States.WAYPOINT
        """
        if self.current_waypoint > 4:
            self.landing_transition()
        else:
            self.cmd_position(self.target_position[0],
                              self.target_position[1],
                              self.target_position[2],
                              0)

            self.flight_state = States.WAYPOINT

    def landing_transition(self):
        """TODO: Fill out this method
        
        1. Command the drone to land
        2. Transition to the LANDING state
        """
        print("landing transition")

        self.land()
        self.flight_state = States.LANDING

    def disarming_transition(self):
        """TODO: Fill out this method
        
        1. Command the drone to disarm
        2. Transition to the DISARMING state
        """
        print("disarm transition")
        print("Local Position: {}".format(self.local_position))

        self.disarm()
        self.flight_state = States.DISARMING

    def manual_transition(self):
        """This method is provided
        
        1. Release control of the drone
        2. Stop the connection (and telemetry log)
        3. End the mission
        4. Transition to the MANUAL state
        """
        print("manual transition")

        self.release_control()
        self.stop()
        self.in_mission = False
        self.flight_state = States.MANUAL

    def start(self):
        """This method is provided
        
        1. Open a log file
        2. Start the drone connection
        3. Close the log file
        """
        print("Creating log file")
        self.start_log("Logs", "NavLog.txt")
        print("starting connection")
        self.connection.start()
        print("Closing log file")
        self.stop_log()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    args = parser.parse_args()

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), threaded=False, PX4=False)
    #conn = WebSocketConnection('ws://{0}:{1}'.format(args.host, args.port))
    drone = BackyardFlyer(conn)
    time.sleep(2)
    drone.start()
