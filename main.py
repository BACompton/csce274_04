#!/usr/bin/python

import datetime
import math
import random
import sys
import threading
import time

import robot_inf
import serial_inf
import sensor_inf


# =============================================================================
#                       Miscellaneous Helper Functions/Definitions
# =============================================================================

class Behavior():
    """
        Behavior helps identify and label the different behavior modules
        spawned to generate the potential actuator commands.

        Available Behaviors:
            OBSTACLE_AVOIDANCE
            WALL_FOLLOW
            DOCK

        If a behavior does not match any of the available behaviors, then a
        behavior of UNKNOWN will be used.

        :type _action Action
        :type _action_lock threading.Lock
    """
    UNKNOWN = -1
    WALL_FOLLOW = 0
    DOCK = 1
    OBSTACLE_AVOIDANCE = 2

    _behavior = None
    _action = None
    _action_lock = None

    def __init__(self, behavior):
        """
            Constructor that sets the behavior to the provided value
        :param behavior:
            The desired behavior. This should be a available behavior
            constant from Behavior (Ex: Behavior.DOCK)
        """
        self._set_behavior(behavior)
        self._action_lock = threading.Lock()

    def _set_behavior(self, behavior):
        """
            Sets the provided behavior only if it is an available behavior.
            When the provided
        :param behavior:
            The desired behavior. This should be a available behavior
            constant from Behavior (Ex: Behavior.DOCK)
        """
        if Behavior.is_behavior(behavior):
            self._behavior = behavior

    def get_behavior(self):
        """
            Either returns the module's associated behavior or UNKNOWN
            if no matching available behavior exist.

            A simple application for the return value is the priority of the
            behavior's action.
        :return:
            Any available behavior or Behavior.UNKNOWN
        """
        return self._behavior

    def _set_action(self, action=None):
        """
            Sets the action for the behavior.
        :type action Action
        :param action:
            The generated action from this behavior.
        """
        self._action_lock.acquire()         # Acquire Lock
        self._action = action
        self._action_lock.release()         # Release Lock


    def has_action(self):
        """
            Determines if the behavior generated an actuator action.
        :return:
            True if it has an action. Otherwise, false.
        """
        self._action_lock.acquire()         # Acquire Lock
        rtn = self._action is not None
        self._action_lock.release()         # Release Lock

        return rtn

    def get_action(self):
        """
            Acquires the generated actuator action from the behavior.
        :return:
            The current actuator action
        """
        self._action_lock.acquire()         # Acquire Lock
        rtn = self._action
        self._action_lock.release()         # Release Lock

        return rtn

    @staticmethod
    def execute_action(robot, action):
        """
            Executes the specified actuator action on the provided robot.
        :type robot robot_inf.Robot
        :param robot:
            The robot to preform the action on
        :type action Action
        :param action:
            The actuator action to preform
        """
        if action.type == Action.STOP_MODULES \
                or Action.type == Action.TERMINATE:
            robot.drive_direct(0, 0)

        elif action.type == Action.DRIVE_DIRECT:
            if len(action.params) >= Action.DRIVE_DIRECT:
                robot.drive_direct(action.params[0], action.params[1])

    @staticmethod
    def log_action(log_file, sensor, action):
        """
            Adds a log statement based on the provided action.
        :type log_file LogFile
        :param log_file:
            The log file to add a statement too.
        :type sensor sensor_inf.Sensor
        :param sensor:
            The sensor interface for the robot
        :type action Action
        :param action:
            The action that needs to be logged
        :return:
        """
        if action.type == Action.DRIVE_DIRECT:
            #TODO finish need to add a reference
            t = 0


    @staticmethod
    def behavior_list():
        """
            Constructs a list of available behaviors
        :return:
            The list of available behaviors
        """
        return [
            Behavior.OBSTACLE_AVOIDANCE,
            Behavior.WALL_FOLLOW,
            Behavior.DOCK
        ]

    @staticmethod
    def is_behavior(behavior):
        """
            Determines if the provided behavior is an available behavior.
        :param behavior:
            The behavior to check.
        :return:
            True if the behavior matches an available behavior.
            Otherwise, false.
        """
        for b in Behavior.behavior_list():
            if b == behavior:
                return True
        return False


class Action():
    """
        Action represents any actuator action. Each action is constructed from
        an action type and a list of parameters.

        The value of a action type represents the required number of parameters.
    :type params list
    :type exec_time double
    :type event threading.Event
    """
    UNKNOWN = -1
    DRIVE_DIRECT = 2
    STOP_MODULES = 0
    TERMINATE = 0

    type = None
    params = None
    event = None
    exec_time = None
    log = False

    def __init__(self, type, params=(), time=robot_inf.SENSOR_UPDATE_WAIT):
        """
            Constructs an action of the specified type with the provided
             parameters
        :param type:
            The type of actuator action. This value should be set to available
            action within Action
        :param params:
            The list of parameters to use with the specified action type.
        :param time
            The amount of time that the action should run
        """
        self.type = type
        self.params = params
        self.exec_time = time
        self.event = threading.Event()
        self.log = False


class LogFile():
    """
        LogFile handles and writes log statements in a thread safe manner.

    :type _file file
    :type _lock threading.Lock
    """
    # The default log file name
    DEFAULT_NAME = "robot_log.log"

    _file = None
    _lock = None

    def __init__(self, name=""):
        """
            Constructor that creates a log file with the provided name.

            If the provided name is an empty string, then a log file with
            the default name will be created instead.
        :param name:
            The desired name of the log file
        """
        self.open(name)
        self._lock = threading.Lock()

    def log_stmt(self, datum):
        """
            Creates a log statement in the supplied file with the provided
            datum.
        :param datum:
            The data to log
        """

        if self._file is not None:
            self._lock.acquire()                 # Acquire Lock
            self._file.write(self._timestamp()+", "+str(datum)+"\n")
            self._lock.release()                 # Release Lock

    def open(self, name):
        """
            Opens a log file with the provided name.

            If the provided name is an empty string, then a log file with
            the default name will be created instead.
        :param name:
            The desired name of the log file
        """
        if name == "":
            name = LogFile.DEFAULT_NAME
        self._file = open(name, "w")

    def close(self):
        """
            Closes the log file.
        """
        self._file.close()
        self._file = None

    def _timestamp(self):
        """ Gets the timestamp of the local system
        :return:
            Returns the current time
        """
        return datetime.datetime.fromtimestamp(time.time())\
                       .strftime("%Y-%m-%d %H:%M:%S")


class _StoppableThread(threading.Thread):
    """
        StoppableThread extends the threading module's Thread class to include
        a method that notifies a thread to halt execution.

        The action on a stopping message is not handled within this definition.
        Instead, it should be handled within the class extending
        StoppableThread. The stopping notification can be acquired by from the
        stop data member (Ex: self._stop).

        Additionally, this class will help initialize different aspects of
        a Thread object such as the daemon attribute.

        :type _stop bool
    """

    _stop = None

    def __init__(self, daemon=True):
        """
            Constructor that sets the initial thread attributes
        """
        threading.Thread.__init__(self)
        self.setDaemon(daemon)
        self._stop = False

    def stop(self):
        """
            Notifies the thread to halt execution. The thread will
            not immediately stop. Instead, the thread will try to exit at
            the safe termination point.
        """
        self._stop = True


# The base velocity to apply to the wheels. This is the velocity of
# the typically forward movement in mm/sec.
_WHEEL_VEL = 100


# =============================================================================
#                           Behavior Modules
# =============================================================================
# TODO: Update/Add behaviors
# TODO: Add logging
class ObstacleAvoidance(_StoppableThread, Behavior):
    """
        Behavior module responsible for avoiding obstacles and dangerous
        situations such as cliffs. This module has a relatively high action
        priority to increase the likelihood that a dangerous situation and
        obstacles are avoided.

    :type _sensor sensor_inf.Sensor
    :type _log LogFile
    """

    # -------------------------------------------------------------------- #
    # -                     Internal Constants                           - #
    # -------------------------------------------------------------------- #

    # Maximum and minimum amount of time the robot can rotate when a bump
    # is pressed. Both limits are calculated from the general differential
    # drive angle formula solved for the time:
    #
    #   time  = wheel_base * angle_radians / ( 2 * wheel_velocity )
    #
    # The final rotational range was decided based on the sensor with
    # the smallest detectable range. In this instance it was the light bump
    # sensor (IR bump).
    #
    # The maximum wait time will rotate the robot roughly 15 degrees
    # The minimum wait time will rotate the robot roughly 5 degrees
    _MIN_TURN_TIME = robot_inf.WHEEL_BASE * (math.pi/12) / (2 * _WHEEL_VEL)
    _MAX_TURN_TIME = robot_inf.WHEEL_BASE * (math.pi/36) / (2 * _WHEEL_VEL)

    # -------------------------------------------------------------------- #
    # -                    Behavior Definition                           - #
    # -------------------------------------------------------------------- #

    _sensor = None
    _log = None

    def __init__(self, sensor, log):
        """
            Constructs an obstacle avoiding behavior for the provided robot.
        :param sensor:
            The sensor interface for the desired robot
        :param log:
            The robot's log file
        """
        _StoppableThread.__init__(self, daemon=True)
        Behavior.__init__(self, Behavior.OBSTACLE_AVOIDANCE)
        self._sensor = sensor
        self._log = log

    def run(self):
        while not self._stop:
            # Log the previous action
            if self.has_action():
                prev_action = self.get_action()

                if prev_action.log:
                    Behavior.log_action(self._log, self._sensor, prev_action)

            # Acquire the necessary sensor data
            bumps = self._sensor.get_bumps()
            wheel_drops = self._sensor.get_wheel_drops()
            cliffs = self._sensor.get_cliffs()

            # Process sensor data into actions
            safe_motion = self._safe_motion(wheel_drops)
            turn_cw = self._turn_cw(cliffs, bumps)
            turn_ccw = self._turn_ccw(cliffs, bumps)

            # Randomly choose a rotation direction when both bumps are down.
            if turn_ccw and turn_cw:
                turn_ccw = bool(random.getrandbits(1))
                turn_cw = not turn_ccw

            # Generate the modules action
            action = Action(Action.UNKNOWN)
            if not safe_motion:
                action = Action(Action.STOP_MODULES)
            elif turn_cw:
                action = Action(
                    Action.DRIVE_DIRECT,
                    params=[-_WHEEL_VEL, _WHEEL_VEL],
                    time=random.uniform(ObstacleAvoidance._MIN_TURN_TIME,
                                        ObstacleAvoidance._MAX_TURN_TIME)
                )
            elif turn_ccw:
                action = Action(
                    Action.DRIVE_DIRECT,
                    params=[_WHEEL_VEL, -_WHEEL_VEL],
                    time=random.uniform(ObstacleAvoidance._MIN_TURN_TIME,
                                        ObstacleAvoidance._MAX_TURN_TIME)
                )

            # Issue the generated action
            if action.type != Action.UNKNOWN:
                self._set_action(action)
            action.event.wait(action.exec_time)

    # -------------------------------------------------------------------- #
    # -                      Internal Helpers                            - #
    # -------------------------------------------------------------------- #

    def _safe_motion(self, wheel_drops):
        """ Determines if movement or rotation is safe.
        :param wheel_drops:
            The value of the wheel drop sensors retrieved from an instance of
            either robot_inf.Robot or sensor_inf.Sensor.
        :return:
            True if motion or rotation is safe.
        """
        for drop in wheel_drops:
            if wheel_drops[drop]:
                return False
        return True

    def _turn_cw(self, cliffs, bumps):
        """ Determines if the robot should rotate clockwise
        :param cliffs:
            The value of the cliff sensors retrieved from an instance of either
            robot_inf.Robot or sensor_inf.Sensor.
        :param bumps:
            The value of the bump sensors retrieved from an instance of either
            robot_inf.Robot or sensor_inf.Sensor.
        :return:
            True if the robot should turn clockwise
        """
        if bumps[robot_inf.Bump.BUMP_L]:
            return True
        if cliffs[robot_inf.Cliff.CLIFF_L] or cliffs[robot_inf.Cliff.CLIFF_FL]:
            return True
        return False

    def _turn_ccw(self, cliffs, bumps):
        """ Determines if the robot should rotate counter-clockwise
        :param cliffs:
            The value of the cliff sensors retrieved from an instance of either
            robot_inf.Robot or sensor_inf.Sensor.
        :param bumps:
            The value of the bump sensors retrieved from an instance of either
            robot_inf.Robot or sensor_inf.Sensor.
        :return:
            True if the robot should turn counter-clockwise
        """
        if bumps(robot_inf.Bump.BUMP_R):
            return True
        if cliffs[robot_inf.Cliff.CLIFF_R] or cliffs[robot_inf.Cliff.CLIFF_FR]:
            return True
        return False


class WallFollow(_StoppableThread, Behavior):
    """
        This is the low level actuator controller to follow a wall with
        obstacle avoidance.

        :type _sensor sensor_inf.Sensor
        :type _log LogFile
    """

    # -------------------------------------------------------------------- #
    # -                     Internal Constants                           - #
    # -------------------------------------------------------------------- #

    # TODO: Test/adjust all set points, gains, and constants

    # The sensor value for the desired distance from the wall
    _WALL_GOAL = 250

    # The minimum distance the robot can be from a forward wall before turning
    # to the left.
    _LEFT_TURN_THRESHOLD = _WALL_GOAL - 150

    # The minimum sensor value required for a wall to be detected. This was
    # chosen based on the senor's noise.
    _WALL_THRESHOLD = 5

    # The maximum amount of cycles before a wall is considered lost. The
    # threshold's value is calculated based on the following formula:
    #
    #   num_of_cycles = max_time / (wait_time + avg_cycle_length)
    _LOST_WALL_THRESHOLD = math.ceil(15. / (robot_inf.SENSOR_UPDATE_WAIT + .25))

    # -------------------------------------------------------------------- #
    # -                    Behavior Definition                           - #
    # -------------------------------------------------------------------- #

    _sensor = None
    _log = None

    def __init__(self, sensor, log):
        _StoppableThread.__init__(self, daemon=True)
        Behavior.__init__(self, Behavior.WALL_FOLLOW)
        self._sensor = sensor
        self._log = log

    def run(self):
        # The initial points for each PID controller
        init_follow_pt = self._sensor.is_light_bump(robot_inf.Bump.LIGHT_BUMP_R)
        init_turn_pt = self._sensor.is_light_bump(robot_inf.Bump.LIGHT_BUMP_CR)

        # The wait time in between behavior decisions
        wait_time = robot_inf.SENSOR_UPDATE_WAIT

        # PID controllers for the wall following/right turn behavior (PID) and
        # the left turn behavior (PD)
        wall_follow = robot_inf.PIDController(goal=WallFollow._WALL_GOAL,
                                              init_pt=init_follow_pt)
        left_turn = robot_inf.PIDController(goal=0, kP=2.5, kI=0, kD=0,
                                            init_pt=init_turn_pt)

        # The separate gains for the wall follow PID controller.
        #   - follow_gains: The default gains for the wall following behavior
        #   - right_turn_gains: Adjust the gains to perform a right wall follow
        #
        # Instead of creating a separate PID controller for right wall following,
        # the gains are adjusted to reduce the radius of a right turn since
        # the wall following PID controller already covers right turns.
        follow_gains = {
            robot_inf.PIDController.KP_KEY: .1,
            robot_inf.PIDController.KI_KEY: .0025,
            robot_inf.PIDController.KD_KEY: .0025
        }
        right_turn_gains = {
            robot_inf.PIDController.KP_KEY: .2
        }
        wall_follow.set_gains(follow_gains)

        # A counter for the number of cycles with no wall detected.
        no_wall_count = 0

        # ------------------------------------------------------------ #
        # -                    Behavior Decision                     - #
        # ------------------------------------------------------------ #

        while not self._wait(wait_time, robot_inf.SENSOR_UPDATE_WAIT):
            # Log the previous action
            if self.has_action():
                prev_action = self.get_action()

                if prev_action.log:
                    Behavior.log_action(self._log, self._sensor, prev_action)

            left_output = left_turn.get_output()

            # Get the current points for the PID controllers
            wall_pt = self._sensor.is_light_bump(robot_inf.Bump.LIGHT_BUMP_R)
            left_pt = self._sensor.is_light_bump(robot_inf.Bump.LIGHT_BUMP_CR)

            # Decide which behavior to perform
            if left_output > WallFollow._LEFT_TURN_THRESHOLD:
                wall_follow.reset(init_pt=wall_pt)
                self._drive_ccw()
            elif no_wall_count >= WallFollow._LOST_WALL_THRESHOLD:
                no_wall_count = WallFollow._LOST_WALL_THRESHOLD
                self._drive_forward()
            else:
                self._wall_follow(wall_follow)

            # Update the PID controllers with the current point
            left_turn.add_point(left_pt)

            # If no right wall
            if wall_pt < WallFollow._WALL_THRESHOLD:
                wall_follow.set_gains(right_turn_gains)
                no_wall_count += 1
                wall_follow.reset(init_pt=wall_pt)

            # Found right wall
            else:
                wall_follow.set_gains(follow_gains)
                no_wall_count = 0
                wall_follow.add_point(wall_pt)

            self.get_action().event.wait(wait_time)

    # -------------------------------------------------------------------- #
    # -                  Behavior Definitions                            - #
    # -------------------------------------------------------------------- #

    def _drive_ccw(self):
        """
            Rotates the robot counter-clockwise.
        """
        self._set_action(Action(Action.DRIVE_DIRECT,
                                [_WHEEL_VEL, -_WHEEL_VEL]))

    def _drive_forward(self):
        """
            Drives the robot forward.
        """
        self._set_action(Action(Action.DRIVE_DIRECT,
                                [_WHEEL_VEL, _WHEEL_VEL]))

    def _wall_follow(self, wall_pid):
        """ The wall following module.

            This will follow a wall using a pid controller whose current
            position is based on an infrared sensor's reading.
        :type wall_pid robot_inf.PIDController
        :param wall_pid:
            The pid controller for the wall following and right turn behavior.
        """
        # PID controller output
        output = int(wall_pid.get_output())

        self._set_action(Action(Action.DRIVE_DIRECT,
                                [_WHEEL_VEL+output, _WHEEL_VEL-output]))


# =============================================================================
#                       High Level Controllers
# =============================================================================

class RobotController(_StoppableThread):
    """
        High level robot controller. This controller decides when to spawn and
        kill specific low level actuator controllers.

        :type _active_behaviors list
        :type _exec_action Action
        :type _log LogFile
    """

    # A list of all active behavior modules
    _active_behaviors = None
    _selected_behavior = Behavior.UNKNOWN
    _exec_action = None

    # Handel for the log file
    _log = None

    def __init__(self):
        _StoppableThread.__init__(self, daemon=True)
        self._active_behaviors = []

    # -------------------------------------------------------------------- #
    # -                        Main Behavior                             - #
    # -------------------------------------------------------------------- #

    def run(self):
        # Get available serial connections
        port_list = serial_inf.list_serial_ports()

        # Check for at least one available serial connection
        if len(port_list) > 1:
            print "Requires a serial connection."
            return -1

        # Create the Log file
        file_name = ""
        if len(sys.argv) > 1:
            file_name = sys.argv[1]
        self._log = LogFile(file_name)

        # Open serial connection to robot.
        robot = robot_inf.Robot(port_list[0])
        robot.change_state(robot_inf.State.FULL)
        print "Connected to robot"

        # Synchronized sensor interface for the serial connection to the robot
        sensor = sensor_inf.Sensor(robot)

        print "Listening for press"
        while not self._stop:
            # When the CLEAN button is pressed and the robot is stopped,
            # start all behaviors. Otherwise stop all behaviors upon a CLEAN
            # button press.
            if sensor.is_btn_pressed(robot_inf.Button.CLEAN):
                # Start all behaviors
                if len(self._active_behaviors) == 0:
                    self._log.log_stmt("Start Button Press")

                    # TODO: Initialize and start the necessary modules
                    self._active_behaviors = [
                        ObstacleAvoidance(sensor, self._log),
                        WallFollow(sensor, self._log)
                    ]
                    for module in self._active_behaviors:
                        if module is threading.Thread:
                            module.start()

                    # Order the active behaviors based on priority
                    # (Higher behavior value = Higher priority)
                    self._active_behaviors.sort(key=lambda m: m.get_behavior())
                    self._active_behaviors.reverse()

                # Stop all behaviors
                else:
                    self._log.log_stmt("Stop Button Press")
                    self._stop_behaviors(robot)

            # Notify the previously active behavior to wake up
            if self._selected_behavior != Behavior.UNKNOWN:
                self._exec_action.event.set()

            # Select the first module with an action from the priority sorted
            # list and execute that module's action
            self._selected_behavior = Behavior.UNKNOWN
            self._exec_action = Action(Action.UNKNOWN)

            for module in self._active_behaviors:
                if module.has_action():
                    action = module.get_action()

                    Behavior.execute_action(robot, action)
                    self._interpret_action(robot, module, action)
                    break

            # Preform the selected action for specified amount of time or until
            # a higher priority action arrives. The action can also be
            # interrupted by a CLEAN button press.
            self._wait(self._exec_action.exec_time,
                       robot_inf.SENSOR_UPDATE_WAIT, sensor)
        print "Stopping Listening"

        # Cleans up all threads spawned during execution.
        self._stop_behaviors(robot)
        robot.change_state(robot_inf.State.PASSIVE)
        self._log.close()
        sensor.stop_update(join=True)

    # -------------------------------------------------------------------- #
    # -                     Internal Helpers                             - #
    # -------------------------------------------------------------------- #

    def _stop_behaviors(self, robot):
        """
            Notifies all modules to halt execution and stops the robot.
        :param robot:
            The robot that should be stopped.
        """
        for i in range(len(self._active_behaviors)-1, 0, -1):
            behavior = self._active_behaviors[i]

            if behavior is _StoppableThread:
                behavior.stop()
            self._active_behaviors.remove(i)
        robot.drive_direct(0,0)

    def _wait(self, wait_time, interval, sensor):
        """ Internal method that divides the time in between actuator
            commands into small intervals. This enables the ability
            to interrupt an action's execution.

        :param wait_time:
            The total amount of time to wait
        :param interval:
            The amount of time for a single interval.
        :type sensor sensor_inf.Sensor
        :param sensor
            The handel for the sensor interface that is used to poll the
            robot's sensor data.
        """
        time_left = wait_time

        while time_left > 0:
            start = time.time()
            # Tell the caller it needs to stop
            if self._interrupt(sensor):
                return True

            # Determine the next sleep interval
            interval_time = interval
            if interval_time > time_left:
                interval_time = time_left

            # Update the amount of remaining time by subtracting the interval
            # time along with the code block's execution time.
            time_left -= (interval_time + (time.time()-start))
            time.sleep(interval_time)

    def _interrupt(self, sensor):
        """ Determines if the executing action should be interrupted.
        :type sensor sensor_inf.Sensor
        :param sensor
            The handel for the sensor interface that is used to poll the
            robot's sensor data.
        :return:
            True when the executing action should be interrupted.
            Otherwise, false.
        """
        # Check for a higher priority action
        for module in self._active_behaviors:
            if module.get_behavior() > self._selected_behavior:
                if module.has_action():
                    return True
            else:
                break

        # Check to see if the CLEAN button is pressed
        return sensor.is_btn_pressed(robot_inf.Button.CLEAN, src="Interrupt")

    def _interpret_action(self, robot, module, action):
        """
            Determines what call to make based on action type
                Action.TERMINATE: notifies to stop
                Action.STOP_MODULES: halts execution of active behavior modules
                Otherwise: Save a handle on behavior's priority and action
        :type robot robot_inf.Robot
        :param robot
            The robot that the controller belongs to.
        :type module Behavior
        :param module
            The selected behavior
        :type action Action
        :param action
            The selected action
        """
        if action.type == Action.TERMINATE:
            self.stop()
        elif action.type == Action.STOP_MODULES:
            self._stop_behaviors(robot)
            self._exec_action.exec_time = 0
        else:
            self._selected_behavior = module.get_behavior()
            self._exec_action = action


# =============================================================================
#                               Main Function
# =============================================================================

def main():
    """
        The main loop that spawns and controls all the threads for the
        wall following project.
    """
    control = RobotController()
    control.start()

    # Prompt to exit safely
    while raw_input("Type 'exit' to quit.") != "exit":
        time.sleep(robot_inf.SENSOR_UPDATE_WAIT)

    control.stop()
    # Wait for every thing to stop safely
    control.join()

if __name__ == '__main__':
    main()
