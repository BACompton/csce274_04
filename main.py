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
    DOCK = 2
    OBSTACLE_AVOIDANCE = 1

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
        rtn = self._action is not None and self._action.type != Action.UNKNOWN
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
            # Temporary Logging Behavior used to track the selected action
            # throughout a test course.
            log_file.log_stmt("Drive Direct " + str(action.params))

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
    UNKNOWN = -2
    DRIVE_DIRECT = 2
    STOP_MODULES = 0
    TERMINATE = -1

    type = None
    params = None
    event = None
    exec_time = None
    log = False

    def __init__(self, act_type, params=(),
                 wait_time=robot_inf.SENSOR_UPDATE_WAIT):
        """
            Constructs an action of the specified type with the provided
             parameters
        :param act_type:
            The type of actuator action. This value should be set to available
            action within Action
        :param params:
            The list of parameters to use with the specified action type.
        :param wait_time
            The amount of time that the action should run
        """
        self.type = act_type
        self.params = params
        self.exec_time = wait_time
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
            self._file.write(LogFile._timestamp()+", "+str(datum)+"\n")
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

    @staticmethod
    def _timestamp():
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
    # The maximum wait time will rotate the robot roughly 30 degrees
    # The minimum wait time will rotate the robot roughly 15 degrees
    _MIN_TURN_TIME = robot_inf.WHEEL_BASE * (math.pi/6) / (2 * _WHEEL_VEL)
    _MAX_TURN_TIME = robot_inf.WHEEL_BASE * (math.pi/12) / (2 * _WHEEL_VEL)

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
        """
            This method is where the obstacle avoidance behavior is defined. The
            decision matrix for this docking behavior is as follows:
                1) Stop All Active Behaviors
                    a) Unsafe motion is detected by the internal helper method
                       '_safe_motion'
                2) Randomly Rotate a direction
                    a) Both the left and right bumps are pressed
                3) Rotate Left
                    a) The left bump is pressed
                4) Rotate Right
                    a) The right bump is pressed
        """
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
                    wait_time=random.uniform(ObstacleAvoidance._MIN_TURN_TIME,
                                             ObstacleAvoidance._MAX_TURN_TIME)
                )
            elif turn_ccw:
                action = Action(
                    Action.DRIVE_DIRECT,
                    params=[_WHEEL_VEL, -_WHEEL_VEL],
                    wait_time=random.uniform(ObstacleAvoidance._MIN_TURN_TIME,
                                             ObstacleAvoidance._MAX_TURN_TIME)
                )

            # Issue the generated action
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
        if bumps[robot_inf.Bump.BUMP_R]:
            return True
        if cliffs[robot_inf.Cliff.CLIFF_R] or cliffs[robot_inf.Cliff.CLIFF_FR]:
            return True
        return False


class Docking(_StoppableThread, Behavior):
    """
        This is the low level actuator controller that is responsible for
        docking the robot on a charging station. Once successfully docked this
        behavior will stop the program.

        :type _sensor sensor_inf.Sensor
        :type _log LogFile
        :type _prev_action Action
        :type _lost_center bool
    """

    # -------------------------------------------------------------------- #
    # -                     Internal Constants                           - #
    # -------------------------------------------------------------------- #

    # The number of times the sampled infrared signal can be empty before
    # the docking station is considered lost.
    LOST_DOCK_LIMIT = 10

    # The number of times the behavior should sample the infrared signal
    # before generating an action.
    SAMPLES = 10

    # The amount of time in seconds a forward or backward command should be ran.
    SEEK_EXEC_TIME = 10*robot_inf.SENSOR_UPDATE_WAIT

    # -------------------------------------------------------------------- #
    # -                    Behavior Definition                           - #
    # -------------------------------------------------------------------- #

    _sensor = None
    _log = None
    _prev_action = Action(Action.UNKNOWN)
    _lost_center = False

    def __init__(self, sensor, log):
        _StoppableThread.__init__(self, daemon=True)
        Behavior.__init__(self, Behavior.DOCK)
        self._sensor = sensor
        self._log = log
        self._prev_action = Action(Action.UNKNOWN)
        self._lost_center = False

    def run(self):
        """
            This method is where the docking behavior is defined. The decision
            matrix for this docking behavior is as follows:
                NOTE: If any of the situations presented within a behavior are
                      met that action will be selected

                      In the case where multiple action can selected, the action
                      that comes first will be the one selected.

                1) Terminate Program:
                    a) The robot is in any charging state
                2) Move Backwards
                    a) The left or right bumper is pressed
                3) Drive Forward
                    a) The internal method '_drive_forward' returns True
                    b) No other explict direction can be determined and the
                       last actual movement was forward.
                4) Rotate clockwise
                    a) The internal method '_rotate_cw' returns True
                5) Rotate counter-clockwise
                    a) The internal method '_rotate_ccw' returns True
                6) Repeat last instruction
                    a) The dock can still be detected, but no explict action
                       can be determined, and the center of the dock is already
                       lost.

                       This case helps the docking behavior recover in the case
                       it over corrects itself.
                7) Randomly Rotate a Direction
                    a) The dock can still be detected but no explicit action
                       be determined, the previous action is not a valid
                       movement, and the center of the dock was just lost.

                       This case seeks the detected dock's center whenever an
                       explicit direction cannot be determined.
                8) Reverse Previous Direction
                    a) The dock can still be detected but no explicit action
                       be determined and the center of the dock was just lost.

                       This case initially helps the docking behavior determine
                       the direction to rotate whenever it happens to over
                       correct itself.
                9) Await further instructions
                    a) The robot is close enough to a new dock to decide on
                       an action.
                    b) There are not enough IR sample to perform an action.
                       The required number of samples is defined in
                       Docking.SAMPLES.
                    c) The active dock's signal was lost.
                10) Nothing
                    a) No active dock is found
        """
        sample = self._get_sample_base()
        sample_count = 0
        dock_count = Docking.LOST_DOCK_LIMIT

        while not self._stop:
            # Log the previous action
            if self.has_action():
                prev_action = self.get_action()

                if prev_action.log:
                    Behavior.log_action(self._log, self._sensor, prev_action)

            # Gather necessary sensor data
            ir_chars = self._sensor.get_ir_chars()
            bumps = self._sensor.get_bumps()
            charging = self._sensor.get_charging()

            # Process the sensor data by decomposing the acquired characters
            # into boolean flags for the force field, red buoy, nad green buoy.
            data = {}
            for ir in ir_chars:
                data[ir] = robot_inf.Dock.decompose(ir_chars[ir])
            self._add_input(sample, data)
            sample_count += 1

            # Generate action based on the current sample
            if bumps[robot_inf.Bump.BUMP_L] \
                    or bumps[robot_inf.Bump.BUMP_R]:
                action = Action(Action.DRIVE_DIRECT,
                                [-_WHEEL_VEL, -_WHEEL_VEL],
                                wait_time=Docking.SEEK_EXEC_TIME)
            else:
                action = Action(Action.DRIVE_DIRECT, [0, 0])

            if charging != robot_inf.Charging.NOT_CHARGING:
                action = Action(Action.TERMINATE)

            # If there was not dock to follow
            elif dock_count >= Docking.LOST_DOCK_LIMIT:
                # If a new dock is found
                if self._possible_action(sample):
                    action = Action(Action.DRIVE_DIRECT, [0, 0])
                    dock_count = 0
                    sample_count = 0
                    sample = self._get_sample_base()
                else:
                    action = Action(Action.UNKNOWN)
                    self._prev_action = action
                    self._lost_center = False

            # If there are enough samples to preform an action
            elif sample_count >= Docking.SAMPLES:
                # If no dock was detected
                if self._lost_dock(sample):
                    # If successfully docked
                    action = Action(Action.DRIVE_DIRECT, [0, 0])
                    dock_count += 1
                # If the dock was detected
                else:
                    # If not collision, generate a docking action.
                    if action.type != Action.UNKNOWN:
                        action = self._gen_action(sample)
                        self._prev_action = action
                    dock_count = 0

                sample_count = 0
                sample = self._get_sample_base()

            self._set_action(action)
            self.get_action().event.wait(action.exec_time)

    # -------------------------------------------------------------------- #
    # -                      Internal Helpers                            - #
    # -------------------------------------------------------------------- #

    def _get_sample_base(self):
        """
            Generates the base data structure for a new sample.
        :return:
            The empty data structure for a new sample.
        """
        return {
            robot_inf.Dock.OMNI: {},
            robot_inf.Dock.LEFT: {},
            robot_inf.Dock.RIGHT: {}
        }

    def _add_input(self, sample, data):
        """ Adds the provided IR data to IR action sample.

        :param sample:
            The IR sample that should append the new IR data
        :param data:
            The new IR data to add to the IR sample
        """
        for ir in data:
            for key in data[ir]:
                if key in data[ir] and data[ir][key]:
                    if ir not in sample:
                        sample[ir] = {}

                    sample[ir][key] = True

    def _lost_dock(self, sample):
        """ Determines if a the dock was not detected given the current sample.

        :param sample:
            The infrared character sample to use
        :return:
            True if the sample has no infrared characters from a dock.
        """
        return len(sample[robot_inf.Dock.OMNI]) == 0 \
            and len(sample[robot_inf.Dock.RIGHT]) == 0 \
            and len(sample[robot_inf.Dock.LEFT]) == 0

    def _possible_action(self, sample):
        """ Determines if a the sample can produce an explicit value.

        :param sample:
            The infrared character sample to use
        :return:
            True if the sample can result in a proper action.
        """
        return self._drive_forward(sample) \
            or self._rotate_cw(sample) \
            or self._rotate_ccw(sample)

    def _gen_action(self, sample):
        """ Generates an action based on the provided infrared sample. The gaol
            of the resulting action is to get the robot closer to a successful
            dock.

        :param sample:
            The infrared sample that the action will be based upon
        :return:
            An action that should provide progress to a successful dock.
        """
        rtn = Action(Action.DRIVE_DIRECT, [0, 0],
                     wait_time=robot_inf.SENSOR_UPDATE_WAIT*5)
        has_prev_action = self._prev_action.type == Action.UNKNOWN
        forward = has_prev_action \
            and self._prev_action.type == Action.DRIVE_DIRECT \
            and self._prev_action.params[0] > 0 \
            and self._prev_action.params[0] == self._prev_action.params[1]

        found_center = True
        if self._drive_forward(sample):
            rtn.params = [_WHEEL_VEL, _WHEEL_VEL]
            rtn.exec_time = Docking.SEEK_EXEC_TIME
        elif self._rotate_cw(sample):
            rtn.params = [-_WHEEL_VEL, _WHEEL_VEL]
        elif self._rotate_ccw(sample):
            rtn.params = [_WHEEL_VEL, -_WHEEL_VEL]
        elif has_prev_action:
            # Continue to move forward
            if forward:
                rtn.params = [_WHEEL_VEL, _WHEEL_VEL]
                rtn.exec_time = Docking.SEEK_EXEC_TIME
            # Continue to rotate in the chosen direction

            elif self._lost_center:
                found_center = False
                rtn = self._prev_action

            # Choose a random way to rotate
            elif not self._lost_center \
                    and self._prev_action.type == Action.DRIVE_DIRECT:
                found_center = False

                if self._prev_action.params[0] != 0:
                    rtn.params = [-self._prev_action.params[0],
                                -self._prev_action.params[1]]
                    rtn.exec_time = self._prev_action.exec_time
                else:
                    gain = 1
                    if bool(random.getrandbits(1)):
                        gain = -1

                    rtn.params = [gain * _WHEEL_VEL, -gain * _WHEEL_VEL]

        self._lost_center = not found_center
        return rtn

    def _rotate_cw(self, sample):
        """
            Determines if the robot should rotate clockwise to line itself
            up with the dock.

            This method should return True if any of the following situations
            are present:
                1) The right IR sensor detects the green buoy and not the red
                   buoy. This essentially translates to the case where the robot
                   is turned too much counter-clockwise.

                   The goal for this situation is to try and keep the red and
                   green buoy within the detection range.

                2) The left IR sensor detects the green buoy and the right IR
                   sensor does not detect the red buoy. This situation
                   represents the case where the robot has slightly overturned
                   counter-clockwise.

                   The goal for this situation is to try and align the robot
                   to the center of the dock
        :param sample:
            The sample that the decision is based on.
        :return:
            True if any of the above situations happen. Otherwise, False.
        """
        if self._check_ir(sample[robot_inf.Dock.RIGHT],
                          robot_inf.Dock.GREEN_BUOY) \
                and not self._check_ir(sample[robot_inf.Dock.RIGHT],
                                       robot_inf.Dock.RED_BUOY):
            return True

        return self._check_ir(sample[robot_inf.Dock.LEFT],
                              robot_inf.Dock.GREEN_BUOY) \
            and not self._check_ir(sample[robot_inf.Dock.RIGHT],
                                   robot_inf.Dock.RED_BUOY)

    def _rotate_ccw(self, sample):
        """
            Determines if the robot should rotate counter-clockwise to line
            itself up with the dock.

            This method should return True if any of the following situations
            are present:
                1) The left IR sensor detects the red buoy and not the green
                   buoy. This essentially translates to the case where the robot
                   is turned too much clockwise.

                   The goal for this situation is to try and keep the red and
                   green buoy within the detection range.

                2) The right IR sensor detects the red buoy and the left IR
                   sensor does not detect the green buoy. This situation
                   represents the case where the robot has slightly overturned
                   clockwise.

                   The goal for this situation is to try and align the robot
                   to the center of the dock
        :param sample:
            The sample that the decision is based on.
        :return:
            True if any of the above situations happen. Otherwise, False.
        """
        if not self._check_ir(sample[robot_inf.Dock.LEFT],
                              robot_inf.Dock.GREEN_BUOY) \
                and self._check_ir(sample[robot_inf.Dock.LEFT],
                                   robot_inf.Dock.RED_BUOY):
            return True

        return not self._check_ir(sample[robot_inf.Dock.LEFT],
                                  robot_inf.Dock.GREEN_BUOY) \
            and self._check_ir(sample[robot_inf.Dock.RIGHT],
                               robot_inf.Dock.RED_BUOY)

    def _drive_forward(self, sample):
        """
            Determines if the robot should move forward by checking the left IR
            sensor for the green buoy and the right for the red buoy.

            The overall goal of this method is to approach the dock .
        :param sample:
            The sample that the decision is based on.
        :return:
            True if both sensors detect the respective buoy. Otherwise, False.
        """
        return self._check_ir(sample[robot_inf.Dock.LEFT],
                              robot_inf.Dock.GREEN_BUOY) \
            and self._check_ir(sample[robot_inf.Dock.RIGHT],
                               robot_inf.Dock.RED_BUOY)

    def _check_ir(self, ir, flag):
        """ Determines the value of the specified flag of the given infrared
            character sensor.

        :param ir:
            The infrared character sensor to check
        :param flag:
            The specific infrared flag to find.
        :return:
            True if the flag is in the sensor's readings and is True.
            Otherwise, False.
        """
        if flag not in ir:
            return False
        return ir[flag]

    def _check_ir_set(self, ir, key_set={}):
        """ Checks if the infrared sensor's charater matches the provided flags.

        :param ir:
            The infrared character sensor to check
        :param key_set:
            The flags to check for
        :return:
            True if the infrared sensor's value has the specified flags
        """
        for key in key_set:
            val = self.check_ir(ir, key)

            if val != key_set[key]:
                return False
        return True

    def _create_key_set(self, force=None, red=None, green=None):
        """ Constructs a key set based on the provided flags.

        :param force:
            The force field flag
        :param red:
            The red buoy flag
        :param green:
            The green buoy flag
        :return:
            The key set of the provided flags
        """
        rtn = {}

        if force is not None:
            rtn[robot_inf.Dock.FORCE_FIELD] = force
        if red is not None:
            rtn[robot_inf.Dock.RED_BUOY] = red
        if green is not None:
            rtn[robot_inf.Dock.GREEN_BUOY] = green

        return rtn


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
    _LOST_WALL_THRESHOLD = math.ceil(15. / (robot_inf.SENSOR_UPDATE_WAIT+.05))

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
        """
            This method is where the wall following behavior is defined. The
            decision matrix for this docking behavior is as follows:
                1) Turn Left
                    a) The controller associated with the front light bump
                       sensor has an output that exceeds the value expressed by
                       WallFollow._LEFT_TURN_THRESHHOLD.
                2) Move Forward
                    a) The number of cycles since the last wall sighting
                       exceeds the value in WallFollow._LOST_WALL_THRESHHOLD.
                3) Use wall following PID controller
                    a) No other action can be selected

        """
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
        # Instead of creating a separate PID controller for right wall
        # following, the gains are adjusted to reduce the radius of a right
        # turn since the wall following PID controller already covers right
        # turns.
        follow_gains = {
            robot_inf.PIDController.KP_KEY: .05,
            robot_inf.PIDController.KI_KEY: .005,
            robot_inf.PIDController.KD_KEY: .0025
        }
        right_turn_gains = {
            robot_inf.PIDController.KP_KEY: .225
        }
        wall_follow.set_gains(follow_gains)

        # A counter for the number of cycles with no wall detected.
        no_wall_count = 0

        # ------------------------------------------------------------ #
        # -                    Behavior Decision                     - #
        # ------------------------------------------------------------ #

        while not self._stop:
            # Log the previous action

            if self.has_action():
                prev_action = self.get_action()

                if prev_action.log:
                    Behavior.log_action(self._log, self._sensor, prev_action)
            else:
                prev_action = Action(Action.UNKNOWN)

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
                if prev_action.log:
                    no_wall_count += 1
                wall_follow.reset(init_pt=wall_pt)

            # Found right wall
            else:
                wall_follow.set_gains(follow_gains)
                no_wall_count = 0
                wall_follow.add_point(wall_pt)

            self.get_action().event.wait(wait_time)

    # -------------------------------------------------------------------- #
    # -                  Action Definitions                            - #
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
        robot.change_state(robot_inf.State.SAFE)
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

                    # Initialize and start the necessary modules
                    self._active_behaviors = [
                        ObstacleAvoidance(sensor, self._log),
                        WallFollow(sensor, self._log),
                        Docking(sensor, self._log)
                    ]
                    for module in self._active_behaviors:
                        if isinstance(module, threading.Thread):
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

            # Stop the robot if no action is found.
            if self._selected_behavior == Behavior.UNKNOWN:
                Behavior.execute_action(robot,
                                        Action(Action.DRIVE_DIRECT, [0, 0]))

            # Preform the selected action for specified amount of time or until
            # a higher priority action arrives. The action can also be
            # interrupted by a CLEAN button press.
            self._wait(self._exec_action.exec_time,
                       robot_inf.SENSOR_UPDATE_WAIT, sensor)
        print "Stopping Listening"

        # Cleans up all threads spawned during execution.
        self._stop_behaviors(robot, wait=True)
        robot.change_state(robot_inf.State.PASSIVE)
        self._log.close()
        sensor.stop_update(join=True)
        exit(0)

    # -------------------------------------------------------------------- #
    # -                     Internal Helpers                             - #
    # -------------------------------------------------------------------- #

    def _stop_behaviors(self, robot, wait=False):
        """
            Notifies all modules to halt execution and stops the robot.
        :param robot:
            The robot that should be stopped.
        :param wait:
            Flags the method to wait for each behavior to end before returning
        """
        for i in range(len(self._active_behaviors)-1, -1, -1):
            behavior = self._active_behaviors[i]

            if isinstance(behavior, _StoppableThread):
                behavior.stop()
                if wait:
                    behavior.join()

            self._active_behaviors.remove(behavior)
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

        # Check to see if the robot docked during action.
        if sensor.get_charging() != robot_inf.Charging.NOT_CHARGING:
            return True

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

            # Play Docking song
            robot.change_state(robot_inf.State.SAFE)
            time.sleep(robot_inf.SENSOR_UPDATE_WAIT)
            robot.play_happy_song()
            action.exec_time = .5
            print "Press any key to finish termination."
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
    prompt = "Type 'exit' to quit.\n"
    while True:
        txt = ""

        try:
            txt = raw_input(prompt)
        except ValueError:
            txt = "exit"

        if txt == "exit":
            break
        time.sleep(robot_inf.SENSOR_UPDATE_WAIT)


    control.stop()
    # Wait for every thing to stop safely
    control.join()

if __name__ == '__main__':
    main()
