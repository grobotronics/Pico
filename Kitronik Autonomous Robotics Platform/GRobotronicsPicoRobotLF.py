from time import sleep

# Delete "jumpStart=True" if PicoAutonomousRobotics library is fixed by removing the "not" operator from line 55

class LineFollowing:
    def __init__(self, robot):
        self.robot = robot
        self.integral = 0
        self.last_error = 0
        self.state = 0
        # Default min and max sensor values in case robot_adjustment() is not called
        self.default_min_sensor_values = [0, 0, 0]
        self.default_max_sensor_values = [65535, 65535, 65535]
        self.min_sensor_values = self.default_min_sensor_values
        self.max_sensor_values = self.default_max_sensor_values
        self.adjusted = False  # Flag to track if robot_adjustment has been called

    # Normalize sensor values from 0 to 100
    def rescale(self, min_value, max_value, current_value):
        if max_value == min_value:
            return 0  # Avoid division by zero
        if current_value < min_value:
            return 0
        elif current_value > max_value:
            return 100
        rescaled_value = (current_value - min_value) / (max_value - min_value) * 100
        return rescaled_value

    # Return rescaled sensor values, based on whether adjustment has been performed or not
    def rescaled_sensor_values(self):
        if not self.adjusted:
            # Use default min and max sensor values if robot_adjustment hasn't been called
            min_values = self.default_min_sensor_values
            max_values = self.default_max_sensor_values
        else:
            # Use the calibrated sensor values after adjustment
            min_values = self.min_sensor_values
            max_values = self.max_sensor_values

        rescaled_left = self.rescale(min_values[0], max_values[0], self.robot.getRawLFValue("l"))
        rescaled_center = self.rescale(min_values[1], max_values[1], self.robot.getRawLFValue("c"))
        rescaled_right = self.rescale(min_values[2], max_values[2], self.robot.getRawLFValue("r")) 
        return [rescaled_left, rescaled_center, rescaled_right]

    # Robot adjustment method to calibrate sensor min and max values
    def robot_adjustment(self):
        # LED indicator: white lights
        for i in range(4):
            self.robot.setLED(i, (255, 255, 255))
        self.robot.show()
        
        sleep(0.5)
        min_sensor_values = [65535, 65535, 65535]
        max_sensor_values = [0, 0, 0]
        while self.robot.button.value() == 0:  # Continue until the button is pressed
            left = self.robot.getRawLFValue("l")
            center = self.robot.getRawLFValue("c")
            right = self.robot.getRawLFValue("r")
            sensor_values = [left, center, right]
            for i in range(3):
                if sensor_values[i] > max_sensor_values[i]:
                    max_sensor_values[i] = sensor_values[i]
                if sensor_values[i] < min_sensor_values[i]:
                    min_sensor_values[i] = sensor_values[i]
                self.min_sensor_values = min_sensor_values
                self.max_sensor_values = max_sensor_values
                print("Min sensor values: ", min_sensor_values, " | Max sensor values: ", max_sensor_values)
                sleep(0.01)
        sleep(0.3)
        self.stop_robot()
        self.adjusted = True  # Indicate that adjustment has been made
        return min_sensor_values, max_sensor_values
    
    def stop_robot(self):
        # LED indicator: red lights
        for i in range(4):
            self.robot.setLED(i, (255, 0, 0))
        self.robot.show()
        # Stop robot motors to allow positioning it on the black line
        self.robot.motorOff("l")
        self.robot.motorOff("r")

    # Simple line following logic
    def simple_line_following(self, speed):
        # LED indicator: green lights
        for i in range(4):
            self.robot.setLED(i, (0, 255, 0))
        self.robot.show()
        # Ensure rescaled values reflect current sensor calibration (adjusted or default)
        rescaled_values = self.rescaled_sensor_values()
        # If no sensor is detecting the black line, stop the robot
        if all(value < 20 for value in rescaled_values):
            self.robot.motorOff("l")
            self.robot.motorOff("r")
        # If the center sensor detects the black line, move forward
        elif rescaled_values[1] > 10:
            self.robot.motorOn("l", "f", speed, jumpStart=True)
            self.robot.motorOn("r", "f", speed, jumpStart=True)
        else:
            # Adjust direction based on left and right sensors if center sensor is off the line
            while rescaled_values[1] < 30:
                if rescaled_values[0] > rescaled_values[2]:
                    self.robot.motorOn("l", "r", speed, jumpStart=True)
                    self.robot.motorOn("r", "f", speed, jumpStart=True)
                else:
                    self.robot.motorOn("l", "f", speed, jumpStart=True)
                    self.robot.motorOn("r", "r", speed, jumpStart=True)
                rescaled_values = self.rescaled_sensor_values()
    
    # PID line following control logic
    def advanced_line_following(self, Kp, Ki, Kd, speed):
        # LED indicator: blue lights
        for i in range(4):
            self.robot.setLED(i, (0, 0, 255))
        self.robot.show()
        
        rescaled_values = self.rescaled_sensor_values()
        error = rescaled_values[2] - rescaled_values[0]
        P = Kp * error
        self.integral += error
        I = Ki * self.integral
        derivative = error - self.last_error
        D = Kd * derivative
        self.last_error = error
        pid_output = P + I + D

        left_speed = speed + pid_output
        right_speed = speed - pid_output

        left_speed = min(max(left_speed, -100), 100)
        right_speed = min(max(right_speed, -100), 100)

        # Stop the robot if no sensor detects the black line
        if all(value < 20 for value in rescaled_values):
            self.robot.motorOff("l")
            self.robot.motorOff("r")
        else:
            self.robot.motorOn("l", "f" if left_speed >= 0 else "r", abs(left_speed), jumpStart=True)
            self.robot.motorOn("r", "f" if right_speed >= 0 else "r", abs(right_speed), jumpStart=True)

