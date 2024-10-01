from PicoAutonomousRobotics import KitronikPicoRobotBuggy
from GRobotronicsPicoRobotLF import LineFollowing
from machine import Timer

robot = KitronikPicoRobotBuggy()
lf_robot = LineFollowing(robot)

def check_button(timer):
    global state
    buttonValue = robot.button.value()
    if buttonValue == 1:
        state += 1
        if state > 4:
            state = 1

debounceTimer = Timer(-1)
debounceTimer.init(period=200, mode=Timer.PERIODIC, callback=check_button)

state = 0

SIMPLE_LF_SPEED = 20
ADVANCED_LF_SPEED = 30

Kp = 0.3
Ki = 0.001
Kd = 5

while True:
    if state == 0:
        lf_robot.robot_adjustment()
    elif state == 1 or state == 3:
        lf_robot.stop_robot()
    elif state == 2:
        lf_robot.simple_line_following(SIMPLE_LF_SPEED)
    elif state == 4:
        lf_robot.advanced_line_following(Kp, Ki, Kd, ADVANCED_LF_SPEED)
        
