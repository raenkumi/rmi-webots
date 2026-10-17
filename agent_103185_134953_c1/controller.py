from controller import Robot
import math


# =========================================================
# ROBOT DEFINITIONS
# =========================================================

robot = Robot()
timeStep = int(robot.getBasicTimeStep())

turningVelocity = 6.0
cruiseVelocity = 6.28
maxVelocity = 6.28

# ACTUATORS =======================================================

# motors
leftMotor = robot.getDevice("left wheel motor")
rightMotor = robot.getDevice("right wheel motor")

leftMotor.setPosition(float("inf"))
rightMotor.setPosition(float("inf"))

leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)


# SENSORS =========================================================

# camera
camera = robot.getDevice("camera")
camera.enable(timeStep)

width = camera.getWidth()
height = camera.getHeight()

print("Camera:", width, "x", height)

# ground
gs0 = robot.getDevice("gs0")
gs1 = robot.getDevice("gs1")
gs2 = robot.getDevice("gs2")

gs0.enable(timeStep)
gs1.enable(timeStep)
gs2.enable(timeStep)


# COMPUTER VISION =========================================

# colors
RED = "RED"
BLUE = "BLUE"
YELLOW = "YELLOW"
WHITE = "WHITE"
NONE = "NONE"

colorTolerance = 100

# line controller parameters
Kp = 0.04
Kd = 0.08
exponent = 1.7
futureWeight = 1
maxCorrection = 3.5
previous_error = 0.0
last_direction = 1

# Values for 180 turn
turnVelocity = 4.0
turnMinSteps = 15
turnMaxSteps = 100

gsBlackThreshold = 500 # gs1 black below this (typical: ~300 black, ~900 white)
black_threshold = 250

# =========================================================
# STATE VARIABLES
# =========================================================

# color memory
previous_color = "WHITE"
turn_requested = False
white_seen = 100 # white required to be seen n cycles
# color_visible_last_frame = False


# =========================================================
# HELPER FUNCTIONS
# =========================================================

# cv line find
# ========================================
def find_line(row):

    difference_threshold = 100

    brightness = [sum(pixel) for pixel in row]

    falling_edge = -1   # WHITE -> BLACK
    rising_edge = -1    # BLACK -> WHITE

    difference = [0] * (len(brightness) - 1)

    for i in range(1, len(brightness)):

        difference[i - 1] = brightness[i] - brightness[i - 1]

        # WHITE -> BLACK
        if difference[i - 1] < -difference_threshold:
            falling_edge = i

        # BLACK -> WHITE
        elif difference[i - 1] > difference_threshold:
            rising_edge = i
            break

    #print(difference)

    # --------------------------------------------------
    # TWO EDGES
    # WHITE -> BLACK -> WHITE
    # --------------------------------------------------

    if falling_edge != -1 and rising_edge != -1:

        line_position = falling_edge
        line_width = rising_edge - falling_edge

        return line_position, line_width

    # --------------------------------------------------
    # ONE EDGE
    # WHITE -> BLACK -> edge of camera
    # --------------------------------------------------

    if falling_edge != -1:

        line_position = falling_edge
        line_width = len(row) - falling_edge

        return line_position, line_width

    # --------------------------------------------------
    # BLACK -> WHITE
    # line starts outside the camera
    # --------------------------------------------------

    if rising_edge != -1:

        line_position = 0
        line_width = rising_edge

        return line_position, line_width

    # --------------------------------------------------
    # NOTHING FOUND
    # --------------------------------------------------

    return -1, 0


# cv color detection
# ========================================
def get_adjacent_color(row, line_position, line_width):
    """
    Checks only the pixel immediately to the RIGHT
    of the black line.
    """

    color_position = line_position + line_width

    if color_position >= len(row):
        return NONE

    r, g, b = row[color_position]

    # WHITE = (255, 255, 255)
    if (
        r > 255 - colorTolerance
        and g > 255 - colorTolerance
        and b > 255 - colorTolerance
    ):
        return WHITE
    
    # YELLOW = (255, 255, 0)
    if (
        r > 255 - colorTolerance
        and g > 255 - colorTolerance
        and b < colorTolerance
    ):
        return YELLOW


    # RED = (255, 0, 0)
    if (
        r > 255 - colorTolerance
        and g < colorTolerance
        and b < colorTolerance
    ):
        return RED

    # GREEN == CYAN FIX:
    # BLUE group = GREEN (0,255,0) or CYAN (0,255,255)
    # low R, high G, B free
    if (
        r < colorTolerance
        and g > 255 - colorTolerance
    ):
        return BLUE

    return NONE


# cv color sequence tracking
# ========================================
def detect_color(row, line_position, line_width):
    """
    Checkpoint tracking. Markers appear in the clockwise
    order RED -> BLUE -> YELLOW. A repeated marker, or the
    reverse transition RED -> YELLOW, means the robot is
    traveling counter-clockwise -> request a 180 turn.

    White_seen gates detections: a marker only counts after
    enough white frames, so one physical square is not
    registered several times while driving over it.
    """
    global previous_color, turn_requested, white_seen

    detected_color = NONE

    if line_position != -1:
        detected_color = get_adjacent_color(row, line_position, line_width)

    if detected_color == WHITE:
        white_seen += 1
        color = NONE
    else:
        color = detected_color

    if white_seen > 40:
        # print (white_seen, "WHITE detected - color markers are now valid")
        if color != NONE:
            print("Color Detected:", color)

            if color == previous_color:
                # print(color,"detected twice -> TURN REQUEST")
                turn_requested = True
            else:
                # print("Previous color:",previous_color,"New color:", color)
                if previous_color == "RED" and color == "YELLOW":
                    print("RED -> YELLOW -> TURN REQUEST")
                    turn_requested = True

                previous_color = color
            white_seen = 0
    else:
        if color != NONE:
            # print(color, "detected but WHITE not seen yet -> IGNORE")
            pass


# =========================================================
# CONTROLLER FUNCTIONS
# =========================================================

# cv line follower/aligner
# ========================================
def align_line(line_position, line_width, future_position, future_width):
    """
    Line-following controller: nonlinear PD on the camera
    line position, with a lookahead (feedforward) term.

    Error = line center offset from image center, in pixels.
    Output = differential wheel speed (positive = turn right).
    """
    global previous_error, last_direction
    
    line_center = (line_position + (line_width - 1) / 2)
    camera_center = width / 2
    current_error = (line_center - camera_center)


    if future_position != -1:
        future_center = (future_position + (future_width - 1) / 2)
        future_error = (future_center - camera_center)
    else:
        future_error = current_error

    if current_error > 1.5: last_direction = 1
    elif current_error < -1.5: last_direction = -1
    
    current_nonlinear = (Kp * math.copysign(
            abs(current_error) ** exponent, current_error))

    future_nonlinear = (Kp * math.copysign(
            abs(future_error) ** exponent, future_error))

    position_correction = (current_nonlinear + futureWeight * future_nonlinear)

    error_change = (current_error - previous_error)
    derivative_correction = (Kd * error_change)

    correction = (position_correction + derivative_correction)

    correction = max(-maxCorrection, min(maxCorrection, correction))

    left_speed = (cruiseVelocity + correction)
    right_speed = (cruiseVelocity - correction)

    if left_speed > maxVelocity:
        #print("Left speed overflow:", left_speed, "-> redistributing to right wheel")
        right_speed -= (left_speed - maxVelocity)
        left_speed = maxVelocity
    elif right_speed > maxVelocity:
       # print("Right speed overflow:", right_speed, "-> redistributing to left wheel")
        left_speed -= (right_speed - maxVelocity)
        right_speed = maxVelocity

    # final safety clamp
    left_speed = max(-maxVelocity, min(maxVelocity, left_speed))
    right_speed = max(-maxVelocity, min(maxVelocity, right_speed))
    #print("Left speed:", left_speed, "Right speed:", right_speed)

    leftMotor.setVelocity(left_speed)
    rightMotor.setVelocity(right_speed)

    # 11. Save error
    previous_error = current_error


# turn around 180 degrees
# ========================================
def turn_180():
    """In-place spin (wheels opposed). Open-loop for the first
    turnMinSteps (to rotate off the line under the robot),
    then closed-loop: exits when the center ground sensor
    gs1 reacquires the line, which by geometry happens at
    ~180 degrees. turnMaxSteps is a timeout fallback.
    """
    global turn_requested, white_seen, previous_error, last_direction

    turn_counter = 0
    turn_requested = False
    white_seen = 0

    print("TURNING 180 DEGREES")

    # TURN: spin in place, exit when gs1 reads black
    # -------------------------------------------------
    while (
            robot.step(timeStep) != -1
            and turn_counter < turnMaxSteps
        ):
        if (last_direction > 0):
            #print("Turning right, step", turn_counter)
            leftMotor.setVelocity(turnVelocity)
            rightMotor.setVelocity(-turnVelocity)
        else:
            #print("Turning left, step", turn_counter)
            leftMotor.setVelocity(-turnVelocity)
            rightMotor.setVelocity(turnVelocity)

        turn_counter += 1

        # first leave the line we are standing on
        if turn_counter < turnMinSteps:
            continue

        if gs1.getValue() < gsBlackThreshold:
            print("Line reacquired by gs1 after", turn_counter, "steps")
            break

    # STOP
    # -------------------------------------------------
    leftMotor.setVelocity(0.0)
    rightMotor.setVelocity(0.0)

    previous_error = 0.0
    last_direction = -1

    print("180 degree turn complete, steps:", turn_counter)


# line lost recovery
# ========================================
def line_lost():
    """Recovery search: open-loop pivot (one wheel stopped)
    toward the side where the line was last seen, until the
    camera reacquires it."""
    # print("Line lost")

    if last_direction > 0:
        leftMotor.setVelocity(cruiseVelocity)
        rightMotor.setVelocity(0.0)
    else:
        leftMotor.setVelocity(0.0)
        rightMotor.setVelocity(cruiseVelocity)


# =========================================================
# MAIN LOOP
# =========================================================
def main():

    while robot.step(timeStep) != -1:

        # GET CAMERA IMAGE
        image = camera.getImageArray()
        if image is None: continue

        # FIND LINE
        row = image[height - 1]
        line_position, line_width = find_line(row)
        #print("Line position:", line_position, "Line width:", line_width)

        # DETECT FUTURE LINE
        future_row_index = max(0, height - 8)
        row = image[future_row_index]
        future_line, future_width = find_line(row)

        # 180 DEGREE TURN
        if turn_requested :
            turn_180()
            continue
        
        # DETECT COLOR
        detect_color(row, line_position, line_width)


        # FOLLOW THE LINE
        if line_position != -1: align_line(line_position, line_width, future_line, future_width)

        # LINE LOST
        else: line_lost()


main()