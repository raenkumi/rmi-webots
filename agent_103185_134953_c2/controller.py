"""sample2 controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
import math
from collections import deque

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getDevice('motorname')
gps = robot.getDevice('gps')
gps.enable(timestep)

camera = robot.getDevice('camera')
camera.enable(timestep)

compass = robot.getDevice('compass')
compass.enable(timestep)

left_motor = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))

initial_gps = None

# # Line-following controller parameters
# Kp = 0.15
# Kd = 0.05
# exponent = 1.0
# cruiseVelocity = 3.0
# maxCorrection = 2.0
# maxVelocity = 6.28

# previous_error = 0.0
# last_direction = 0

# width = camera.getWidth()


# Function to rotate the robot 90 degrees.
def rotate_90():
    print("Rotating the robot 90 degrees...")
    # Get the current orientation of the robot
    compass_values = compass.getValues()
    #Compass gives[x,y,z])
    current_orientation = math.atan2(compass_values[1], compass_values[0])
    print('current_orientation', current_orientation)
    # Calculate the target orientation (90 degrees from current)
    target_orientation = current_orientation + math.pi / 2

    # Normalize the target orientation to be within -pi to pi
    if target_orientation > math.pi:
        target_orientation -= 2 * math.pi

    # Rotate the robot until it reaches the target orientation
    while True:
        compass_values = compass.getValues()
        # print('compass', compass_values)
        current_orientation = math.atan2(compass_values[1], compass_values[0])

        # Check if the robot has reached the target orientation
        if abs(current_orientation - target_orientation) < 0.01:  # Allow a small margin of error
            break

        # Set motor speeds to rotate the robot (this is a placeholder, actual motor control code needed)
        left_motor.setVelocity(1.0)
        right_motor.setVelocity(-1.0)

        robot.step(timestep)

    # Stop the motors after rotation
    left_motor.setVelocity(0.0)
    right_motor.setVelocity(0.0)

# gets 2 
    

def find_line(row):

    difference_threshold = 100

    brightness = [sum(pixel) for pixel in row]

    falling_edge = -1   # WHITE -> BLACK
    rising_edge = -1    # BLACK -> WHITE

    line_brightness = sum(brightness) / len(brightness)

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
    # Everything is black
    # --------------------------------------------------
    # if line_brightness < 500:
    #     return 0, len(row)

    # --------------------------------------------------
    # NOTHING FOUND
    # --------------------------------------------------

    return -1, 0
#---
#        |
#      -- --
#        |
#---
def caracterize_line(line_position, line_width):
    # find where the robot is facing:
    compass_values = compass.getValues()

    #get the vector with the most absolute value, this is the direction the robot is facing
    if abs(compass_values[0]) > abs(compass_values[1]):
        if compass_values[0] > 0:
            facing = "N"
        else:
            facing = "S"
    else:
        if compass_values[1] > 0:
            facing = "E"
        else:
            facing = "W"
    
    if line_position > 0 and line_position < camera.getWidth():
        return "l" + facing
    else:
        return "n" + facing

Nodes = []
toexplore = []

def infereNodes(node_location=None):
    # Get the current node from the Nodes list
    current_node = next((node for node in Nodes if node["location"] == node_location), None)
    
    # if current_node is None:
    #     print(current_node)
    #     print("Error: Current node not found in Nodes list.")
    #     return
    
    # Check the existing lines in the current node
    existing_lines = current_node["lines"]


    # Infer the presence of lines in the adjacent nodes based on the current node's lines
    for line in existing_lines:
        line_type, direction = line[0], line[1]
        if line_type.startswith("n"):  # Only infer if there's a line present
            # if direction == "N":
            #     adjacent_node_location = (node_location[0], node_location[1] + 1)
            #     adj_line_type = "nS"  # The adjacent node would have a line facing south
            # elif direction == "E":
            #     adjacent_node_location = (node_location[0] + 1, node_location[1])
            #     adj_line_type = "nW"  # The adjacent node would have a line facing west
            # elif direction == "S":
            #     adjacent_node_location = (node_location[0], node_location[1] - 1)
            #     adj_line_type = "nN"  # The adjacent node would have a line facing north
            # elif direction == "W":
            #     adjacent_node_location = (node_location[0] - 1, node_location[1])
            #     adj_line_type = "nE"  # The adjacent node would have a line facing east
            continue  # Skip if the line type is not recognized
        else:
            # Determine the adjacent node's location based on the direction
            if direction == "N":
                adjacent_node_location = (node_location[0], node_location[1] + 1)
                adj_line_type = "lS"  # The adjacent node would have a line facing south
            elif direction == "E":
                adjacent_node_location = (node_location[0] + 1, node_location[1])
                adj_line_type = "lW"  # The adjacent node would have a line facing west
            elif direction == "S":
                adjacent_node_location = (node_location[0], node_location[1] - 1)
                adj_line_type = "lN"  # The adjacent node would have a line facing north
            elif direction == "W":
                adjacent_node_location = (node_location[0] - 1, node_location[1])
                adj_line_type = "lE"  # The adjacent node would have a line facing east
            else:
                continue  # Skip if the direction is not recognized
        
        if any(node["location"] == adjacent_node_location for node in Nodes):
            continue

        # Check if the node is on the list of the explored nodes if not, add it to the list of nodes to explore
        adjacent_node = next((node for node in Nodes if node["location"] == adjacent_node_location), None)
        if adjacent_node is None:
            # If the adjacent node is not in the Nodes list, add it to the toexplore list with the inferred line type
            if adjacent_node_location not in toexplore:
                toexplore.append({"location": adjacent_node_location, "lines": [adj_line_type]})
        else:
            # If the adjacent node is already in the Nodes list, update its lines if necessary
            if line_type[0] not in adjacent_node["lines"]:
                adjacent_node["lines"].append(adj_line_type)

    #check if there's any toexplore node that became complete, if so, remove it from the toexplore list add it to the Nodes list
    for node in toexplore[:]:  # Iterate over a copy of the list
        if len(node["lines"]) == 4:  # Assuming a node is complete if it has lines in all four directions
            Nodes.append(node)
            toexplore.remove(node)
            print(f"Node at {node['location']} is now complete and added to Nodes list.")

    


def exploreNode(node_location=None):

    # Get the current node from the Nodes list
    current_node = next((node for node in Nodes if node["location"] == node_location), None)
    
    if current_node is None:
        print("Error: Current node not found in Nodes list.")
        return
    
    for direction in range(4):  # Check all four directions
        
        # Capture camera image and find line
        line_position, line_width = find_line(camera.getImageArray()[camera.getHeight()//2])
        
        # Characterize the line based on robot's orientation
        line_characterization = caracterize_line(line_position, line_width)

        if line_characterization not in current_node["lines"]:
            current_node["lines"].append(line_characterization)

        rotate_90()  # Rotate the robot 90 degrees to check the next direction

import math


def follow_line(line_position, line_width):

    # if line_position < 0 or line_width <= 0:
    #     left_motor.setVelocity(0)
    #     right_motor.setVelocity(0)
    #     return

    # Find the center of the line and camera
    line_center = line_position + line_width / 2
    camera_center = camera.getWidth() / 2

    # Calculate the error
    error = line_center - camera_center

    # Adjust wheel speeds to center the line
    speed = 5.0
    correction = 0.02 * error

    left_motor.setVelocity(speed + correction)
    right_motor.setVelocity(speed - correction)


def move_direction(direction):
    # Define target orientations (in radians)
    directions = {
        "n": 0.0,
        "e": math.pi / 2,
        "s": math.pi,
        "w": -math.pi / 2
    }
    initial_gps = gps.getValues()

    # Validate the input
    direction = direction.lower()
    if direction not in directions:
        print("Invalid direction. Choose n, e, s, or w.")
        return

    target_orientation = directions[direction]

    # Rotate until the robot faces the target direction
    while True:
        compass_values = compass.getValues()
        current_orientation = math.atan2(
            compass_values[1], compass_values[0]
        )

        # Calculate the shortest angular difference
        angle_difference = math.atan2(
            math.sin(target_orientation - current_orientation),
            math.cos(target_orientation - current_orientation)
        )

        # Check if the target orientation is reached
        if abs(angle_difference) < 0.01:
            break

        # Rotate in the correct direction
        rotation_speed = 1.0
        if angle_difference > 0:
            left_motor.setVelocity(rotation_speed)
            right_motor.setVelocity(-rotation_speed)
        else:
            left_motor.setVelocity(-rotation_speed)
            right_motor.setVelocity(rotation_speed)

        robot.step(timestep)

    #if N:
    if direction == "n":
        target_location = (initial_gps[0], round((initial_gps[1] + 0.15) / 0.15) * 0.15)
    elif direction == "e":
        target_location = (round((initial_gps[0] + 0.15) / 0.15) * 0.15, initial_gps[1])
    elif direction == "s":
        target_location = (initial_gps[0], round((initial_gps[1] - 0.15) / 0.15) * 0.15)
    elif direction == "w":
        target_location = (round((initial_gps[0] - 0.15) / 0.15) * 0.15, initial_gps[1])

    target_location = tuple(round(coord, 2) for coord in target_location)

    # add 0.15 to the gps location to move the robot forward in the facing direction
    while True:
        gps_values = gps.getValues()
        # print("Current GPS:", gps_values, "Target GPS:", target_location)

        # Check if the robot has reached the target location
        if direction == "n" or direction == "s":
            if abs(gps_values[1] - target_location[1]) < 0.01:
                break
        else:  # direction == "e" or direction == "w"
            if abs(gps_values[0] - target_location[0]) < 0.01:
                break

         # Capture camera image and find line
        line_position, line_width = find_line(camera.getImageArray()[camera.getHeight()//2])
                

        # Move forward
        follow_line(line_position, line_width)

        robot.step(timestep)

    left_motor.setVelocity(0.0)
    right_motor.setVelocity(0.0)

def bfs(start, goal):
    """Find a path between nodes using the discovered line graph.

    Returns a list of directions, for example ['N', 'E', 'S'].
    Returns None when no path exists.
    """
    if start == goal:
        return []

    movements = {
        "N": (0, 1),
        "E": (1, 0),
        "S": (0, -1),
        "W": (-1, 0),
    }

    queue = deque([(start, [])])
    visited = {start}

    while queue:
        current_node, path = queue.popleft()

        node_data = next(
            (node for node in Nodes if node["location"] == current_node),
            None
        )

        if node_data is None:
            continue

        for line in node_data["lines"]:
            if len(line) < 2 or line[0] != "l":
                continue

            direction = line[1].upper()

            if direction not in movements:
                continue

            dx, dy = movements[direction]
            next_node = (
                current_node[0] + dx,
                current_node[1] + dy
            )

            next_path = path + [direction]

            if next_node == goal:
                return next_path

            if next_node not in visited:
                visited.add(next_node)
                queue.append((next_node, next_path))

    return None


def moveto_node(node_location, next_node_location):
    """Move from one node to another using the BFS-generated path."""
    path = bfs(node_location, next_node_location)

    if path is None:
        print(
            "No path found from",
            node_location,
            "to",
            next_node_location
        )
        return

    print("Path:", path)

    for direction in path:
        move_direction(direction)

def printmap():
    # map is 21 lines height and 49 characters width, with the origin at the location (11,25)

    # Create a grid representation of the map
    grid = [[" " for _ in range(50)] for _ in range(22)]

    for node in Nodes:
        x, y = node["location"]
        # Normalize the coordinates to fit the grid
        grid_x = int((x * 2) + 25)  # Adjust based on your coordinate system
        grid_y = -int((y * 2 ) + 11)  # Adjust based on your coordinate system

        for line in node["lines"]:
            if len(line) < 2:
                continue

            line_type, direction = line[0], line[1]

            if line_type == "l":
                if direction == "N":
                    if grid_y > 0:
                        grid[grid_y - 1][grid_x] = "|"
                elif direction == "E":
                    if grid_x < 48:
                        grid[grid_y][grid_x + 1] = "-"
                elif direction == "S":
                    if grid_y < 20:
                        grid[grid_y + 1][grid_x] = "|"
                elif direction == "W":
                    if grid_x > 0:
                        grid[grid_y][grid_x - 1] = "-"

    grid[11][25] = "I"  # Mark the origin
    # Print the grid
    for row in grid:
        print("".join(row))


# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    if initial_gps is None:
        initial_gps = gps.getValues()
        print('initial_gps', initial_gps)
        print(find_line(camera.getImageArray()[camera.getHeight()//2]))
        
    #print("customData",robot.getCustomData())
    gps_values = gps.getValues()
    compass_values = compass.getValues()
    #Normalize the gps location to the initial position
    node_location = tuple(round(round((gps_values[i] - initial_gps[i]) / 0.05) * 0.05 / 0.15)for i in range(2) )
    print ("Current node location:", node_location)
    print(Nodes, toexplore)
    if node_location not in [Nodes[i]["location"] for i in range(len(Nodes))]:
        #append an empty node
        Nodes.append({"location": node_location, "lines": []})
        exploreNode(node_location)
        infereNodes(node_location)

    #check if there's any node to explore
    if toexplore:
    
        printmap()
        next_node_location = toexplore.pop()["location"]
        # check if node has valid lines

        # print("Next node to explore:", next_node_location)

        moveto_node(node_location, next_node_location)
        exploreNode(next_node_location)
    else:
        print("All nodes explored. No more nodes to explore.")
        break
        
    

          # Exit after one iteration for testing purposes
        
print("Nodes", Nodes)
print("toexplore", toexplore)

# Enter here exit cleanup code.