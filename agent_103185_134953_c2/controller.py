"""c2_controller"""

from controller import Robot
import math
from collections import deque

# ROBOT DEFINITIONS
# You should insert a getDevice-like function in order to get the
# instance of a device of the robot.
robot = Robot()
timestep = int(robot.getBasicTimeStep())

gps = robot.getDevice('gps')
camera = robot.getDevice('camera')
compass = robot.getDevice('compass')
left_motor = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')

gps.enable(timestep)
camera.enable(timestep)
compass.enable(timestep)
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))

initial_coordinates = None


# NAVIGATION DEFINITIONS
MapNodes = []
ExplorationStack = []

# # Line-following PD controller parameters
# Kp = 0.15
# Kd = 0.05
# exponent = 1.0
# cruiseVelocity = 3.0
# maxCorrection = 2.0
# maxVelocity = 6.28
# previous_error = 0.0
# last_direction = 0
# width = camera.getWidth()

# ROBOT CONTROL METHODS
# -----------------------------

def rotate_90():
    """ Turns the robot 90 degrees."""
    
    print("Rotating the robot 90 degrees...")
    
    compass_values = compass.getValues()
    current_orientation = math.atan2(compass_values[1], compass_values[0])
  
    print('current_orientation', current_orientation)
    
    # Calculate the target orientation (90 degrees from current)
    target_orientation = current_orientation + math.pi / 2

    # Normalize the target orientation to be within -pi to pi
    if target_orientation > math.pi: target_orientation -= 2 * math.pi

    # Rotate the robot until it reaches the target orientation
    while True:
        compass_values = compass.getValues()
        current_orientation = math.atan2(compass_values[1], compass_values[0])

        if abs(current_orientation - target_orientation) < 0.01:  # Allow a small margin of error
            break

        # Set motor speeds to rotate the robot
        # TODO this is a placeholder, actual motor control code needed
        left_motor.setVelocity(1.0)
        right_motor.setVelocity(-1.0)

        robot.step(timestep)

    left_motor.setVelocity(0.0)
    right_motor.setVelocity(0.0)


def find_line():
    """ Uses bottom half of camera to find line."""
    
    image = camera.getImageArray()[camera.getHeight()//2]
    difference_threshold = 100
    brightness = [sum(pixel) for pixel in image]
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
        line_width = len(image) - falling_edge

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

#---
#        |
#      -- --
#        |
#---
def caracterize_line(line_position, line_width):
    """Checks if node contains line to another node."""
    
    compass_values = compass.getValues()

    # get the vector with the most absolute value, this is the direction the robot is facing
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


def follow_line(line_position, line_width):
    """Centers Robot on line using the middle of camera."""

    line_center = line_position + line_width / 2
    camera_center = camera.getWidth() / 2
    error = line_center - camera_center

    # Adjust wheel speeds to center the line
    # TODO make both constants
    speed = 5.0
    correction = 0.02 * error

    left_motor.setVelocity(speed + correction)
    right_motor.setVelocity(speed - correction)


def move_direction(direction):
    """Centers Robot on line using the middle of camera."""
    
    # TODO make this an enum
    directions = {
        "n": 0.0,
        "e": math.pi / 2,
        "s": math.pi,
        "w": -math.pi / 2
    }
    
    initial_coordinates = gps.getValues()
    direction = direction.lower()

    # Validate the input
    if direction not in directions:
        print("Invalid direction. Choose n, e, s, or w.")
        return

    target_orientation = directions[direction]

    # TODO this part rotate_90() in a better way, may extract.
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

    if direction == "n":
        target_location = (initial_coordinates[0], round((initial_coordinates[1] + 0.15) / 0.15) * 0.15)
    elif direction == "e":
        target_location = (round((initial_coordinates[0] + 0.15) / 0.15) * 0.15, initial_coordinates[1])
    elif direction == "s":
        target_location = (initial_coordinates[0], round((initial_coordinates[1] - 0.15) / 0.15) * 0.15)
    elif direction == "w":
        target_location = (round((initial_coordinates[0] - 0.15) / 0.15) * 0.15, initial_coordinates[1])

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

        line_position, line_width = find_line()
        follow_line(line_position, line_width)
        robot.step(timestep)

    left_motor.setVelocity(0.0)
    right_motor.setVelocity(0.0)


def inferNodes(node_location=None):
    """Populates node info"""
    
    current_node = next((node for node in MapNodes if node["location"] == node_location), None)
    
    # Check the existing lines in the current node
    existing_lines = current_node["lines"]

    # Infer the presence of lines in the adjacent nodes based on the current node's lines
    # TODO use gs0, gs2 and camera for this
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
        
        if any(node["location"] == adjacent_node_location for node in MapNodes):
            continue

        # Check if the node is on the list of the explored nodes if not, add it to the list of nodes to explore
        adjacent_node = next((node for node in MapNodes if node["location"] == adjacent_node_location), None)
        if adjacent_node is None:
            # If the adjacent node is not in the Nodes list, add it to the toexplore list with the inferred line type
            if adjacent_node_location not in ExplorationStack:
                ExplorationStack.append({"location": adjacent_node_location, "lines": [adj_line_type]})
        else:
            # If the adjacent node is already in the Nodes list, update its lines if necessary
            if line_type[0] not in adjacent_node["lines"]:
                adjacent_node["lines"].append(adj_line_type)

    #check if there's any toexplore node that became complete, if so, remove it from the toexplore list add it to the Nodes list
    for node in ExplorationStack[:]:  # Iterate over a copy of the list
        if len(node["lines"]) == 4:  # Assuming a node is complete if it has lines in all four directions
            MapNodes.append(node)
            ExplorationStack.remove(node)
            print(f"Node at {node['location']} is now complete and added to Nodes list.")


# NAVIGATION METHODS
# -----------------------------

def exploreNode(node_location = None):
    """Add lines from/to current node."""
    current_node = next((node for node in MapNodes if node["location"] == node_location), None)
    
    if current_node is None:
        print("Error: Current node not found in Nodes list.")
        return
    
    # TODO sub this to gs0 + gs2 + camera
    for direction in range(4):  # Check all four directions
        
        # Capture camera image and find line
        line_position, line_width = find_line()
        
        # Characterize the line based on robot's orientation
        line_characterization = caracterize_line(line_position, line_width)

        if line_characterization not in current_node["lines"]:
            current_node["lines"].append(line_characterization)

        rotate_90()  # Rotate the robot 90 degrees to check the next direction


def dfs(start, destination):
    """Find a path between nodes using the discovered line graph.

    Returns a list of directions, for example ['N', 'E', 'S'].
    Returns None when no path exists.
    """
    if start == destination:
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

        node_data = next((node for node in MapNodes if node["location"] == current_node), None)

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

            if next_node == destination:
                return next_path

            if next_node not in visited:
                visited.add(next_node)
                queue.append((next_node, next_path))

    return None


def navigate_to_node(current_node, destination_node):
    """Move from one node to another using the DFS-generated path."""
    path = dfs(current_node, destination_node)

    if path is None:
        print("No path found from", current_node, "to", destination_node)
        return

    print("Path:", path)

    for direction in path: move_direction(direction)


# PRINTING METHODS
# -----------------------------

def printmap():
    """Create a grid representation of the map"""
    # map is 21 lines height and 49 characters width, with the origin at the location (11,25)

    grid = [[" " for _ in range(50)] for _ in range(22)]

    for node in MapNodes:
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


def main():
    global initial_coordinates
    
    # TODO
    # 1. configure robot, initial position and next nodes
    
    # 2. While there are nodes to explore:
    
        # a. Control robot to next node
            #   i. STRAIGTH = motors
            #  ii. TURN
            # iii. STOP
            #  iv. LOST
        # b. Camera: use to anticipate turn and mark node as goal (straight)
        # c. If GS0 and GS2 see black, add to next nodes to stack
        # d. If all GSx see white = dead end
        # e. add to next nodes to stack
        # f. Pop next node to explore 
        # g. find shortest path to next node
    
    # TODO: steps by nodes left
    while robot.step(timestep) != -1:
            
        if initial_coordinates is None:
            initial_coordinates = gps.getValues()
            print('initial_coordinates', initial_coordinates)
            print(find_line())
            
        current_coordinates = gps.getValues()
        current_direction = compass.getValues()
        # Normalize the gps location to the initial position
        current_node = tuple(round(round((
            current_coordinates[i] - initial_coordinates[i]) / 0.05) * 0.05 / 0.15) for i in range(2))
        
        print ("Current node location:", current_node)
        print(MapNodes, ExplorationStack)
        
        if current_node not in [MapNodes[i]["location"] for i in range(len(MapNodes))]:
            # append an new empty node to be explored
            MapNodes.append({"location": current_node, "lines": []})
            exploreNode(current_node)
            inferNodes(current_node)

        # check next node to explore
        if ExplorationStack:
            printmap()
            destination_node = ExplorationStack.pop()["location"]
            navigate_to_node(current_node, destination_node)
            exploreNode(destination_node)
            
        else:
            print("All nodes explored. No more nodes to explore.")
            break

            # Exit after one iteration for testing purposes
            
    print("Nodes", MapNodes)
    print("toexplore", ExplorationStack)

main()