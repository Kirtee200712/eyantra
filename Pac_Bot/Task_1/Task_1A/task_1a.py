#!/usr/bin/env python3
"""
*****************************************************************************************
*
*        =================================================
*             Pac Bot (EB) Theme (eYRC 2026-27)
*        =================================================
*
*  This script is to implement Task 1A of Pac Bot (EB) Theme (eYRC 2026-27).
*
******************************************************************************************
"""

# Team ID:          [ 2243 ]
# Author List:      [ Akanksha Singh, Kirtee Mishra, Koshika Verma, Srinidhi Sivakumar ]
# Filename:         Task1A.py
# Functions:        choose_command, parse_pellets,
# Global variables: MAZE_ROWS, MAZE_COLS, MQTT_BROKER, POSE_TOPIC, WALL_N, WALL_E, WALL_S,WALL_W,
#                   WALLS, EXIT_CELLS, BOT_CMD_TOPIC, PELLETS_TOPIC, CMD_VEL_TOPIC, HEADING_DELTA             


import json
import time
from tkinter import LEFT, RIGHT

import paho.mqtt.client as mqtt

MAZE_ROWS = 13
MAZE_COLS = 13
MQTT_BROKER = "localhost" 
MQTT_PORT = 1883
POSE_TOPIC = "robot/pose"

# wall bit per side, OR'd together
WALL_N, WALL_E, WALL_S, WALL_W = 0x1, 0x2, 0x4, 0x8

WALLS = [
    [12, 6, 12, 6, 13, 4, 0, 4, 5, 6, 12, 5, 6],
    [10, 11, 10, 10, 12, 3, 8, 2, 12, 1, 1, 6, 10],
    [8, 5, 3, 8, 1, 6, 9, 2, 9, 6, 13, 2, 10],
    [10, 12, 4, 3, 12, 1, 4, 0, 6, 9, 4, 2, 10],
    [10, 10, 8, 5, 3, 13, 2, 10, 9, 6, 10, 11, 10],
    [8, 3, 9, 4, 5, 6, 8, 1, 6, 10, 9, 5, 2],
    [10, 12, 4, 1, 6, 10, 9, 6, 10, 8, 5, 5, 2],
    [8, 1, 2, 12, 3, 10, 12, 3, 9, 2, 12, 6, 10],
    [9, 6, 10, 10, 12, 1, 2, 12, 5, 1, 0, 1, 3],
    [14, 8, 1, 1, 3, 12, 1, 3, 12, 4, 2, 12, 6],
    [8, 0, 4, 7, 12, 3, 12, 6, 10, 9, 1, 2, 10],
    [10, 10, 9, 6, 10, 12, 2, 8, 1, 7, 12, 0, 2],
    [9, 1, 5, 1, 1, 3, 8, 1, 5, 5, 3, 9, 3],
]
# Maze map -- the same data as WALLS above, drawn. row 0 = SOUTH (bottom),
# col 0 = WEST (left). The gaps in the top and bottom edges are EXIT_CELLS.
#
#            0  1  2  3  4  5  6  7  8  9 10 11 12   <- col
#          +--+--+--+--+--+--+  +--+--+--+--+--+--+
#   row 12 |                 |              |     |
#          +  +  +--+  +  +  +  +  +--+--+  +  +  +
#   row 11 |  |  |     |  |     |        |        |
#          +  +  +  +--+  +--+  +  +  +--+--+  +  +
#   row 10 |           |     |     |  |        |  |
#          +  +  +--+--+--+  +--+--+  +  +  +  +  +
#   row  9 |  |           |        |        |     |
#          +--+  +  +  +  +--+  +  +--+--+  +--+--+
#   row  8 |     |  |  |        |                 |
#          +  +--+  +  +--+  +  +--+--+  +  +  +  +
#   row  7 |        |     |  |     |     |     |  |
#          +  +  +  +--+  +  +--+  +  +  +--+--+  +
#   row  6 |  |           |  |     |  |           |
#          +  +--+--+  +--+  +  +--+  +  +--+--+  +
#   row  5 |     |           |        |  |        |
#          +  +  +  +--+--+--+  +  +--+  +  +--+  +
#   row  4 |  |  |        |     |  |     |  |  |  |
#          +  +  +  +--+  +--+  +  +  +--+  +  +  +
#   row  3 |  |        |              |        |  |
#          +  +--+--+  +--+  +--+  +--+  +--+  +  +
#   row  2 |        |        |     |     |     |  |
#          +  +--+  +  +  +--+  +  +  +--+--+  +  +
#   row  1 |  |  |  |  |     |     |           |  |
#          +  +  +  +  +--+  +  +  +--+  +  +--+  +
#   row  0 |     |     |                 |        |
#          +--+--+--+--+--+--+  +--+--+--+--+--+--+
#            0  1  2  3  4  5  6  7  8  9 10 11 12   <- col

# the 2 known exits: (row, col, facing)
EXIT_CELLS = [
    (0, 6, 'south'),
    (MAZE_ROWS - 1, 6, 'north'),
]

BOT_CMD_TOPIC = "bot/cmd"         
PELLETS_TOPIC = "pellets/pose"    
CMD_VEL_TOPIC = "robot/cmd_vel"  

# yaw -> dr, dc, wall bit. 0=EAST, 90=NORTH, 180=WEST, 270=SOUTH
HEADING_DELTA = {
    0.0:   (0, 1, WALL_E),
    90.0:  (1, 0, WALL_N),
    180.0: (0, -1, WALL_W),
    270.0: (-1, 0, WALL_S),
}


def choose_command(pacbot_cell, pacbot_yaw, pellets_remaining):
    """
    Purpose:
    --
    Calculates and returns the immediate single-step directional movement 
    for the bot using A* search. It determines the globally shortest sequence 
    to pick up both pallets (evaluating Pallet 1 -> Pallet 2 vs. Pallet 2 -> Pallet 1) 
    and then route to the maze exit, returning only the next move.

    Input:
    ---
    pacbot_cell, pacbot_yaw, pellets_remaining

    Output:
    ---
    Returns the next movement command for the bot as a string ("FRONT", "LEFT", "BACK", or None).

    Example call:
    ---
    choose_command(pacbot_cell, pacbot_yaw, pellets_remaining)
    """
    # Check if at an exit threshold once all pellets are collected
    if not pellets_remaining:
        dir_to_yaw = {'north': 90.0, 'south': 270.0, 'east': 0.0, 'west': 180.0}
        for ex_r, ex_c, ex_dir in EXIT_CELLS:
            if pacbot_cell == (ex_r, ex_c):
                target_yaw = dir_to_yaw[ex_dir]
                diff = float((target_yaw - pacbot_yaw) % 360)
                if abs(diff) > 1e-3:
                    return {0.0: "FRONT", 90.0: "LEFT", 180.0: "BACK", 270.0: "RIGHT"}.get(diff, "FRONT")
                return "FRONT"

    # Select goals: collect pellets first, then exit once all pellets are collected
    if pellets_remaining: 
        goals = set(pellets_remaining)  # Set the goal coordinates to the remaining pallet locations
    # If all pellets have been collected    
    else:
        # Switch goal to the exit coordinates.
        goals = {(r, c) for r, c, _ in EXIT_CELLS} 

    # Stop condition: Return None if no goals exist
    if not goals:
        return None

    # Function Name: heuristic()
    # Input:         cell - a tuple (row, col)
    # Output:        the heuristic value for the cell
    # Purpose:       This function calculates the heuristic value for a given cell based 
    #                on the Manhattan distance to the nearest goal. The heuristic is used 
    #                in the A* search algorithm to estimate the cost from the current cell to the goal.
    def heuristic(cell):
        r, c = cell
        return min(abs(r - gr) + abs(c - gc) for gr, gc in goals) # Returns Manhattan distance to the nearest goal

    # A* search to find the next cell to move to
    start = pacbot_cell  # Initialize the starting position by bot's current coordinates

    # Initialize the frontier with starting points to be evaluated
    frontier = [start]

    # Initialize the lookup dictionary to store parent cell
    came_from = {}

    # Cost from start to start is zero
    g_score = {start: 0}

    # Initial estimated total cost f(start) = g(start)+ h(start)
    f_score = {start: heuristic(start)}

    goal_reached = None

    while frontier:
        # Pop the node with the lowest f_score
        current = min(frontier, key=lambda cell: f_score.get(cell, float("inf")))
        frontier.remove(current)

        # Check if the popped node is one of our target nodes and not the node we started on
        if current in goals and current != start:
            goal_reached = current
            # Exit the search loop early
            break

        cr, cc = current  # current row and column value
        cell_walls = WALLS[cr][cc] # Get the wall bitmask integer for the current cell

        # Explore all four orthogonal directions
        for yaw_deg, (dr, dc, wall_bit) in HEADING_DELTA.items():
            # Check if a wall exist blocking movement in this direction
            if cell_walls & wall_bit:
                continue

            # Calculate adjacent neighbor coordinates by adding directional offsets to current coordinates
            nr, nc = cr + dr, cc + dc

            # Verify the adjacent cell in within maze boundary
            if 0 <= nr < MAZE_ROWS and 0 <= nc < MAZE_COLS:
                neighbor = (nr, nc) 

                # Calculate tentative step cost g_score(neighbor)= g_score(current) + 1
                cost = g_score[current] + 1

                # Check if new route to neighbor is cheaper
                if cost < g_score.get(neighbor, float("inf")):
                    # Record current node as the optimal predecessor for this neighbor
                    came_from[neighbor] = current

                    # Store updated lowest path cost
                    g_score[neighbor] = cost

                    # Update total estimated cost
                    f_score[neighbor] = cost + heuristic(neighbor)

                    # If neighbor isn't already queued for evaluation, add it to the frontier
                    if neighbor not in frontier:
                        frontier.append(neighbor)

    # No valid route found
    if not goal_reached:
        return None

    # Begin tracing the path backward starting from goal cell reached
    curr = goal_reached

    # Backtrack along came_from links until predecessor of curr is the start cell
    while came_from.get(curr) != start:
        # Move one step back along the path towards the start node
        curr = came_from.get(curr)

        # Safety check: return none if path linkage is broken unexpectedly
        if curr is None:
            # Traversal broken
            return None
        
    next_cell = curr # Store immediately adjacent step found during backtracking as next target cell

    # Translate cell delta into target world yaw
    # Calculate change in row index needed to reach the next cell from current cell
    dr = next_cell[0] - start[0]
    # Calculate change in column index needed to reach the next cell from current cell
    dc = next_cell[1] - start[1]

    desired_yaw = None # Initialize a variable to hold the required yaw
    for yaw_deg, (hdr, hdc, _) in HEADING_DELTA.items():
        # Check If this heading's directional offset matches our (dr, dc) 
        if (hdr, hdc) == (dr, dc):
            desired_yaw = yaw_deg
            break

    if desired_yaw is None:
        return None

    # 6. Convert relative orientation to movement command
    offset = float((desired_yaw - pacbot_yaw) % 360)
    directions = {
        0.0: "FRONT",
        90.0: "LEFT",
        180.0: "BACK",
        270.0: "RIGHT"        
    }

    return directions.get(offset, None)
# ============================================================================


def parse_pellets(payload):
    return {tuple(cell) for cell in json.loads(payload)}


def main():
    state = {
        "running": False,
        "pellets": set(),
        "cell": (MAZE_ROWS // 2, MAZE_COLS // 2),
        "yaw": 0.0,
        "in_flight": False,
        "got_pose": False,
        "got_pellets": False,
    }

    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="Controller")

    def decide_and_send():
        if (not state["running"] or state["in_flight"]
                or not state["got_pose"] or not state["got_pellets"]):
            return
        print(f"[debug] pose={state['cell']} yaw={state['yaw']} pellets={state['pellets']}")
        cmd = choose_command(state["cell"], state["yaw"], set(state["pellets"]))
        if cmd is not None:
            state["in_flight"] = True
            client.publish(CMD_VEL_TOPIC, cmd)
            print(f"[controller] {state['cell']} yaw={state['yaw']} -> {cmd}, "
                  f"pellets_left={len(state['pellets'])}")

    def on_message(client, userdata, msg):
        try:
            if msg.topic == BOT_CMD_TOPIC:
                running = msg.payload.decode().startswith("1")
                was_running = state["running"]
                state["running"] = running
                if running and not was_running:
                    decide_and_send()   # kick off the reactive loop on Start
            elif msg.topic == PELLETS_TOPIC:
                state["pellets"] = parse_pellets(msg.payload.decode())
                state["got_pellets"] = True
                decide_and_send()
            elif msg.topic == POSE_TOPIC:
                data = json.loads(msg.payload.decode())
                state["cell"] = (int(data["col"]), int(data["row"]))   # wire is swapped
                state["got_pose"] = True
                state["yaw"] = float(data.get("yaw", 0.0))
                state["in_flight"] = False   # this pose is the ack for our last command
                decide_and_send()   # every pose/command-ack triggers the next step
        except Exception as e:
            print("[controller] mqtt parse error:", e)

    client.on_message = on_message
    client.connect(MQTT_BROKER, MQTT_PORT, 60)
    client.subscribe([(BOT_CMD_TOPIC, 0), (PELLETS_TOPIC, 0), (POSE_TOPIC, 0)])
    client.loop_start()

    print(f"[controller] ready; sending one '{CMD_VEL_TOPIC}' command at a time, "
          f"reacting to '{POSE_TOPIC}'/'{PELLETS_TOPIC}' feedback")

    try:
        while True:
            time.sleep(0.2)  
    except KeyboardInterrupt:
        pass
    finally:
        client.loop_stop()
        client.disconnect()


if __name__ == "__main__":
    main()
