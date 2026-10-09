"""
*****************************************************************************************
*
*        =================================================
*             Pac Bot (EB) Theme (eYRC 2026-27)
*        =================================================
*
*  This script is to implement Task 1B of Pac Bot (EB) Theme (eYRC 2026-27).
*
******************************************************************************************
"""

# Team ID:          2243
# Theme:            PacBot
# Author List:      [ Akanksha Singh, Kirtee Mishra, Koshika Verma, Srinidhi Sivakumar ]
# Filename:         task_1b.py
# Functions:        _mqtt_client, on_message, main
# Global variables: MQTT_HOST, MQTT_PORT, TOPIC_SENSORS, TOPIC_WHEEL_VEL


import json

import paho.mqtt.client as mqtt

MQTT_HOST = "localhost"
MQTT_PORT = 1883
TOPIC_SENSORS = "pacbot/sensors"      # simulator publishes, this file subscribes
TOPIC_WHEEL_VEL = "pacbot/wheel_vel"  # this file publishes, simulator subscribes

# DEG_TO_RAD: Conversion factor (pi / 180) to convert degrees to radians without extra imports
DEG_TO_RAD = 0.017453292519943295


def _mqtt_client():
    # paho-mqtt >= 2.0 requires picking a callback API version explicitly.
    try:
        return mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    except AttributeError:
        return mqtt.Client()

def on_message(client, userdata, msg):
    '''
    Purpose:
    ---
    Reads distance and gyro readings, figures out whether to steer straight,
    hug a wall, or turn at a dead end, and sends wheel speeds back to the simulation.

    Input Arguments:
    ---
    `client` :  [ paho.mqtt.client.Client ]
    MQTT client object handling the connection.

    `userdata` :  [ Any ]
    User data passed to the callback if set.

    `msg` :  [ paho.mqtt.client.MQTTMessage ]
    Incoming MQTT packet containing JSON sensor data.

    Returns:
    ---
    None

    Example call:
    ---
    client.on_message = on_message
    '''
    # unpack sensor packet from simulator
    data = json.loads(msg.payload.decode())

    fl = data["fl"]            # Front-left ToF distance readings
    fr = data["fr"]            # Front-right ToF distance readings
    sl = data["sl"]            # Side-left ToF distance readings 
    sr = data["sr"]            # Side-right ToF distance readings 
    yaw_rate = data["gyro"][2]  # rad/s about z
    dt = data["dt"]            # s, simulator timestep


    print(f"fl={fl:.3f} fr={fr:.3f} sl={sl:.3f} sr={sr:.3f} "
          f"yaw_rate={yaw_rate:+.3f} dt={dt:.4f}")

    
    left_vel = 0.0
    right_vel = 0.0

 
 # ---------------------------------TUNED GAINS & SETPOINTS --------------------------------------

    
    FOLLOW = "L"      # We bias turning left whenever an intersection gives us multiple choices.  
    BASE = 8.0        # 9.0 rad/s is our baseline forward speed where the robot stays fast but stable.
    KP = 50.0         # KP needs to be aggressive enough (50.0) to shove the robot back to center in narrow corridors.
    KI = 1.0          # KI accumulates tiny persistent offsets, especially if friction or weight leans slightly one way.
    KD = 0.5          # D uses the gyro (yaw rate)
    SIDE_WALL = 0.15    # Any reading beyond 0.15 m means that wall has ended and we are entering an opening.
    SIDE_TARGET = 0.10  # distance to keep from a single wall
    FRONT_STOP = 0.08   # front reading below this = stop and turn
    TURN_SPEED = 2.0    # wheel speed while spinning in place (rad/s)
    YAW_SIGN = 1.0     # Multiplier to reconcile the gyro's coordinate system with our clockwise/counterclockwise conventions.
    MAX_STEP = 0.05    # max wheel-speed change per message
    
    # If the simulator glitches and passes a zero or negative timestep, we substitute
    # a safe 2 ms fallback to prevent division by zero or broken integrals.
    if dt <= 0:
        dt = 0.002
 
    # Because ToF sensors become unreliable and noisy at longer ranges, and because walls further
    # than 30 cm don't matter for corridor navigation, we clip all readings at 0.30 m.
    # sl and sr measure the path ahead, while fl and fr measure the walls beside us.
    front = min(sl, sr, 0.30)
    left = min(fl, 0.30)
    right = min(fr, 0.30)

    # When a sensor beam drops out or hits an unreflective surface, it produces NaN or infinite values.
    # Because NaN never equals itself, x != x catches missing data, while the infinity check prevents
    # mathematical overflow from corrupting our steering error.
    if (front != front) or abs(front) == float('inf'): front = 0.30
    if (left != left) or abs(left) == float('inf'): left = 0.30
    if (right != right) or abs(right) == float('inf'): right = 0.30

    # Instead of relying on global variables, we attach our persistent state dictionary
    # directly to the function object so it survives between MQTT message calls.
    s = getattr(on_message, "s", None)
    if s is None:
        s = on_message.s = {"mode": "DRIVE", "integ": 0.0, "turned": 0.0,
                            "goal": 0.0, "L": 0.0, "R": 0.0, "n": 0}
    s["n"] += 1
 
    # This branch handles normal forward movement and corridor steering.
    if s["mode"] == "DRIVE":

        # A wall has appeared within our front safety boundary, meaning we must stop and pivot.
        if front < FRONT_STOP:

            # We inspect both flanks to see which directions are open corridors.
            open_l, open_r = left > SIDE_WALL, right > SIDE_WALL

            # We pick the turn angle according to our left-first strategy, falling back
            # to a 180-degree turnaround if we've driven into a dead-end pocket.
            if FOLLOW == "L":
                goal = 90 if open_l else (-90 if open_r else 180)
            else:
                goal = -90 if open_r else (90 if open_l else 180)

            # We switch modes, convert degrees to radians manually, and reset the accumulators.
            s.update(mode="TURN", turned=0.0, goal=goal * DEG_TO_RAD, integ=0.0)

        else:

            # We check whether we have walls on both sides, just one side, or neither
            wall_l, wall_r = left < SIDE_WALL, right < SIDE_WALL

            if wall_l and wall_r:

                # With walls on both sides, our target is the exact midpoint between them.
                # If left > right, err is positive, which tells the robot it is too far right.
                err = (left - right) / 2   

            elif wall_l:
                # Only a left wall exists, so we measure our offset from our 10 cm target line
                err = left - SIDE_TARGET        

            elif wall_r:
                # Only a right wall exists, so we invert the sign to keep coordinate consistency
                err = SIDE_TARGET - right   

            else:
                # In an open junction without walls, we have no lateral reference, so error is zero
                err = 0.0   

            # We accumulate error for the I-term, but clamp it between -0.05 and 0.05
            # so integral windup doesn't cause massive overshoots after long straightaways
            s["integ"] = max(-0.05, min(0.05, s["integ"] + err * dt))

            # Our PID formula combines error correction (P), steady-state trimming (I),
            # and gyro-based rotational damping (D)
            steer = KP * err + KI * s["integ"] - KD * YAW_SIGN * yaw_rate

            # We clamp steer to the base speed so wheel velocity never turns negative while driving forward
            steer = max(-BASE, min(BASE, steer))  

            # Differential drive geometry: positive steer slows the left wheel and speeds up the right,
            # swinging the front of the robot toward the left
            left_vel = BASE - steer
            right_vel = BASE + steer
 
    # This branch executes closed-loop in-place rotations when a turn is triggered
    if s["mode"] == "TURN":

        # We integrate angular velocity over the slice of time dt to track total degrees turned
        s["turned"] += YAW_SIGN * yaw_rate * dt
        remaining = s["goal"] - s["turned"]


       # 3 degrees (~0.0524 rad) is tight enough to align with the next hallway without hunting back and forth
        if abs(remaining) < (3.0 * DEG_TO_RAD):

            # Once aligned, we reset wheel speeds to zero and resume driving mode
            s["mode"] = "DRIVE"
            left_vel = right_vel = 0.0

        else:

            # Spin direction is set by the sign of the remaining angle
            w = TURN_SPEED if remaining > 0 else -TURN_SPEED

            # When within 20 degrees (~0.349 rad) of finishing, we cut speed to 40% so momentum doesn't overshoot
            if abs(remaining) < (20.0 * DEG_TO_RAD):
                w *= 0.4

            # Opposing wheel velocities create a zero-radius spin in place
            left_vel, right_vel = -w, w

    # Instant velocity jumps cause the robot's front wheels to lift or slip on the ground plane.
    # Clamping the per-tick delta to MAX_STEP enforces smooth acceleration ramping.
    left_vel = s["L"] + max(-MAX_STEP, min(MAX_STEP, left_vel - s["L"]))
    right_vel = s["R"] + max(-MAX_STEP, min(MAX_STEP, right_vel - s["R"]))

    # We store the final ramped velocities so the next tick can ramp from this baseline
    s["L"], s["R"] = left_vel, right_vel

    # Finally, we serialize and publish the calculated wheel speeds to the simulator
    client.publish(TOPIC_WHEEL_VEL, json.dumps({
        "left": float(left_vel), "right": float(right_vel),
    }))


def main():
    client = _mqtt_client()
    client.on_message = on_message
    client.connect(MQTT_HOST, MQTT_PORT)
    client.subscribe(TOPIC_SENSORS)
    client.loop_forever()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
