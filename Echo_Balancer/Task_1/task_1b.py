"""Boilerplate for PB Task 1B.

Subscribes to the simulator's sensor topic, logs each reading, and publishes
a wheel velocity command back. Fill in your control logic where marked.

Run (three terminals):
    mosquitto
    ./task_1b_launch
    python3 task_1b_boilerplate.py
"""
import json

import paho.mqtt.client as mqtt

MQTT_HOST = "localhost"
MQTT_PORT = 1883
TOPIC_SENSORS = "pacbot/sensors"      # simulator publishes, this file subscribes
TOPIC_WHEEL_VEL = "pacbot/wheel_vel"  # this file publishes, simulator subscribes



def _mqtt_client():
    # paho-mqtt >= 2.0 requires picking a callback API version explicitly.
    try:
        return mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    except AttributeError:
        return mqtt.Client()

def on_message(client, userdata, msg):
    data = json.loads(msg.payload.decode())

    fl = data["fl"]            # Front-left ToF distance readings
    fr = data["fr"]            # Front-right ToF distance readings
    sl = data["sl"]            # Side-left ToF distance readings 
    sr = data["sr"]            # Side-right ToF distance readings 
    yaw_rate = data["gyro"][2]  # rad/s about z
    dt = data["dt"]            # s, simulator timestep

    print(f"fl={fl:.3f} fr={fr:.3f} sl={sl:.3f} sr={sr:.3f} "
          f"yaw_rate={yaw_rate:+.3f} dt={dt:.4f}")

    # TODO: compute wheel velocities (rad/s) from the readings above.
    left_vel = 0.0
    right_vel = 0.0

        # ======================= OUR CODE STARTS HERE =======================
    import math
 
    # ---------- TUNE THESE ----------
    FOLLOW = "L"        # which wall to prefer at a turn: "L" or "R"
    BASE = 3.0          # forward wheel speed (rad/s)
    KP = 0.0          # PID gains on side-distance error
    KI = 0.0
    KD = 0.0            # D uses the gyro (yaw rate) -> no noisy spikes
    SIDE_WALL = 0.15    # side reading below this = wall is there
    SIDE_TARGET = 0.10  # distance to keep from a single wall
    FRONT_STOP = 0.09   # front reading below this = stop and turn
    TURN_SPEED = 2.0    # wheel speed while spinning in place (rad/s)
    YAW_SIGN = 1.0      # -1 if turns never finish / overshoot forever
    MAX_STEP = 0.05     # max wheel-speed change per message (anti-tip ramp)
    # --------------------------------
 
    if dt <= 0:
        dt = 0.002
 
    # Sensor remap (from your straight-line test):
    # sl/sr point FORWARD, fl/fr point SIDEWAYS. Clamp junk values to 0.30.
    front = min(sl, sr, 0.30)
    left = min(fl, 0.30)
    right = min(fr, 0.30)
    if not math.isfinite(front): front = 0.30
    if not math.isfinite(left): left = 0.30
    if not math.isfinite(right): right = 0.30
 
    # Memory between messages (kept on the function itself)
    s = getattr(on_message, "s", None)
    if s is None:
        s = on_message.s = {"mode": "DRIVE", "integ": 0.0, "turned": 0.0,
                            "goal": 0.0, "L": 0.0, "R": 0.0, "n": 0}
    s["n"] += 1
 
    # ---------- DRIVE: PID keeps the bot centred ----------
    if s["mode"] == "DRIVE":
        if front < FRONT_STOP:
            # Wall ahead -> choose turn: preferred side, other side, else U-turn
            open_l, open_r = left > SIDE_WALL, right > SIDE_WALL
            if FOLLOW == "L":
                goal = 90 if open_l else (-90 if open_r else 180)
            else:
                goal = -90 if open_r else (90 if open_l else 180)
            s.update(mode="TURN", turned=0.0, goal=math.radians(goal), integ=0.0)
        else:
            wall_l, wall_r = left < SIDE_WALL, right < SIDE_WALL
            if wall_l and wall_r:
                err = (left - right) / 2        # centre between both walls
            elif wall_l:
                err = left - SIDE_TARGET        # hold distance from left wall
            elif wall_r:
                err = SIDE_TARGET - right       # hold distance from right wall
            else:
                err = 0.0                       # no walls: just go straight
 
            s["integ"] = max(-0.05, min(0.05, s["integ"] + err * dt))
            steer = KP * err + KI * s["integ"] - KD * YAW_SIGN * yaw_rate
            steer = max(-BASE, min(BASE, steer))   # +steer = turn left
            left_vel = BASE - steer
            right_vel = BASE + steer
 
    # ---------- TURN: spin in place until the gyro says we've turned enough ----------
    if s["mode"] == "TURN":
        s["turned"] += YAW_SIGN * yaw_rate * dt
        remaining = s["goal"] - s["turned"]
        if abs(remaining) < math.radians(3):
            s["mode"] = "DRIVE"
            left_vel = right_vel = 0.0
        else:
            w = TURN_SPEED if remaining > 0 else -TURN_SPEED
            if abs(remaining) < math.radians(20):
                w *= 0.4                         # slow down near the end
            left_vel, right_vel = -w, w          # +w = spin left
 
    # ---------- smooth speed changes so the bot doesn't tip ----------
    left_vel = s["L"] + max(-MAX_STEP, min(MAX_STEP, left_vel - s["L"]))
    right_vel = s["R"] + max(-MAX_STEP, min(MAX_STEP, right_vel - s["R"]))
    s["L"], s["R"] = left_vel, right_vel
 
    if s["n"] % 100 == 0:
        print(f"  >> [{s['mode']}] front={front:.3f} left={left:.3f} right={right:.3f} "
              f"turned={math.degrees(s['turned']):+.0f}/{math.degrees(s['goal']):+.0f} "
              f"L={left_vel:+.2f} R={right_vel:+.2f}")


 
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
