# Adaptive single-obstacle avoidance (one obstacle per mission)
import logging
import time
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.position_hl_commander import PositionHlCommander
from cflib.utils import uri_helper
from cflib.utils.multiranger import Multiranger

# Connection setup — change URI to your Crazyflie
URI = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E702')
logging.basicConfig(level=logging.ERROR)

# Waypoints
z0 = 0.4
x0 = 1.0
x1 = 0.0
x2 = -1.0
y0 = 0.0
y1 = -0.4

newsequence = [
    (x1, y0, z0, 3.0),
    (x0, y0, z0, 3.0),
    (x0, y1, z0, 3.0),
    (x1, y1, z0, 3.0),
    (x2, y1, z0, 3.0),
    (x2, y0, z0, 3.0),
    (x1, y0, z0, 3.0),
]

# Helper utilities
def is_close(distance, threshold=0.25):
    return distance is not None and distance < threshold

def get_commander_position(commander, fallback_z=z0):
    try:
        cx = float(getattr(commander, "_x", 0.0))
        cy = float(getattr(commander, "_y", 0.0))
        cz = float(getattr(commander, "_z", fallback_z))
    except Exception:
        cx, cy, cz = 0.0, 0.0, fallback_z
    return cx, cy, cz


def move_with_avoidance(commander, multiranger,
                         target_x, target_y, target_z, duration,
                         obstacle_handled):
    """
    One-shot obstacle bypass:
      - left 0.5
      - forward 0.5
      - right 0.5
    """
    start_time = time.time()
    print(f">>> Move to ({target_x}, {target_y}, {target_z}) for {duration}s (handled={obstacle_handled})")

    while time.time() - start_time < duration:

        # Handle the one single front obstacle
        if not obstacle_handled and is_close(multiranger.front):
            print("Obstacle detected (front) — performing bypass")

            cx, cy, cz = get_commander_position(commander)

            sidestep = 0.5
            forward = 0.5

            # 1) left
            commander.go_to(cx, cy - sidestep, cz)
            time.sleep(1.0)

            # 2) forward
            commander.go_to(cx + forward, cy - sidestep, cz)
            time.sleep(1.0)

            # 3) right
            commander.go_to(cx + forward, cy, cz)
            time.sleep(1.0)

            print("Bypass complete")
            return True

        # Minor avoidance on right side (won’t count as obstacle handled)
        if is_close(getattr(multiranger, "right", None)) and not obstacle_handled:
            cx, cy, cz = get_commander_position(commander)
            sidestep_y = cy - 0.5
            print(f"Obstacle on right — sidestep left to ({cx:.2f}, {sidestep_y:.2f}, {cz:.2f})")
            commander.go_to(cx, sidestep_y, cz)
            time.sleep(1.0)
            return obstacle_handled

        # No obstacle
        commander.go_to(target_x, target_y, target_z)
        time.sleep(0.1)

    return obstacle_handled


# -----------------------------------
# MAIN PROGRAM (no failsafe)
# -----------------------------------

if __name__ == "__main__":
    try:
        cflib.crtp.init_drivers()
        cf = Crazyflie(rw_cache="./cache")

        with SyncCrazyflie(URI, cf=cf) as scf:
            scf.cf.platform.send_arming_request(True)
            time.sleep(1.0)

            with PositionHlCommander(scf, default_height=z0) as commander:
                with Multiranger(scf) as multiranger:
                    print("Takeoff...")
                    time.sleep(3)

                    obstacle_handled = False   # One obstacle per mission

                    for i, (tx, ty, tz, t) in enumerate(newsequence):
                        print(f"Waypoint {i+1}/{len(newsequence)}: ({tx}, {ty}, {tz})")

                        obstacle_handled = move_with_avoidance(
                            commander, multiranger, tx, ty, tz, t, obstacle_handled
                        )
                        time.sleep(0.5)

                    print("Sequence complete — hovering...")
                    time.sleep(5)

                    print("Landing...")
                    commander.land(0.0, 2.0)
                    time.sleep(3)
                    print("Mission completed successfully")

    except KeyboardInterrupt:
        print("Manual abort — landing.")
        try:
            commander.land(0.0, 2.0)
        except:
            pass
        time.sleep(2)

    except Exception as e:
        print("Unexpected error:", e)
        try:
            commander.land(0.0, 2.0)
        except:
            pass
        time.sleep(2)
        print("Drone disarmed safely.")
