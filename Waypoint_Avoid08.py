# Single-obstacle avoidance between takeoff and first waypoint
import logging
import time
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.position_hl_commander import PositionHlCommander
from cflib.utils import uri_helper
from cflib.utils.multiranger import Multiranger

# Connection setup
URI = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E707')
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

def is_close(distance, threshold=0.25):
    return distance is not None and distance < threshold

def get_pos(commander):
    """Safely read current estimated (x,y,z) from commander."""
    try:
        x = float(getattr(commander, "_x", 0.0))
        y = float(getattr(commander, "_y", 0.0))
        z = float(getattr(commander, "_z", z0))
        return x, y, z
    except:
        return 0.0, 0.0, z0

def move_with_avoidance(commander, multiranger, tx, ty, tz, duration, obstacle_handled):
    """
    Executes waypoint movement.
    If obstacle not yet handled and front sensor detects it,
    perform bypass then return immediately.
    """
    start = time.time()

    while time.time() - start < duration:

        # Only handle obstacle once per mission
        if not obstacle_handled and is_close(multiranger.front):
            print("Obstacle detected in front. Executing bypass.")

            cx, cy, cz = get_pos(commander)

            sidestep = 0.5
            forward = 0.5

            # 1. Sidestep left
            commander.go_to(cx, cy - sidestep, cz)
            time.sleep(1.0)

            # 2. Move forward past obstacle
            commander.go_to(cx + forward, cy - sidestep, cz)
            time.sleep(1.0)

            # 3. Return right to original line
            commander.go_to(cx + forward, cy, cz)
            time.sleep(1.0)

            print("Bypass complete. Resuming waypoint sequence.")
            return True   # obstacle_handled = True

        # Normal movement (or ignore further obstacles after handled)
        commander.go_to(tx, ty, tz)
        time.sleep(0.1)

    return obstacle_handled


# Main program
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

                    start_time = time.time()
                    max_duration = 120
                    obstacle_handled = False

                    # Execute flight path
                    for i, (tx, ty, tz, t) in enumerate(newsequence):
                        print(f"Waypoint {i+1}/{len(newsequence)}: ({tx}, {ty}, {tz})")

                        # Mission timeout
                        if time.time() - start_time > max_duration:
                            print("Failsafe: Mission timeout. Landing.")
                            commander.land(0.0, 2.0)
                            raise SystemExit

                        # Radio loss
                        if not scf.cf.link:
                            print("Failsafe: Radio link lost. Landing.")
                            commander.land(0.0, 2.0)
                            raise SystemExit

                        # Low altitude fail-safe
                        if multiranger.up and multiranger.up < 0.15:
                            print("Failsafe: Altitude too low. Landing.")
                            commander.land(0.0, 2.0)
                            raise SystemExit

                        # Move with obstacle avoidance
                        obstacle_handled = move_with_avoidance(
                            commander, multiranger, tx, ty, tz, t, obstacle_handled
                        )

                        time.sleep(0.3)

                    print("Sequence complete. Hovering.")
                    time.sleep(5)

                    print("Landing.")
                    commander.land(0.0, 2.0)
                    time.sleep(2)

    except KeyboardInterrupt:
        print("Manual abort. Landing.")
        try:
            commander.land(0.0, 2.0)
        except:
            pass

    except Exception as e:
        print("Unexpected error:", e)
        try:
            commander.land(0.0, 2.0)
        except:
            pass
