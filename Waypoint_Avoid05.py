#Importing modules and libraries
import logging
import time
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.position_hl_commander import PositionHlCommander
from cflib.utils import uri_helper
from cflib.utils.multiranger import Multiranger


# Connection setup
URI = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E700')
logging.basicConfig(level=logging.ERROR)


# Helper: obstacle detection
def is_close(distance, threshold=0.25):
    """Returns True if an obstacle is detected within threshold (m)."""
    return distance is not None and distance < threshold

# Helper: safe movement with avoidance
def move_with_avoidance(commander, multiranger, target_x, target_y, target_z, duration):
    """Move to target waypoint with a fixed, predictable obstacle bypass."""
    start_time = time.time()
    print(f" Moving to ({target_x}, {target_y}, {target_z}) for {duration}s")

    # Normal movement loop
    while time.time() - start_time < duration:

      
        # AVOIDANCE
        if is_close(multiranger.front):
            print("Obstacle in front — executing hard-coded bypass")

            # 1. Sidestep LEFT 0.5 m
            print("Sidestep left 0.5 m")
            commander.go_to(target_x, target_y - 0.5, target_z)
            time.sleep(1.0)

            # 2. Move FORWARD 0.5 m
            print("Move forward 0.5 m")
            commander.go_to(target_x + 0.5, target_y - 0.5, target_z)
            time.sleep(1.0)

            # 3. Move RIGHT 0.5 m (return to original Y line)
            print("Return right 0.5 m to rejoin path")
            commander.go_to(target_x + 0.5, target_y, target_z)
            time.sleep(1.0)

            print("Bypass complete — resuming next waypoint\n")
            return  #  return immediately to continue sequence

        # NO OBSTACLE → NORMAL POINT MOVE
        commander.go_to(target_x, target_y, target_z)
        time.sleep(0.1)




# Main program with failsafe
if __name__ == '__main__':
    try:
        # Initialize CRTP drivers (radio)
        cflib.crtp.init_drivers()
        cf = Crazyflie(rw_cache='./cache')

        # Connect to Crazyflie safely
        with SyncCrazyflie(URI, cf=cf) as scf:
            scf.cf.platform.send_arming_request(True)
            time.sleep(1.0)

            with PositionHlCommander(scf, default_height=0.4) as commander:
                with Multiranger(scf) as multiranger:

                    print("Takeoff...")
                    time.sleep(3)

                    # Define square flight path
                    waypoints = [
                                    (1.0, 0.0, 0.4),   # Forward
                                    (1.0, 0.7, 0.4),   # Right
                                    (0.0, 0.7, 0.4),   # Back
                                    (-1.0, 0.7, 0.4),  # Back again
                                    (-1.0, 0.0, 0.4),  # Left
                                    (0.0, 0.0, 0.4)    # Return to start
                                ]

                  
                    # FAILSAFE VARIABLES
                    start_time = time.time()
                    max_flight_duration = 60   # seconds
                    min_safe_altitude = 0.15   # meters
                    x, y, z = 0.0, 0.0, 0.4

                    for i, (target_x, target_y, target_z) in enumerate(waypoints):
                        print(f"➡️ Moving to waypoint {i+1}: ({target_x}, {target_y}, {target_z})")

                        #Failsafe 1: timeout check
                        if time.time() - start_time > max_flight_duration:
                            print("Failsafe: Maximum flight time exceeded. Landing.")
                            commander.land(0.0, 2.0)
                            raise SystemExit

                        #Failsafe 2: link loss check 
                        if not scf.cf.link:
                            print(" Failsafe: Lost radio link! Attempting to land.")
                            commander.land(0.0, 2.0)
                            time.sleep(3)
                            raise SystemExit

                        #Failsafe 3: altitude safety
                        if multiranger.up and multiranger.up > 1.5:
                            print(" Altitude unexpectedly high — initiating emergency descent")
                            commander.land(0.0, 2.0)
                            raise SystemExit

                        # Execute movement with obstacle avoidance
                        x, y = move_with_avoidance(commander, multiranger, target_x, target_y, target_z)
                        time.sleep(1)

                    print("Square path complete. Hovering 5s...")
                    time.sleep(5)

                    print(" Landing...")
                    commander.land(0.0, 2.0)
                    time.sleep(3)
                    print("Mission completed safely!")


    # FAILSAFE 4: Manual abort (Ctrl + C)
    except KeyboardInterrupt:
        print("\n Manual abort detected (Ctrl+C) — landing immediately.")
        try:
            commander.land(0.0, 2.0)
        except Exception:
            pass
        time.sleep(3)


    # General exception catch-all
    except Exception as e:
        print(f" Unexpected error: {e}")
        try:
            commander.land(0.0, 2.0)
        except Exception:
            pass
        time.sleep(3)
        print(" Drone disarmed and safe.")
