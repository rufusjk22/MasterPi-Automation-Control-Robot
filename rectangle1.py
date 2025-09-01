#!/usr/bin/env python3
import time

# ----------------------------------------------------------------
# CALIBRATION CONSTANTS - YOU MUST ADJUST THESE
# ----------------------------------------------------------------
# Example: if speed=50 => 20 cm/s => 1 cm takes 0.05 s
DISTANCE_TO_TIME = 0.05   # seconds per cm at speed=50
# Example: if speed=50 => 120 deg/s => 1 deg takes 0.0083 s
ANGLE_TO_TIME    = 0.0083 # seconds per degree at speed=50

DEFAULT_SPEED_PCT = 50    # default motor speed in [0..100]

def move_forward(distance_cm, speed_pct=DEFAULT_SPEED_PCT):
    """
    Drive forward 'distance_cm' centimeters using time-based control.
    'speed_pct' is the motor speed setting [0..100].
    """
    # Time needed = distance * (sec/cm), then adjust if user speed != 50
    travel_time = distance_cm * DISTANCE_TO_TIME * (50.0 / speed_pct)

    print(f"[INFO] move_forward: {distance_cm} cm at speed={speed_pct} => ~{travel_time:.2f}s")

    # --- REPLACE with your actual robot commands ---
    # e.g., robot.forward(speed_pct)
    print(f"robot.forward({speed_pct})")
    time.sleep(travel_time)
    # robot.stop()
    print("robot.stop()")

def turn_angle(angle_deg, speed_pct=DEFAULT_SPEED_PCT, direction='left'):
    """
    Turn the robot in place by 'angle_deg' degrees at 'speed_pct'.
    'direction' = 'left' or 'right'.
    Uses time-based spinning without sensor feedback.
    """
    turn_time = angle_deg * ANGLE_TO_TIME * (50.0 / speed_pct)

    print(f"[INFO] turn_angle: {angle_deg}° {direction} at speed={speed_pct} => ~{turn_time:.2f}s")

    # --- REPLACE with your actual robot spin commands ---
    if direction == 'left':
        # robot.spin_left(speed_pct)
        print(f"robot.spin_left({speed_pct})")
    else:
        # robot.spin_right(speed_pct)
        print(f"robot.spin_right({speed_pct})")

    time.sleep(turn_time)
    # robot.stop()
    print("robot.stop()")

def draw_rectangle(length_cm, width_cm, speed_pct=DEFAULT_SPEED_PCT):
    """
    Draws a rectangle with sides: length_cm, width_cm.
    Each corner is a 90° left turn (in time-based mode).
    """
    print(f"[INFO] Drawing rectangle: length={length_cm}cm, width={width_cm}cm, speed={speed_pct}")

    # Move length, turn 90°, move width, turn 90°,
    # move length, turn 90°, move width, turn 90° => back to start
    move_forward(length_cm, speed_pct)
    turn_angle(90, speed_pct, 'left')

    move_forward(width_cm, speed_pct)
    turn_angle(90, speed_pct, 'left')

    move_forward(length_cm, speed_pct)
    turn_angle(90, speed_pct, 'left')

    move_forward(width_cm, speed_pct)
    turn_angle(90, speed_pct, 'left')

    print("[INFO] Done. Robot should be back at start (within some margin).")

def main():
    print("=== Rectangle Drawing Demo ===")

    # Ask user for rectangle dimensions
    length_str = input("Enter rectangle length in cm (e.g., 50): ")
    width_str  = input("Enter rectangle width in cm  (e.g., 30): ")

    try:
        length_cm = float(length_str)
        width_cm  = float(width_str)
    except ValueError:
        length_cm = 50.0
        width_cm  = 30.0
        print("Invalid input. Using defaults (length=50, width=30).")

    # Optionally ask user for speed (or just use default)
    # speed_str = input("Enter speed in 0..100 (e.g. 50): ")
    # ...
    speed_pct = DEFAULT_SPEED_PCT

    # Execute the shape
    draw_rectangle(length_cm, width_cm, speed_pct)

if __name__ == "__main__":
    main()
