#!/usr/bin/env python3
import time

# -----------------------------------
# CALIBRATION CONSTANTS - ADJUST!
# -----------------------------------
# Example: If speed=50 means ~20 cm/s, then traveling 1 cm takes 0.05 s
DISTANCE_TO_TIME = 0.05  # seconds per cm at speed=50 (example)

# Example: If speed=50 spins ~120 degrees per second => ~0.0083 s/degree
ANGLE_TO_TIME    = 0.0083  # seconds per degree at speed=50 (example)

# Desired speed in [0..100]; you can also ask the user for this
DEFAULT_SPEED_PCT = 50  

def move_forward(distance_cm, speed_pct=DEFAULT_SPEED_PCT):
    """
    Move the robot straight forward for 'distance_cm' centimeters,
    using time-based control at 'speed_pct' speed.
    """
    # Convert distance to time
    # (Here we assume the same scale for all speeds, or you can scale further)
    travel_time = distance_cm * DISTANCE_TO_TIME * (50.0 / speed_pct)
    # The (50.0 / speed_pct) factor is a rough attempt to account
    # for user-chosen speed if the above calibration was done at speed=50.

    print(f"[INFO] move_forward: distance={distance_cm}cm, speed={speed_pct}, time={travel_time:.2f}s")

    # --- REPLACE WITH YOUR ACTUAL ROBOT COMMANDS ---
    # Example:
    # robot.forward(speed_pct)
    print(f"robot.forward({speed_pct})")
    time.sleep(travel_time)
    # robot.stop()
    print("robot.stop()")


def turn_angle(angle_deg, speed_pct=DEFAULT_SPEED_PCT, direction='left'):
    """
    Turn the robot in place by 'angle_deg' degrees at 'speed_pct' speed.
    direction='left' or 'right' controls turning direction.
    Time-based approach, no sensor feedback.
    """
    # Convert angle to time
    turn_time = angle_deg * ANGLE_TO_TIME * (50.0 / speed_pct)

    print(f"[INFO] turn_angle: angle={angle_deg}°, speed={speed_pct}, time={turn_time:.2f}s, direction={direction}")

    # --- REPLACE WITH YOUR ACTUAL ROBOT COMMANDS ---
    if direction == 'left':
        # robot.spin_left(speed_pct)
        print(f"robot.spin_left({speed_pct})")
    else:
        # robot.spin_right(speed_pct)
        print(f"robot.spin_right({speed_pct})")

    time.sleep(turn_time)
    # robot.stop()
    print("robot.stop()")


def draw_equilateral_triangle(side_length_cm, speed_pct=DEFAULT_SPEED_PCT):
    """
    Drive the robot in an equilateral triangle with side_length_cm (in cm).
    Uses time-based forward and 120° turns.
    """
    print(f"[INFO] Drawing equilateral triangle, side={side_length_cm}cm, speed={speed_pct}")

    for i in range(3):
        move_forward(side_length_cm, speed_pct)
        # Turn 120°, typically 'left' is standard
        turn_angle(120, speed_pct, direction='left')

    print("[INFO] Done. The robot should be back at the start (within some margin).")


def main():
    print("=== Equilateral Triangle Demo ===")

    # Ask the user for side length
    side_str = input("Enter side length in cm (e.g. 50): ")
    try:
        side_cm = float(side_str)
    except ValueError:
        side_cm = 50.0
        print("Invalid input. Using default side=50 cm.")

    # Optionally ask user for speed:
    # speed_str = input("Enter speed in 0..100 (e.g. 50): ")
    # try:
    #     spd = float(speed_str)
    # except ValueError:
    #     spd = 50.0
    #     print("Invalid speed. Using default=50.")

    # For simplicity, we'll just use DEFAULT_SPEED_PCT
    spd = DEFAULT_SPEED_PCT

    draw_equilateral_triangle(side_cm, spd)


if __name__ == "__main__":
    main()
