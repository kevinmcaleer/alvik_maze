"""
Simple Maze Explorer for Arduino Alvik
Moves forward until detecting a wall, then turns left 90 degrees and continues
Kevin McAleer
16 November 2025
"""
from arduino_alvik import ArduinoAlvik
import time

# Initialize Alvik
alvik = ArduinoAlvik()
alvik.begin()

# Configuration
WALL_DISTANCE_CM = 10.0  # Stop when wall is closer than this (cm)
MOVE_STEP_CM = 5.0       # Move forward in small steps (cm)
TURN_ANGLE_DEG = 90      # Turn left 90 degrees when wall detected

print("Maze Explorer Starting!")
print(f"Wall detection threshold: {WALL_DISTANCE_CM}cm")
print(f"Movement step size: {MOVE_STEP_CM}cm")
print("")

try:
    while True:
        # Get distance from ToF sensor (returns tuple of 64 zones for 8x8 grid)
        distance_reading = alvik.get_distance()
        print(f"distance_reading is {distance_reading}")

        distance_cm = distance_reading[2]  # Center zone
        if distance_cm <= 0:
            distance_cm = 200  # Invalid reading, assume clear
        else:
            # Single value
            distance_cm = 200 if distance_reading[2] <= 0 else distance_reading[2]

        print(f"Distance ahead: {distance_cm:.1f}cm")

        # Check if wall is detected
        if distance_cm < WALL_DISTANCE_CM:
            print("WALL DETECTED! Turning left 90 degrees...")
            alvik.rotate(TURN_ANGLE_DEG)
            time.sleep(0.5)  # Brief pause after turn
            print("Turn complete. Checking new direction...")
        else:
            print(f"Path clear. Moving forward {MOVE_STEP_CM}cm...")
            alvik.drive(MOVE_STEP_CM, 0)
            time.sleep(0.3)  # Brief pause between movements

        print("")  # Blank line for readability

except KeyboardInterrupt:
    print("\nMaze exploration stopped by user")
    alvik.stop()

except Exception as e:
    print(f"Error: {e}")
    alvik.stop()

finally:
    print("Stopping robot...")
    alvik.stop()
