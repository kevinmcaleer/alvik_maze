"""
Alvik Main Firmware - ROS2 Integration
Connects to MQTT broker and provides scanning and navigation capabilities
Uses asyncio for non-blocking MQTT communication
"""
from arduino_alvik import ArduinoAlvik
from my_secrets import WIFI_SSID, WIFI_PASSWORD, MQTT_BROKER, MQTT_PORT
import network
import time
import uasyncio as asyncio
from umqtt.robust import MQTTClient
import json
import math

# Initialize Alvik
alvik = ArduinoAlvik()
alvik.begin()

# Robot state
robot_x = 0.0
robot_y = 0.0
robot_theta = 0.0

# Global MQTT client
mqtt_client = None
scanning = False

def connect_wifi():
    """Connect to WiFi network"""
    print("Connecting to WiFi...")
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)

    if not wlan.isconnected():
        wlan.connect(WIFI_SSID, WIFI_PASSWORD)

        timeout = 50
        while not wlan.isconnected() and timeout > 0:
            print(".", end="")
            time.sleep(1)
            timeout -= 1

        if wlan.isconnected():
            print("\nWiFi connected!")
            print("IP: {}".format(wlan.ifconfig()[0]))
            return True
        else:
            print("\nWiFi connection failed!")
            return False

    return True

def connect_mqtt():
    """Connect to MQTT broker"""
    print("Connecting to MQTT broker at {}...".format(MQTT_BROKER))

    client = MQTTClient(
        client_id="alvik_robot",
        server=MQTT_BROKER,
        port=MQTT_PORT,
        keepalive=120  # Increased to 120s for better tolerance during blocking operations
    )

    try:
        client.connect()
        print("MQTT connected!")

        # Subscribe to command topic
        client.set_callback(on_mqtt_message)
        client.subscribe(b"alvik/command")

        # Send status
        client.publish(b"alvik/status", b"Alvik connected and ready")

        return client
    except Exception as e:
        print("MQTT connection failed: {}".format(e))
        return None

def on_mqtt_message(topic, msg):
    """Handle incoming MQTT commands"""
    global scanning
    try:
        command = json.loads(msg.decode())
        cmd_type = command.get('type')

        print("Received command: {}".format(cmd_type))

        if cmd_type == 'scan':
            # Schedule async scan task
            if not scanning:
                asyncio.create_task(perform_scan())
            else:
                print("Scan already in progress")

        elif cmd_type == 'move':
            # Move forward/backward
            distance = command.get('distance', 0)
            move_robot(distance)

        elif cmd_type == 'rotate':
            # Rotate in place
            angle = command.get('angle', 0)
            rotate_robot(angle)

        elif cmd_type == 'stop':
            # Emergency stop
            alvik.stop()
            publish_status("Stopped")

    except Exception as e:
        print("Error handling command: {}".format(e))

async def mqtt_keepalive():
    """Background task to keep MQTT connection alive"""
    global mqtt_client
    while True:
        try:
            if mqtt_client:
                mqtt_client.check_msg()
            await asyncio.sleep(0.1)  # Check every 100ms
        except Exception as e:
            print("MQTT keepalive error: {}".format(e))
            await asyncio.sleep(1)

async def perform_scan():
    """Perform 360° scan and publish readings using async approach"""
    global mqtt_client, scanning

    scanning = True
    print("Starting 360° scan...")
    publish_status("Scanning...")

    angle_increment = 5
    num_readings = 360 // angle_increment
    readings_sent = 0

    try:
        for i in range(num_readings):
            angle = i * angle_increment
            iter_start = time.time()

            print("[{}] Starting reading at {}°".format(i, angle))

            # Get ToF distance - may return single value or tuple
            tof_start = time.time()
            dist = alvik.get_distance()
            tof_time = time.time() - tof_start
            print("[{}] ToF read: {:.3f}s, value: {}".format(i, tof_time, dist))

            # Handle both single value and tuple returns
            distances = []
            if isinstance(dist, tuple):
                # Multiple readings - add all valid ones
                for d in dist:
                    if d > 0:
                        distances.append(d)
            else:
                # Single reading
                if dist > 0:
                    distances.append(dist)

            # Calculate average distance
            if distances:
                avg_distance = sum(distances) / len(distances)
            else:
                avg_distance = 9999  # No valid reading

            # Publish reading
            pub_start = time.time()
            try:
                message = "{},{}".format(angle, avg_distance)
                mqtt_client.publish(b"alvik/scan", message.encode())
                readings_sent += 1
                pub_time = time.time() - pub_start
                print("[{}] Published: {:.3f}s".format(i, pub_time))

            except Exception as e:
                print("[{}] Publish error: {}".format(i, e))

            # Yield control to event loop
            await asyncio.sleep(0.05)

            # Rotate to next angle
            if i < num_readings - 1:
                rot_start = time.time()
                print("[{}] Starting rotation...".format(i))
                try:
                    alvik.rotate(angle_increment)
                    rot_time = time.time() - rot_start
                    print("[{}] Rotation done: {:.3f}s".format(i, rot_time))
                except Exception as e:
                    print("[{}] Rotation error: {}".format(i, e))
                    break

                # Yield after rotation
                await asyncio.sleep(0.05)

            iter_time = time.time() - iter_start
            print("[{}] Total iteration: {:.3f}s\n".format(i, iter_time))

    except Exception as e:
        print("Scan error: {}".format(e))

    finally:
        # Stop any motion
        alvik.stop()

        # Publish scan complete signal
        try:
            mqtt_client.publish(b"alvik/scan_complete", b"1")
            print("Scan complete! Sent {}/{} readings".format(readings_sent, num_readings))
        except Exception as e:
            print("Error publishing scan complete: {}".format(e))

        publish_status("Scan complete")
        scanning = False

def move_robot(distance_cm):
    """Move robot forward/backward by specified distance in cm"""
    global robot_x, robot_y, robot_theta

    print("Moving {}cm...".format(distance_cm))

    # Move the robot
    alvik.move(distance_cm)

    # Update odometry (simple dead reckoning)
    distance_m = distance_cm / 100.0
    robot_x += distance_m * math.cos(robot_theta)
    robot_y += distance_m * math.sin(robot_theta)

    publish_odometry()
    publish_status("Moved {}cm".format(distance_cm))

def rotate_robot(angle_deg):
    """Rotate robot by specified angle in degrees"""
    global robot_theta

    print("Rotating {}°...".format(angle_deg))

    # Rotate the robot
    alvik.rotate(angle_deg)

    # Update odometry
    robot_theta += math.radians(angle_deg)

    # Normalize angle to [-pi, pi]
    while robot_theta > math.pi:
        robot_theta -= 2 * math.pi
    while robot_theta < -math.pi:
        robot_theta += 2 * math.pi

    publish_odometry()
    publish_status("Rotated {}°".format(angle_deg))

def publish_odometry():
    """Publish current robot position"""
    global robot_x, robot_y, robot_theta, mqtt_client

    odom_data = {
        "x": round(robot_x, 3),
        "y": round(robot_y, 3),
        "theta": round(robot_theta, 3)
    }

    mqtt_client.publish(b"alvik/odom", json.dumps(odom_data).encode())

def publish_status(status):
    """Publish status message"""
    global mqtt_client
    mqtt_client.publish(b"alvik/status", status.encode())

async def main_loop():
    """Main program loop using asyncio"""
    global mqtt_client

    print("Alvik ROS2 Integration Starting...")

    # Connect to WiFi
    if not connect_wifi():
        print("Failed to connect to WiFi. Please check credentials.")
        return

    # Connect to MQTT
    mqtt_client = connect_mqtt()
    if not mqtt_client:
        print("Failed to connect to MQTT. Please check broker address.")
        return

    print("\nAlvik ready! Waiting for commands...")
    print("Robot at position: ({}, {}), angle: {}°".format(robot_x, robot_y, math.degrees(robot_theta)))

    # Publish initial odometry
    publish_odometry()

    # Start MQTT keepalive task in background
    asyncio.create_task(mqtt_keepalive())

    # Main loop - just keep the event loop alive
    try:
        while True:
            await asyncio.sleep(1)

    except KeyboardInterrupt:
        print("\nShutting down...")
        alvik.stop()
        mqtt_client.disconnect()

# Start the program
if __name__ == "__main__":
    try:
        asyncio.run(main_loop())
    except KeyboardInterrupt:
        print("\nProgram terminated")
