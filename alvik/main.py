"""
Alvik Main Firmware - ROS2 Integration
Connects to MQTT broker and provides scanning and navigation capabilities
"""
from arduino_alvik import ArduinoAlvik
import network
import time
from umqtt.simple import MQTTClient
import json
import math

# Configuration - UPDATE THESE!
WIFI_SSID = "YOUR_WIFI_SSID"
WIFI_PASSWORD = "YOUR_WIFI_PASSWORD"
MQTT_BROKER = "192.168.1.XXX"  # Your Mac's IP address
MQTT_PORT = 1883

# Initialize Alvik
alvik = ArduinoAlvik()
alvik.begin()

# Robot state
robot_x = 0.0
robot_y = 0.0
robot_theta = 0.0

def connect_wifi():
    """Connect to WiFi network"""
    print("Connecting to WiFi...")
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    
    if not wlan.isconnected():
        wlan.connect(WIFI_SSID, WIFI_PASSWORD)
        
        timeout = 20
        while not wlan.isconnected() and timeout > 0:
            print(".", end="")
            time.sleep(1)
            timeout -= 1
        
        if wlan.isconnected():
            print("\nWiFi connected!")
            print(f"IP: {wlan.ifconfig()[0]}")
            return True
        else:
            print("\nWiFi connection failed!")
            return False
    
    return True

def connect_mqtt():
    """Connect to MQTT broker"""
    print(f"Connecting to MQTT broker at {MQTT_BROKER}...")
    
    client = MQTTClient(
        client_id="alvik_robot",
        server=MQTT_BROKER,
        port=MQTT_PORT,
        keepalive=60
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
        print(f"MQTT connection failed: {e}")
        return None

def on_mqtt_message(topic, msg):
    """Handle incoming MQTT commands"""
    try:
        command = json.loads(msg.decode())
        cmd_type = command.get('type')
        
        print(f"Received command: {cmd_type}")
        
        if cmd_type == 'scan':
            # Perform 360° scan
            perform_scan()
            
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
        print(f"Error handling command: {e}")

def perform_scan():
    """Perform 360° scan and publish readings"""
    global mqtt_client
    
    print("Starting 360° scan...")
    publish_status("Scanning...")
    
    # Take readings every 5 degrees (72 total readings)
    angle_increment = 5
    num_readings = 360 // angle_increment
    
    for i in range(num_readings):
        angle = i * angle_increment
        
        # Get ToF distance in mm
        distance = alvik.get_distance()
        
        # Publish reading
        message = f"{angle},{distance}"
        mqtt_client.publish(b"alvik/scan", message.encode())
        
        # Small delay for stability
        time.sleep(0.05)
        
        # Rotate to next angle
        if i < num_readings - 1:  # Don't rotate after last reading
            alvik.rotate(angle_increment)
            time.sleep(0.1)  # Let robot stabilize
    
    # Publish scan complete signal
    mqtt_client.publish(b"alvik/scan_complete", b"1")
    
    publish_status("Scan complete")
    print("Scan complete!")

def move_robot(distance_cm):
    """Move robot forward/backward by specified distance in cm"""
    global robot_x, robot_y, robot_theta
    
    print(f"Moving {distance_cm}cm...")
    
    # Move the robot
    alvik.move(distance_cm)
    
    # Update odometry (simple dead reckoning)
    distance_m = distance_cm / 100.0
    robot_x += distance_m * math.cos(robot_theta)
    robot_y += distance_m * math.sin(robot_theta)
    
    publish_odometry()
    publish_status(f"Moved {distance_cm}cm")

def rotate_robot(angle_deg):
    """Rotate robot by specified angle in degrees"""
    global robot_theta
    
    print(f"Rotating {angle_deg}°...")
    
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
    publish_status(f"Rotated {angle_deg}°")

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

def main_loop():
    """Main program loop"""
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
    print(f"Robot at position: ({robot_x}, {robot_y}), angle: {math.degrees(robot_theta)}°")
    
    # Publish initial odometry
    publish_odometry()
    
    # Main loop - check for MQTT messages
    while True:
        try:
            mqtt_client.check_msg()  # Non-blocking check for messages
            time.sleep(0.1)
            
        except KeyboardInterrupt:
            print("\nShutting down...")
            alvik.stop()
            mqtt_client.disconnect()
            break
            
        except Exception as e:
            print(f"Error in main loop: {e}")
            time.sleep(1)

# Start the program
if __name__ == "__main__":
    main_loop()
