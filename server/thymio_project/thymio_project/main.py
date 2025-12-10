import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import numpy as np
import threading
import time
import math
import io
from flask import Flask, jsonify, send_file
from PIL import Image
from thymiodirect import Thymio

#lab06 constants
SPEED_COEFF = 0.0003027 #kd
ROBOT_WIDTH = 0.0935

ROBOT_SPEED = 100       # default velocity
ROTATION_SPEED = 50     # velocity of spin 

# sensors thresholds
IR_THRESHOLD = 1800      # IR: if any front sensor > this, emergency stop
LIDAR_WARN_DIST = 0.25   # LIDAR: starts a warning if obstacle is closer than this (in meters)

app = Flask(__name__)
robot_node = None

def euler_to_quaternion(roll, pitch, yaw):
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    return [qx, qy, qz, qw]

class ThymioBrain(Node):
    def __init__(self):
        super().__init__('thymio_driver')
        self.connect_thymio()
        
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.create_subscription(OccupancyGrid, '/map', self.map_cb, 10)

        #state variables
        #pose
        self.x = 0.0
        self.y = 0.0
        self.th_yaw = 0.0
        self.last_time = self.get_clock().now()
        self.latest_scan = None
        self.latest_map = None
        self.last_png = None
        self.last_png_time = 0
        self.is_exploring = False 
        
        self.create_timer(0.1, self.update_loop) # 10 Hz

    def connect_thymio(self):
        try:
            # check the correct port
            port = "/dev/ttyACM0" 
            self.th = Thymio(serial_port=port, on_connect=lambda node_id: print(f'Thymio found {node_id}'))
            self.th.connect()
            self.id = self.th.first_node()
            self.th[self.id]["leds.top"] = [0, 32, 0] 
            print("[LOG] THYMIO CONNECTED.")
            return True
        except Exception as e:
            print(f"[LOG] CONNECTION FAIL: {e}")
            self.th = None
            return False

    def scan_cb(self, msg):
        self.latest_scan = msg

    def map_cb(self, msg):
        self.latest_map = msg

    def update_loop(self):
        if self.th is None:
            if (self.get_clock().now().nanoseconds % 2000000000) < 100000000:
                print("[LOG] LOOKING FOR DONGLE")
                self.connect_thymio()
            return

        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        try:
            # ODOMETRY CALCULATION
            rl = self.th[self.id]["motor.left.speed"]
            rr = self.th[self.id]["motor.right.speed"]
            # read all 7 proximity sensors
            prox = self.th[self.id]["prox.horizontal"]

            if rl > 32767: rl -= 65536
            if rr > 32767: rr -= 65536

            #based on lab06
            vl = rl * SPEED_COEFF
            vr = rr * SPEED_COEFF
            v_lin = (vr + vl) / 2.0 # linear displacement of the robot
            v_ang = (vr - vl) / ROBOT_WIDTH # angular displacement of the robot (∆θ)

            delta_th = v_ang * dt
            #2nd order Runga-Kutta
            mid_theta = self.th_yaw + (delta_th / 2.0)
            self.x += v_lin * math.cos(mid_theta) * dt
            self.y += v_lin * math.sin(mid_theta) * dt
            self.th_yaw += delta_th

            self.publish_tf(current_time, self.x, self.y, self.th_yaw)
            
            # Hybrid Navigation Logic
            tgt_l = 0
            tgt_r = 0

            if self.is_exploring:

                # Verify IR sensors: if any front sensor > threshold, emergency stop
                front_prox = prox[0:5]
                max_ir = max(front_prox)

                if max_ir > IR_THRESHOLD:
                    #Emergency stop - obstacle very close
                    self.th[self.id]["leds.top"] = [32, 0, 0]
                    print(f"[NAV] EMERGENCY IR: {max_ir}")
                    
                    # calculate the weight on each side
                    #with this we can decide which way to turn
                    weight_left = prox[0]*2 + prox[1]
                    weight_right = prox[4]*2 + prox[3]
                    
                    if weight_left > weight_right:
                        # obstacle on the left -> turn right
                        tgt_l = ROTATION_SPEED
                        tgt_r = -ROTATION_SPEED
                    else:
                        # obstacle on the right -> turn left
                        tgt_l = -ROTATION_SPEED
                        tgt_r = ROTATION_SPEED

                # Verify LIDAR data
                elif self.latest_scan:
                    # antecipation with LIDAR
                    ranges = np.array(self.latest_scan.ranges)
                    ranges[ranges == 0] = 10.0 # avoid outliers
                    
                    # analyze front of the robot (counterclockwise)
                    left_cone = np.min(ranges[0:30])   # 0º a 30º 
                    right_cone = np.min(ranges[-30:])  # 330º a 360º
                    
                    min_dist = min(left_cone, right_cone)

                    if min_dist < LIDAR_WARN_DIST:
                        self.th[self.id]["leds.top"] = [32, 16, 0]
                        print(f"[NAV] SMALL TURN LIDAR: {min_dist:.2f}m")
                        
                        # small turn to avoid obstacle 
                        #note: this logic are inverted because of the LIDAR angle convention
                        if left_cone < right_cone:
                            # obstacle on the left -> turn right
                            tgt_l = -10
                            tgt_r = ROBOT_SPEED
                            #tgt_l = ROBOT_SPEED
                            #tgt_r = -10
                        else:
                            # Oobstacle on the right -> turn left
                            tgt_l = ROBOT_SPEED
                            tgt_r = -10
                            #tgt_l = -10
                            #tgt_r = ROBOT_SPEED
                            
                    else:
                        # cruise mode
                        self.th[self.id]["leds.top"] = [0, 0, 32]
                        tgt_l = ROBOT_SPEED
                        tgt_r = ROBOT_SPEED
                
                else:
                    # without LIDAR data, just using IR
                    tgt_l = ROBOT_SPEED
                    tgt_r = ROBOT_SPEED

            else:
                # stopped
                self.th[self.id]["leds.top"] = [0, 32, 0]
                tgt_l = 0; tgt_r = 0

            self.th[self.id]["motor.left.target"] = int(tgt_l)
            self.th[self.id]["motor.right.target"] = int(tgt_r)

        except Exception as e:
            print(f"[LOG] ERROR LOOP: {e}")
            self.th = None

    def publish_tf(self, current_time, x, y, yaw):
        q = euler_to_quaternion(0, 0, yaw)
        t = TransformStamped()
        t.header.stamp = (current_time + Duration(seconds=0.1)).to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.tf_broadcaster.sendTransform(t)
        
        # publish /odom
        odom = Odometry()
        odom.header = t.header
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.orientation = t.transform.rotation
        self.odom_pub.publish(odom)

# Flask API Endpoints
@app.route('/start', methods=['GET'])
def start_exp():
    if robot_node:
        robot_node.is_exploring = True
        return jsonify({"status": "Started"})
    return jsonify({"error": "Error"}), 500

@app.route('/stop', methods=['GET'])
def stop_exp():
    if robot_node:
        robot_node.is_exploring = False
        return jsonify({"status": "Stopped"})
    return jsonify({"error": "Error"}), 500

@app.route('/map-image', methods=['GET'])
def get_map_image():
    # check if map is available
    if not robot_node or not robot_node.latest_map:
        return jsonify({"error": "Map not ready yet"}), 404

    now = time.time()
    if robot_node.last_png and (now - robot_node.last_png_time) < 1.0:
        return send_file(robot_node.last_png, mimetype='image/png')  
    try:
        msg = robot_node.latest_map
        width = msg.info.width
        height = msg.info.height
        
        # convert OccupancyGrid data to image
        data = np.array(msg.data, dtype=np.int8).reshape((height, width))
        
        #create rgb image
        img_data = np.full((height, width), 127, dtype=np.uint8)
        
        # Mapping values: https://docs.ros.org/en/api/nav_msgs/html/msg/OccupancyGrid.html
        img_data[data == 0] = 255
        img_data[data == 100] = 0
        
        # ROS uses origin at bottom-left, images use top-left
        img_data = np.flipud(img_data)
        
        #convert numpy array to PNG image
        #grayscale, 8-bit pixels (L)
        img = Image.fromarray(img_data, mode='L')
        img_io = io.BytesIO()
        img.save(img_io, 'PNG')
        img_io.seek(0)

        return send_file(img_io, mimetype='image/png')

    except Exception as e:
        print(f"Erro ao gerar mapa: {e}")
        return jsonify({"error": str(e)}), 500    

def main(args=None):
    global robot_node
    rclpy.init(args=args)
    robot_node = ThymioBrain()
    t = threading.Thread(target=rclpy.spin, args=(robot_node,), daemon=True)
    t.start()
    app.run(host='0.0.0.0', port=5000)
    robot_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
