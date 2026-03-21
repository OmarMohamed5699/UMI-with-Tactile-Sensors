
import rclpy
from rclpy.node import Node
from sensor_interfaces.msg import SensorState
from sensor_interfaces.srv import BiasRequest, StartSlipDetection, StopSlipDetection
from std_msgs.msg import String
import numpy as np
import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import Dict, List, Optional
import json
import qrcode
import cv2
from datetime import datetime


@dataclass
class TactileReading:
    """One reading from a tactile sensor"""
    hub_id: int
    sensor_id: int
    timestamp: float
    global_force: np.ndarray
    global_torque: np.ndarray
    is_contact: bool
    friction_est: float
    target_grip_force: float
    is_sd_active: bool
    pillar_forces: np.ndarray
    pillar_displacements: np.ndarray
    pillar_contacts: np.ndarray
    pillar_slip_states: np.ndarray
    pillar_ids: np.ndarray


class TactileDataHandler(Node):
    def __init__(self):
        super().__init__('tactile_data_handler')
        
        # CONFIGURATION
        self.num_hubs = 1
        self.sensors_per_hub = 2
        self.pillars_per_sensor = 9
        self.buffer_size = 100
        
        # DATA STORAGE
        self.data_lock = threading.RLock()
        self.sensor_data = {}
        self.latest_readings = {}
        
        # PERFORMANCE TRACKING
        self.stats = {
            'messages_received': 0,
            'last_update_time': time.time(),
            'update_frequency': 0.0
        }
        
        # UMI INTEGRATION - Publishers
        self.tactile_pub = None
        self.tactile_60hz_pub = None
        self.last_60hz_publish_time = 0.0
        
       
        
        # SETUP
        self._initialize_data_storage()
        self._create_publishers()
        self._create_all_subscribers()
        self._create_service_clients()
        self._start_monitoring()
        
        self.get_logger().info(f"Started! Listening to {self.num_hubs} hub with {self.sensors_per_hub} sensors")
        self.get_logger().info("Publishing tactile data to /umi_gripper/tactile (full rate) and /umi_gripper/tactile_60hz (60 Hz)")
        self.get_logger().info("QR code synchronization enabled - call display_sync_qr() to create sync events")

    # =================================================================
    # All original initialization methods stay the same
    # ======================================================================
    
    
    
    def _initialize_data_storage(self):
        for hub_id in range(self.num_hubs):
            for sensor_id in range(self.sensors_per_hub):
                key = (hub_id, sensor_id)
                self.sensor_data[key] = deque(maxlen=self.buffer_size)
                self.latest_readings[key] = None




    def _create_publishers(self):
        self.tactile_pub = self.create_publisher(String, '/umi_gripper/tactile', 10)
        self.tactile_60hz_pub = self.create_publisher(String, '/umi_gripper/tactile_60hz', 10)
        
        self.get_logger().info("Created tactile publishers")





    def _create_all_subscribers(self):
        self.subscribers = []
        for hub_id in range(self.num_hubs):
            for sensor_id in range(self.sensors_per_hub):
                topic = f'/hub_{hub_id}/sensor_{sensor_id}'
                callback = self._make_callback_for_sensor(hub_id, sensor_id)
                qos = self._get_realtime_qos()
                sub = self.create_subscription(SensorState, topic, callback, qos)
                self.subscribers.append(sub)





    def _get_realtime_qos(self):
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
        return QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )




    def _start_monitoring(self):
        self.stats_timer = self.create_timer(1.0, self._update_statistics)




    def _create_service_clients(self):
        hub_id = 0
        bias_service_name = f'/hub_{hub_id}/send_bias_request'
        self.bias_client = self.create_client(BiasRequest, bias_service_name)
        start_sd_service_name = f'/hub_{hub_id}/start_slip_detection'
        self.start_sd_client = self.create_client(StartSlipDetection, start_sd_service_name)
        stop_sd_service_name = f'/hub_{hub_id}/stop_slip_detection'
        self.stop_sd_client = self.create_client(StopSlipDetection, stop_sd_service_name)






    def _make_callback_for_sensor(self, hub_id: int, sensor_id: int):
        def callback(msg: SensorState):
            try:
                reading = self._convert_message_to_reading(msg, hub_id, sensor_id)
                self._store_reading_safely(reading, hub_id, sensor_id)
                self._publish_tactile_data()
                self.stats['messages_received'] += 1
            except Exception as e:
                self.get_logger().error(f"Error with hub {hub_id}, sensor {sensor_id}: {e}")
        return callback





    def _convert_message_to_reading(self, msg: SensorState, hub_id: int, sensor_id: int) -> TactileReading:
        num_pillars = len(msg.pillars)
        pillar_forces = np.zeros((num_pillars, 3))
        pillar_displacements = np.zeros((num_pillars, 3))
        pillar_contacts = np.zeros(num_pillars, dtype=bool)
        pillar_slip_states = np.zeros(num_pillars)
        pillar_ids = np.zeros(num_pillars, dtype=int)
        
        for i, pillar in enumerate(msg.pillars):
            pillar_forces[i] = [pillar.fx, pillar.fy, pillar.fz]
            pillar_displacements[i] = [pillar.dx, pillar.dy, pillar.dz]
            pillar_contacts[i] = pillar.in_contact
            pillar_slip_states[i] = pillar.slip_state
            pillar_ids[i] = pillar.id
            
        return TactileReading(
            hub_id=hub_id,
            sensor_id=sensor_id,
            timestamp=time.time(),
            global_force=np.array([msg.gfx, msg.gfy, msg.gfz]),
            global_torque=np.array([msg.gtx, msg.gty, msg.gtz]),
            is_contact=msg.is_contact,
            friction_est=msg.friction_est,
            target_grip_force=msg.target_grip_force,
            is_sd_active=msg.is_sd_active,
            pillar_forces=pillar_forces,
            pillar_displacements=pillar_displacements,
            pillar_contacts=pillar_contacts,
            pillar_slip_states=pillar_slip_states,
            pillar_ids=pillar_ids
        )





    def _store_reading_safely(self, reading: TactileReading, hub_id: int, sensor_id: int):
        with self.data_lock:
            key = (hub_id, sensor_id)
            self.sensor_data[key].append(reading)
            self.latest_readings[key] = reading
            
            
            

    # ===========================================================
    # PUBLISHING 
    # ========================================================================
    
    
    
    def _publish_tactile_data(self):
        
        with self.data_lock:
            reading_0 = self.latest_readings.get((0, 0))
            reading_1 = self.latest_readings.get((0, 1))
            
            if reading_0 is None or reading_1 is None:
                return
            
            json_str = self._convert_readings_to_json(reading_0, reading_1)
            
            msg = String()
            msg.data = json_str
            
            self.tactile_pub.publish(msg)
            
            current_time = time.time()
            time_since_last_60hz = current_time - self.last_60hz_publish_time
            
            if time_since_last_60hz >= (1.0 / 60.0):
                self.tactile_60hz_pub.publish(msg)
                self.last_60hz_publish_time = current_time





    def _convert_readings_to_json(self, reading_0: TactileReading, reading_1: TactileReading) -> str:
        avg_timestamp = (reading_0.timestamp + reading_1.timestamp) / 2.0
        
        data = {
            "timestamp": avg_timestamp,
            "sensor_0": self._reading_to_dict(reading_0),
            "sensor_1": self._reading_to_dict(reading_1)
        }
        
        return json.dumps(data)

    def _reading_to_dict(self, reading: TactileReading) -> dict:
        pillars = []
        for i in range(len(reading.pillar_ids)):
            pillar_dict = {
                "id": int(reading.pillar_ids[i]),
                "force": reading.pillar_forces[i].tolist(),
                "displacement": reading.pillar_displacements[i].tolist(),
                "in_contact": bool(reading.pillar_contacts[i]),
                "slip_state": float(reading.pillar_slip_states[i])
            }
            pillars.append(pillar_dict)
        
        return {
            "global_force": reading.global_force.tolist(),
            "global_torque": reading.global_torque.tolist(),
            "is_contact": bool(reading.is_contact),
            "friction_est": float(reading.friction_est),
            "target_grip_force": float(reading.target_grip_force),
            "is_sd_active": bool(reading.is_sd_active),
            "pillars": pillars
        }




    # =============================================================
    # DATA ACCESS AND HELPERS 
    # ========================================================================
    
    
    
    def get_latest_reading(self, hub_id: int, sensor_id: int) -> Optional[TactileReading]:
        
        with self.data_lock:
            return self.latest_readings.get((hub_id, sensor_id))




    def get_all_latest_readings(self) -> Dict[tuple, TactileReading]:
        
        with self.data_lock:
            return {k: v for k, v in self.latest_readings.items() if v is not None}




    def _update_statistics(self):
        
        current_time = time.time()
        time_diff = current_time - self.stats['last_update_time']
    
        if time_diff > 0:
            self.stats['update_frequency'] = self.stats['messages_received'] / time_diff
            
        self.get_logger().info(f"Tactile: {self.stats['update_frequency']:.1f} Hz")
        
        self.stats['messages_received'] = 0
        self.stats['last_update_time'] = current_time



    def calibrate_sensors(self, timeout_seconds=10.0):
        
        self.get_logger().info(" Starting sensor calibration")
        if not self.bias_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(" Bias request service not available!")
            return False
        request = BiasRequest.Request()
        future = self.bias_client.call_async(request)
        start_time = time.time()
        while not future.done() and (time.time() - start_time) < timeout_seconds:
            rclpy.spin_once(self, timeout_sec=0.1)
        if future.done():
            response = future.result()
            if response.result:
                self.get_logger().info(" Sensor calibration is successful")
                return True
            
        return False



    def start_slip_detection(self, timeout_seconds=10.0):
        
        self.get_logger().info(" Starting slip detection")
        if not self.start_sd_client.wait_for_service(timeout_sec=5.0):
            return False
        request = StartSlipDetection.Request()
        future = self.start_sd_client.call_async(request)
        start_time = time.time()
        while not future.done() and (time.time() - start_time) < timeout_seconds:
            rclpy.spin_once(self, timeout_sec=0.1)
        if future.done():
            return future.result().result
        return False



    # ===============================================================
    # High-level information about all sensors
    # ==========================================================================
    def get_combined_tactile_state(self) -> Dict:

        with self.data_lock:
            # Initialize summary
            combined_state = {
                'timestamp': time.time(),
                'sensors': {},
                'summary': {
                    'total_contact_sensors': 0,
                    'max_global_force': 0.0,
                    'average_grip_force': 0.0,
                    'any_slip_detected': False
                }
            }
            
            
            # Process each sensor
            valid_readings = []
            for (hub_id, sensor_id), reading in self.latest_readings.items():
                if reading is not None:
                    sensor_key = f'hub_{hub_id}_sensor_{sensor_id}'
                    combined_state['sensors'][sensor_key] = {
                        'global_force': reading.global_force.tolist(),
                        'global_torque': reading.global_torque.tolist(),
                        'is_contact': reading.is_contact,
                        'friction_est': reading.friction_est,
                        'pillar_forces': reading.pillar_forces.tolist(),
                        'pillar_contacts': reading.pillar_contacts.tolist(),
                        'any_pillar_slip': np.any(reading.pillar_slip_states > 0)
                    }
                    valid_readings.append(reading)
            
            
            # Calculate summary statistics
            if valid_readings:
                combined_state['summary']['total_contact_sensors'] = sum(1 for r in valid_readings if r.is_contact)
                combined_state['summary']['max_global_force'] = max(np.linalg.norm(r.global_force) for r in valid_readings)
                combined_state['summary']['average_grip_force'] = np.mean([r.target_grip_force for r in valid_readings])
                combined_state['summary']['any_slip_detected'] = any(np.any(r.pillar_slip_states > 0) for r in valid_readings)
            
            return combined_state





def main():
    rclpy.init()
    
    # Create the tactile handler
    handler = TactileDataHandler()
    
    time.sleep(1.0)

    
    # CRITICAL: Spin ROS2 to process incoming messages
    rclpy.spin_once(handler, timeout_sec=0.1)
    
        
    # Wait for user to release the sensors
    
    # Calibrate to zero
    if handler.calibrate_sensors():
        print(" Sensors calibrated successfully!")
        handler.start_slip_detection()
        print(" Slip detection started.")
    else:
        print(" Sensor calibration failed! Continuing anyway.")
    
    print("\n Starting sensor monitoring.")
    print(" Publishing to /umi_gripper/tactile (full rate) and /umi_gripper/tactile_60hz (60 Hz)")
    print("="*160)
    
        
    # run ROS2
    try:
        rclpy.spin(handler)
    except KeyboardInterrupt:
        pass
    
    handler.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()