
import cv2
import qrcode
import numpy as np
import json
import time
from datetime import datetime   
import argparse

# Add ROS2
import rclpy
from rclpy.node import Node


class ContinuousQRDisplay(Node):  # ← Inherit from Node
    def __init__(self, update_rate_hz=60):
        # Initialize ROS node
        super().__init__('continuous_qr_display')
        
        self.update_rate_hz = update_rate_hz
        self.update_interval = 1.0 / update_rate_hz
        self.running = False
        
        # Display settings
        self.display_height = 800
        self.display_width = 800
        self.qr_size = 600
        
        # Pre-create QR generator
        self.qr_generator = qrcode.QRCode(
            version=1,
            error_correction=qrcode.constants.ERROR_CORRECT_L,
            box_size=10,
            border=4,
            
        )
        
        # Create window (normal mode - movable)
        self.window_name = 'UMI QR Sync - Continuous'
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.window_name, 800, 800)
        cv2.setWindowProperty(
            self.window_name,
            cv2.WND_PROP_TOPMOST,
            1
        )
        
        self.get_logger().info("✓ Continuous QR Display initialized (ROS2)")
        self.get_logger().info(f"  Update rate: {update_rate_hz} Hz")
        self.get_logger().info(f"  Using ROS time for sync")
    
    
    def get_ros_timestamp(self):
        """Get current ROS timestamp as float seconds."""
        ros_time = self.get_clock().now()
        return ros_time.nanoseconds / 1e9  # Convert to seconds
    
    
    def generate_qr_image(self, timestamp):
        """Generate QR code image for given timestamp."""
        qr_data = {
            'timestamp': timestamp,
            'time_source': 'ROS'  
        }
        
        qr_string = json.dumps(qr_data)
        
        # Generate QR
        self.qr_generator.clear()
        self.qr_generator.add_data(qr_string)
        self.qr_generator.make(fit=True)
        
        # Create image
        qr_img = self.qr_generator.make_image(
            fill_color="black",
            back_color="white"
        )
        
        return np.array(qr_img.convert('RGB'))
    
    
    def create_display_frame(self, qr_array, timestamp):
        """Create full display frame with QR and text."""
        # White background
        display_img = np.ones(
            (self.display_height, self.display_width, 3),
            dtype=np.uint8
        ) * 255
        
        # Resize QR
        qr_resized = cv2.resize(
            qr_array,
            (self.qr_size, self.qr_size),
            interpolation=cv2.INTER_NEAREST
        )
        
        # Place QR in center
        y_offset = 50
        x_offset = (self.display_width - self.qr_size) // 2
        display_img[
            y_offset:y_offset+self.qr_size,
            x_offset:x_offset+self.qr_size
        ] = qr_resized
        
        # Add text
        
        
        time_text = f"Time: {timestamp:.3f}"
        cv2.putText(
            display_img, time_text, (50, 750),
            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 2
        )
        
        # Add update rate indicator
        rate_text = f"{self.update_rate_hz} Hz (ROS)"
        cv2.putText(
            display_img, rate_text, (550, 750),
            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (128, 128, 128), 1
        )
        
        return display_img
    
    
    def run(self):
        """Main loop - continuously update and display QR."""
        self.running = True
        
        print("\n" + "="*60)
        print(" CONTINUOUS QR DISPLAY RUNNING (ROS2)")
        print("="*60)
        print("="*60 + "\n")
        
        frame_count = 0
        start_time = time.time()
        
        try:
            while self.running and rclpy.ok():
                loop_start = time.time()
                
                # Get ROS timestamp (synchronized!)
                current_timestamp = self.get_ros_timestamp()
                
                # Generate QR for this timestamp
                qr_array = self.generate_qr_image(current_timestamp)
                
                # Create display frame
                display_frame = self.create_display_frame(qr_array, current_timestamp)
                
                # Show
                cv2.imshow(self.window_name, display_frame)
                
                # Check for keys
                key = cv2.waitKey(1)
                if key == ord('q') or key == 27:
                    break
                elif key == ord('f'):
                    current_prop = cv2.getWindowProperty(
                        self.window_name, 
                        cv2.WND_PROP_FULLSCREEN
                    )
                    if current_prop == cv2.WINDOW_FULLSCREEN:
                        cv2.setWindowProperty(
                            self.window_name, 
                            cv2.WND_PROP_FULLSCREEN, 
                            cv2.WINDOW_NORMAL
                        )
                        print("Fullscreen OFF")
                    else:
                        cv2.setWindowProperty(
                            self.window_name, 
                            cv2.WND_PROP_FULLSCREEN, 
                            cv2.WINDOW_FULLSCREEN
                        )
                        print(" Fullscreen ON")
                
                # Maintain update rate
                elapsed = time.time() - loop_start
                sleep_time = max(0, self.update_interval - elapsed)
                if sleep_time > 0:
                    time.sleep(sleep_time)
                
                # Spin ROS (process callbacks)
                rclpy.spin_once(self, timeout_sec=0)
                
                # Stats
                frame_count += 1
                if frame_count % 50 == 0:
                    actual_rate = frame_count / (time.time() - start_time)
                    print(f"{actual_rate:.1f} Hz | ROS time: {current_timestamp:.3f}")
        
        except KeyboardInterrupt:
            print("\n Interrupted by user")
        
        finally:
            self.stop()
    
    
    def stop(self):
        """Clean shutdown."""
        self.running = False
        cv2.destroyWindow(self.window_name)
        self.get_logger().info("Continuous QR Display stopped")


def main():
    parser = argparse.ArgumentParser(
        description='Continuous QR Display for UMI Tactile Sync'
    )
    parser.add_argument(
        '--rate',
        type=int,
        default=60,
        help='Update rate in Hz defult 60'
    )
    
    args = parser.parse_args()
    
    # Validate rate
    if args.rate < 1 or args.rate >= 60:
        print("Warning: Rate should be between 1-60 Hz, using 60 Hz")
        args.rate = 60
    
    # Initialize ROS2
    rclpy.init()
    
    # Create and run display
    display = ContinuousQRDisplay(update_rate_hz=args.rate)
    
    try:
        display.run()
    finally:
        display.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()