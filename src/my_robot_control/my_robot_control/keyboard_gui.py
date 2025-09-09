import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import tkinter as tk

class KeyboardGUI(Node):
    def __init__(self):
        super().__init__('keyboard_gui')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        self.speed = 0.5          # m/s
        self.angular_speed = 1.57 # rad/s (≈90°/s)

        self.current_linear = 0.0
        self.current_angular = 0.0
        self.last_command = "None"

        # GUI
        self.root = tk.Tk()
        self.root.title("Robot Keyboard Controller")

        self.label = tk.Label(self.root, text="Press F/B/L/R/S", font=("Arial", 14))
        self.label.pack(pady=10)

        self.status = tk.Label(
            self.root, 
            text="Speed: 0.0 | Angular: 0.0 | Last Command: None", 
            font=("Arial", 12)
        )
        self.status.pack(pady=10)

        # Bind keys
        self.root.bind("<f>", lambda event: self.move_forward())
        self.root.bind("<b>", lambda event: self.move_backward())
        self.root.bind("<s>", lambda event: self.stop())
        self.root.bind("<l>", lambda event: self.turn_left())
        self.root.bind("<r>", lambda event: self.turn_right())

        tk.Button(self.root, text="Forward (F)", command=self.move_forward, width=20).pack(pady=2)
        tk.Button(self.root, text="Backward (B)", command=self.move_backward, width=20).pack(pady=2)
        tk.Button(self.root, text="Stop (S)", command=self.stop, width=20).pack(pady=2)
        tk.Button(self.root, text="Left (L)", command=self.turn_left, width=20).pack(pady=2)
        tk.Button(self.root, text="Right (R)", command=self.turn_right, width=20).pack(pady=2)

        self.timer = self.create_timer(0.1, self.update_gui)

    def update_gui(self):
        self.status.config(
            text=f"Speed: {self.current_linear:.2f} | Angular: {self.current_angular:.2f} | Last Command: {self.last_command}"
        )
        self.root.update()

    def publish_cmd(self, linear_x=0.0, angular_z=0.0, command=""):
        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        self.publisher_.publish(twist)

        self.current_linear = linear_x
        self.current_angular = angular_z
        self.last_command = command

        self.get_logger().info(f"Command: {command} | lin={linear_x:.2f}, ang={angular_z:.2f}")

    def move_forward(self):
        self.publish_cmd(self.speed, 0.0, "Forward")

    def move_backward(self):
        self.publish_cmd(-self.speed, 0.0, "Backward")

    def stop(self):
        self.publish_cmd(0.0, 0.0, "Stop")

    def turn_left(self):
        self.publish_cmd(0.0, self.angular_speed, "Turn Left")

    def turn_right(self):
        self.publish_cmd(0.0, -self.angular_speed, "Turn Right")

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardGUI()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
