import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Bag2GSDBConverter(Node):
    
        def __init__(self):
                super().__init__("bag2gsdb_converter")

                # create logfile
                self.filename = self.declare_parameter("filename", "output.gsdb").value
                self.get_logger().info(self.filename)
                self.file = open(self.filename, "w")
                self.file.close()
                self.file = open(self.filename, "a")

                self.sub_data_raw = self.create_subscription(String, "/genesys/adma/data_raw", self.data_cb, 10)

        def data_cb(self, msg : String):
                self.file.write(msg.data + "\n")


def main(args=None):
        rclpy.init(args=args)
        l = Bag2GSDBConverter()
        while rclpy.ok():
                rclpy.spin(l)
        l.file.close()


if __name__ == "__main__":
        main()

