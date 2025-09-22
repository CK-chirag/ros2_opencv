import argparse, socket, time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class TCPSender(Node):
    def __init__(self, host, port):
        super().__init__('tcp_sender')
        self.host = host
        self.port = port
        self.sub = self.create_subscription(String, '/yolo/detections', self.cb_detections, 10)
        self.sock = None
        self.connect()

    def connect(self):
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(5.0)
            self.sock.connect((self.host, self.port))
            self.get_logger().info(f'Connected to {self.host}:{self.port}')
        except Exception as e:
            self.get_logger().warning(f'Connection failed: {e}')
            self.sock = None

    def cb_detections(self, msg: String):
        data = msg.data.strip()
        if not data:
            return
        if self.sock is None:
            self.connect()
            if self.sock is None:
                return
        try:
            # send each JSON terminated by newline for easy delimiting
            self.sock.sendall((data + '\n').encode('utf-8'))
        except Exception as e:
            self.get_logger().error(f'Send error: {e}')
            try:
                self.sock.close()
            except:
                pass
            self.sock = None

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--host', required=True, help='Receiver host IP')
    parser.add_argument('--port', required=True, type=int, help='Receiver port')
    args = parser.parse_args()

    rclpy.init()
    node = TCPSender(args.host, args.port)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

