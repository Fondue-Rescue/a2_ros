import psutil
import rclpy
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue


class SystemMonitor(Node):

    def __init__(self):
        super().__init__('system_monitor')
        self.declare_parameter('publish_rate', 1.0)
        rate = self.get_parameter('publish_rate').value

        self.pub = self.create_publisher(DiagnosticArray, '/system/cpu_ram_usage', 10)
        self.timer = self.create_timer(1.0 / rate, self.publish_usage)
        self.get_logger().info(f'System monitor started at {rate} Hz')

    def publish_usage(self):
        cpu_percent = psutil.cpu_percent(interval=None)
        mem = psutil.virtual_memory()

        cpu_status = DiagnosticStatus()
        cpu_status.name = 'CPU Usage'
        cpu_status.hardware_id = 'cpu'
        cpu_status.level = (
            DiagnosticStatus.OK if cpu_percent < 80.0
            else DiagnosticStatus.WARN if cpu_percent < 95.0
            else DiagnosticStatus.ERROR
        )
        cpu_status.message = f'{cpu_percent:.1f}%'
        cpu_status.values = [
            KeyValue(key='total_percent', value=f'{cpu_percent:.1f}'),
            KeyValue(key='core_count', value=str(psutil.cpu_count(logical=True))),
        ]
        per_cpu = psutil.cpu_percent(percpu=True)
        for i, pct in enumerate(per_cpu):
            cpu_status.values.append(
                KeyValue(key=f'core_{i}_percent', value=f'{pct:.1f}')
            )

        mem_status = DiagnosticStatus()
        mem_status.name = 'RAM Usage'
        mem_status.hardware_id = 'memory'
        mem_status.level = (
            DiagnosticStatus.OK if mem.percent < 80.0
            else DiagnosticStatus.WARN if mem.percent < 95.0
            else DiagnosticStatus.ERROR
        )
        mem_status.message = f'{mem.percent:.1f}%'
        mem_status.values = [
            KeyValue(key='total_gb', value=f'{mem.total / (1024**3):.2f}'),
            KeyValue(key='used_gb', value=f'{mem.used / (1024**3):.2f}'),
            KeyValue(key='available_gb', value=f'{mem.available / (1024**3):.2f}'),
            KeyValue(key='percent', value=f'{mem.percent:.1f}'),
        ]

        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.status = [cpu_status, mem_status]
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SystemMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
