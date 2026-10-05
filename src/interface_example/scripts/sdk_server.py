#!/usr/bin/env python3
"""
宿主机速度中转程序：
1. 接收容器UDP发送的vx/vy/wz
2. 解析后发布为/motion/body_vel_cmd话题（适配SDK）
3. 新增：超时清零+速度有效性判断，解决"停发后仍走"问题
4. 日志优化：所有速度相关日志改为INFO级别，添加心跳日志
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from interface_protocol.msg import BodyVelCmd
import socket
import json
import threading
import time

class SpeedForwarder(Node):
    def __init__(self):
        super().__init__('g1_speed_forwarder')

        # ====================== 1. ROS发布器 ======================
        self.publisher_ = self.create_publisher(
            BodyVelCmd,
            '/motion/body_vel_cmd',
            10
        )

        # ====================== 2. UDP服务端配置 ======================
        self.udp_host = "0.0.0.0"
        self.udp_port = 8888
        self.udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_sock.bind((self.udp_host, self.udp_port))
        self.udp_sock.setblocking(False)

        # ====================== 3. 核心变量（新增超时/速度阈值） ======================
        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0
        self.last_udp_time = time.time()  # 最后一次收到UDP的时间
        self.udp_timeout = 0.5  # UDP超时时间（0.5秒没收到则清零）
        self.speed_threshold = 0.01  # 速度阈值（小于此值视为0）
        self.last_heartbeat = time.time()  # 心跳日志时间戳

        # ====================== 4. 启动UDP接收线程 ======================
        self.udp_thread = threading.Thread(target=self._udp_listen_loop, daemon=True)
        self.udp_thread.start()

        # ====================== 5. 发布定时器（100Hz） ======================
        self.timer_period = 0.01
        self.timer = self.create_timer(self.timer_period, self._publish_vel_cmd)

        self.get_logger().info(f"""
✅ 宿主机速度中转程序启动完成
- UDP监听：{self.udp_host}:{self.udp_port}
- ROS发布话题：/motion/body_vel_cmd
- 发布频率：100Hz
- UDP超时清零：{self.udp_timeout}秒
        """)

    def _is_speed_valid(self):
        """判断速度是否有效（大于阈值）"""
        return abs(self.vx) > self.speed_threshold or abs(self.vy) > self.speed_threshold or abs(self.wz) > self.speed_threshold

    def _udp_listen_loop(self):
        """UDP接收循环（新增：更新最后接收时间+心跳日志）"""
        while rclpy.ok():
            try:
                data, addr = self.udp_sock.recvfrom(1024)
                speed_data = json.loads(data.decode("utf-8"))
                # 更新速度+最后接收时间
                self.vx = speed_data.get("vx", 0.0)
                self.vy = speed_data.get("vy", 0.0)
                self.wz = speed_data.get("wz", 0.0)
                self.last_udp_time = time.time()  # 重置超时时间
                # 改为INFO级别，直接显示接收的速度
                self.get_logger().info(f"📥 接收容器[{addr}]速度：vx={self.vx:.2f}, vy={self.vy:.2f}, wz={self.wz:.2f}")
            except BlockingIOError:
                # 添加心跳日志：每5秒打印一次，确认线程在运行
                if time.time() - self.last_heartbeat > 5:
                    self.get_logger().info("🟢 UDP监听中，暂未收到数据...")
                    self.last_heartbeat = time.time()
                threading.Event().wait(0.001)
            except json.JSONDecodeError:
                self.get_logger().warn("⚠️ 接收无效JSON数据，跳过")
            except Exception as e:
                self.get_logger().error(f"❌ UDP接收异常: {e}")

    def _publish_vel_cmd(self):
        """定时器回调：新增超时清零+仅发布有效速度+INFO级别日志"""
        # 1. 超时判断：0.5秒没收到UDP则强制清零
        if time.time() - self.last_udp_time > self.udp_timeout:
            # 仅当速度非零时打印超时清零日志（避免刷屏）
            if self._is_speed_valid():
                self.get_logger().info("⏹️ UDP超时（>0.5秒），速度已清零")
            self.vx = 0.0
            self.vy = 0.0
            self.wz = 0.0

        # 2. 仅当速度有效时才发布（避免持续发0速度）
        if not self._is_speed_valid():
            return  # 无有效速度，不发布

        # 3. 正常发布速度指令
        msg = BodyVelCmd()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'body'
        msg.linear_velocity = [self.vx, self.vy]
        msg.yaw_velocity = self.wz

        self.publisher_.publish(msg)
        # 改为INFO级别，显示发布的速度
        self.get_logger().info(f"📤 发布速度到/motion/body_vel_cmd：vx={self.vx:.2f}, vy={self.vy:.2f}, wz={self.wz:.2f}")

def main(args=None):
    rclpy.init(args=args)
    speed_forwarder = SpeedForwarder()
    try:
        rclpy.spin(speed_forwarder)
    except KeyboardInterrupt:
        speed_forwarder.get_logger().info("🛑 程序被键盘中断，正在退出...")
    finally:
        speed_forwarder.udp_sock.close()
        speed_forwarder.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
