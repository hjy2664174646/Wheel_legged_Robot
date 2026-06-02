#!/usr/bin/env python3
import rclpy
import serial
import struct
from rclpy.node import Node
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
#from transformations import quaternion_from_euler
from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_euler

class SerialBridgeNode(Node):
    def __init__(self):
        super().__init__('serial_bridge_node')

        # Allow port/baud configuration from launch files
        self.declare_parameter('serial_port', '/dev/serial_ttl')
        self.declare_parameter('baudrate', 115200)
        serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value

        self.vx = 0.0
        self.wz = 0.0
        
        # 创建订阅者
        self.cmd_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # 初始化串口
        try:
            self.ser = serial.Serial(
                port=serial_port,
                baudrate=baudrate,
                timeout=0.01,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS,
                xonxoff=False,
                rtscts=False
            )
            self.get_logger().info(f'✅ Serial port {serial_port} opened successfully')
        except Exception as e:
            self.get_logger().error(f'❌ Failed to open serial port: {str(e)}')
            self.ser = None
        
        # 创建定时器（30Hz 发送）
        self.timer = self.create_timer(1.0 / 30, self.send_cmd_timer)
        
        # 数据接收 & 解析 & 发布
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)

        # 启动串口接收定时器
        self.create_timer(0.01, self.serial_read)

        # 用于帧解析
        self.rx_buffer = bytearray()
        
        self.get_logger().info('🚀 SerialBridgeNode started successfully')
        # 创建TF广播器
        self.tf_broadcaster = TransformBroadcaster(self)

    def serial_read(self):
        """从串口读取数据并解析 Odom 帧"""
        if not self.ser:
            return
            
        try:
            data = self.ser.read(60)
        except Exception as e:
            self.get_logger().error(f'Serial read error: {e}')
            return
            
        if not data:
            return

        self.rx_buffer.extend(data)

        # 找帧头：发送端是 0xAA55（uint16_t），小端存储→串口字节为 0x55 0xAA
        while len(self.rx_buffer) >= 60:
            # 正确判断帧头：读取前2字节（小端）是否等于 0xAA55
            if struct.unpack('<H', self.rx_buffer[:2])[0] != 0xAA55:
                self.rx_buffer.pop(0)
                continue

            # 帧总长度：60字节（与发送端结构体大小一致）
            if len(self.rx_buffer) < 60:
                return

            frame = self.rx_buffer[:60]
            
            # 先验证校验和
            checksum_recv = struct.unpack('<H', frame[58:60])[0]
            checksum_calc = sum(frame[2:58]) & 0xFFFF
            
            if checksum_calc != checksum_recv:
                self.get_logger().warn(f"❌ Checksum error: calc={checksum_calc}, recv={checksum_recv}")
                self.rx_buffer.pop(0)  # 只丢1字节，重新搜索帧头
                continue
            
            # 校验通过，移除这帧并解析
            self.rx_buffer = self.rx_buffer[60:]
            self.parse_odom_frame(frame)

    def parse_odom_frame(self, frame):
        """解析 60 字节的 Odom 帧（严格对齐发送端结构体）"""
        try:
            # 1. 验证帧头（二次确认，避免错位）
            head = struct.unpack('<H', frame[0:2])[0]
            if head != 0xAA55:
                self.get_logger().warn("❌ Frame header error")
                return

            # 2. 解析数据（严格按发送端结构体顺序）
            # 里程计核心数据：x→y→yaw→v→w
            x = struct.unpack('<f', frame[2:6])[0]        # 位置x (m)
            y = struct.unpack('<f', frame[6:10])[0]       # 位置y (m)
            yaw_odom = struct.unpack('<f', frame[10:14])[0]  # 里程计偏航角(rad)
            v = struct.unpack('<f', frame[14:18])[0]      # 线速度(m/s)
            w = struct.unpack('<f', frame[18:22])[0]      # 角速度(rad/s)
            
            # IMU 陀螺仪数据（3轴）
            gyro_x = struct.unpack('<f', frame[22:26])[0]  # Gyro[0]
            gyro_y = struct.unpack('<f', frame[26:30])[0]  # Gyro[1]
            gyro_z = struct.unpack('<f', frame[30:34])[0]  # Gyro[2]
            
            # IMU 加速度数据（3轴）
            accel_x = struct.unpack('<f', frame[34:38])[0] # Accel[0]
            accel_y = struct.unpack('<f', frame[38:42])[0] # Accel[1]
            accel_z = struct.unpack('<f', frame[42:46])[0] # Accel[2]
            
            # 姿态角：Pitch→Roll→Yaw（IMU偏航角）
            pitch = struct.unpack('<f', frame[46:50])[0]   # 俯仰角(rad)
            roll = struct.unpack('<f', frame[50:54])[0]    # 横滚角(rad)
            yaw_imu = struct.unpack('<f', frame[54:58])[0] # IMU偏航角(rad)
            
            # 校验和（uint16_t，小端）
            checksum_recv = struct.unpack('<H', frame[58:60])[0]

            # 3. 校验和计算（完全对齐发送端逻辑）
            # 发送端：sum(txBuffer[2:sizeof(odom)-2]) → 即 frame[2:58]（跳过2字节头+2字节校验和）
            checksum_calc = sum(frame[2:58]) & 0xFFFF  # 限制为16位（与发送端uint16_t匹配）

            # 4. 调试信息（精简输出，避免日志刷屏）
            self.get_logger().debug(f"Raw frame: {frame.hex()}")
            self.get_logger().info(
                f"📊 Odom: x={x:.3f} y={y:.3f} yaw={yaw_odom:.3f} v={v:.3f} w={w:.3f}"
            )
            #self.get_logger().info(
            #    f"📊 IMU: Gyro=[{gyro_x:.3f},{gyro_y:.3f},{gyro_z:.3f}] Accel=[{accel_x:.3f},{accel_y:.3f},{accel_z:.3f}]"
            #)
            #self.get_logger().info(
            #    f"📊 Attitude: Pitch={pitch:.3f} Roll={roll:.3f} Yaw={yaw_imu:.3f}"
            #)
            self.get_logger().info(f"🔍 Checksum: calc={checksum_calc} recv={checksum_recv}")

            # 校验和不匹配直接返回
            if checksum_calc != checksum_recv:
                self.get_logger().warn(f"❌ Checksum error: calc={checksum_calc}, recv={checksum_recv}")
                return

            # 5. 发布 Odometry 消息
            odom_msg = Odometry()
            odom_msg.header.stamp = self.get_clock().now().to_msg()
            odom_msg.header.frame_id = "odom"
            odom_msg.child_frame_id = "base_link"

            odom_msg.pose.pose.position.x = x
            odom_msg.pose.pose.position.y = y
            odom_msg.pose.pose.position.z = 0.0
            self.get_logger().info(
                f"📊 odom_msg: x={odom_msg.pose.pose.position.x:.3f} y={odom_msg.pose.pose.position.y:.3f} "
            )
            # 姿态四元数（使用IMU的完整姿态角）
            q = quaternion_from_euler(roll, pitch, yaw_odom)
            odom_msg.pose.pose.orientation.x = q[0]
            odom_msg.pose.pose.orientation.y = q[1]
            odom_msg.pose.pose.orientation.z = q[2]
            odom_msg.pose.pose.orientation.w = q[3]

            # 速度数据
            odom_msg.twist.twist.linear.x = v
            odom_msg.twist.twist.angular.z = w

            self.odom_pub.publish(odom_msg)

            # 6. 发布 IMU 消息
            imu_msg = Imu()
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = "base_link"
            
            # 角速度（rad/s）
            imu_msg.angular_velocity.x = gyro_x
            imu_msg.angular_velocity.y = gyro_y
            imu_msg.angular_velocity.z = gyro_z
            
            # 线加速度（m/s²）
            imu_msg.linear_acceleration.x = accel_x
            imu_msg.linear_acceleration.y = accel_y
            imu_msg.linear_acceleration.z = accel_z
            
            # 姿态四元数
            q_imu = quaternion_from_euler(roll, pitch, yaw_imu)
            imu_msg.orientation.x = q_imu[0]
            imu_msg.orientation.y = q_imu[1]
            imu_msg.orientation.z = q_imu[2]
            imu_msg.orientation.w = q_imu[3]
            
            self.imu_pub.publish(imu_msg)

            # 7. 广播 TF 变换
            self.broadcast_tf(x, y, 0.0, 0.0, yaw_odom)

            self.get_logger().info("✅ Successfully published Odom + IMU + TF\n")

        except Exception as e:
            self.get_logger().error(f"❌ Parse frame error: {e}")
            import traceback
            self.get_logger().error(traceback.format_exc())

    def cmd_vel_callback(self, msg: Twist):
        self.vx = msg.linear.x
        self.wz = msg.angular.z
        self.get_logger().info(f'📥 Received cmd_vel: vx={self.vx:.3f}, wz={self.wz:.3f}')

    def send_cmd_timer(self):
        vx = self.vx
        wz = self.wz
        
        # 二进制帧构建（对齐STM32接收端的解析逻辑）
        payload = struct.pack('<Bff', 0x01, vx, wz)  # 类型0x01 + 速度数据
        length = len(payload) + 1  # payload长度 + 1字节校验和
        frame = bytearray([0xAA, 0x55, length])  # 帧头（AA 55）+ 长度
        frame.extend(payload)
        
        # XOR校验（从长度字段开始计算，对齐接收端check_sum函数）
        checksum = 0
        for b in frame[2:]:  # 跳过帧头AA 55，从length开始XOR
            checksum ^= b
        frame.append(checksum)
        
        # 串口发送
        if self.ser and self.ser.is_open:
            try:
                bytes_sent = self.ser.write(frame)
                self.get_logger().debug(f'📤 Sent {bytes_sent} bytes: {frame.hex()}')
            except Exception as e:
                self.get_logger().error(f'❌ Send failed: {str(e)}')
    
    def broadcast_tf(self, x, y, roll, pitch, yaw):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"

        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 0.0

        q = quaternion_from_euler(roll, pitch, yaw)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = SerialBridgeNode()
    rclpy.spin(node)
    if node.ser and node.ser.is_open:
        node.ser.close()
        node.get_logger().info('🔌 Serial port closed')
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
