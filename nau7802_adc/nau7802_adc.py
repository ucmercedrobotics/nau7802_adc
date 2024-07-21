import sys
import board
from cedargrove_nau7802 import NAU7802

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Int32
from std_srvs.srv import Trigger
from geometry_msgs.msg import Wrench, WrenchStamped

class Nau7802Node(Node):
    """
    ROS2 publisher node for a load cell via the Adafruit NAU7802 24-bit ADC breakout board

    The class publishes both the raw ADC values and a force 
    via a WrenchStamped message. 
    NOTE: This node is only configured to read and publish
    the contents of the *first* channel of the ADC. Adding the second channel
    is trivial, PRs welcome.
    """
    def __init__(self) -> None:
        super().__init__("nau7802")    # type: ignore
        self.poll_interval = 0.2 # seconds

        # Initialize the device
        self.nau7802 = NAU7802(i2c_bus=board.I2C(), address=0x2A, active_channels=1)
        # Enable NAU7802 digital and analog power
        nau_enabled: bool = self.nau7802.enable(power=True)
        self.get_logger().info(f'Digital and analog power enabled: {nau_enabled}')
        # Tare the load cell (zero the input channel)
        self.zero_channel()
        
        # Initialize the messages and create the publishers
        self.msg_loadcell = WrenchStamped()
        self.msg_adc1 = Int32()
        self.msg_loadcell.header.stamp = self.get_clock().now().to_msg()
        self.publisher_nau7802adc1 = self.create_publisher(msg_type=Int32, 
            topic='/nau7802/adc1', qos_profile=10)
        self.publisher_nau7802load = self.create_publisher(msg_type=WrenchStamped, 
            topic='/nau7802/load', qos_profile=10)

        self.timer = self.create_timer(self.poll_interval, self.nau7802_callback)

        self.get_logger().info(
            f'channel {self.nau7802.channel} ready'
        )

        self._zero_channel_service = self.create_service(
            srv_type=Trigger,
            srv_name="nau7802_tare",
            callback=self.zero_channel_service_callback)

    def zero_channel_service_callback(self, _, response):
        if self.zero_channel(channel=1):
            response.success = True
            return response
        else:
            response.success = False
            return response

    def zero_channel(self, channel=1):
        """Initiate internal + external calibration for current channel.

        The NAU7802 can perform an internal and external (offset) calibration.
        This function runs BOTH calibration procedures.
        The internal ADC calibration deals with the amplifier gain and 
        offset errors. The external calibrarion sets an offset relative to the
        current input voltage. Internal calibration should be done to compensate
        for measurememt drift (eg. due to temperature). External calibration
        is what is done when a scale is tared (seting the zero point).
        """
        self.get_logger().info(
            f'channel {self.nau7802.channel}, calibrate.INTERNAL '
            f'{self.nau7802.calibrate(mode="INTERNAL")}'
        )
        self.get_logger().info(
            f'channel {self.nau7802.channel}, calibrate.OFFSET '
            f'{self.nau7802.calibrate(mode="OFFSET")}'
        )
        self.get_logger().info(f'...channel {self.nau7802.channel} zeroed')

        return True

    def read_raw_value_avg(self, samples=2) -> int:
        """Read and average consecutive raw sample values. 
        
        Return the average raw value for n number of last samples.
        """
        sample_sum = 0
        sample_count: int = samples
        while sample_count > 0:
            while not self.nau7802.available():
                pass
            sample_sum = sample_sum + self.nau7802.read()
            sample_count -= 1
        return int(sample_sum / samples)

    def nau7802_callback(self) -> None:
        """
        NAU7802 timer callback

        Reads the sensor values and publishes the messages at 
        the node polling interval
        """

        self.msg_loadcell = WrenchStamped()
        self.msg_loadcell.header.stamp = self.get_clock().now().to_msg()
        raw_value: int =  self.read_raw_value_avg()
        self.msg_loadcell.wrench.force.z = float(raw_value)

        self.publisher_nau7802load.publish(msg=self.msg_loadcell)
        self.publisher_nau7802adc1.publish(msg=self.msg_adc1)


def main(args=None) -> None:
    rclpy.init(args=args)

    nau7802_pub: Nau7802Node = Nau7802Node()

    try:
        rclpy.spin(nau7802_pub) 
    except KeyboardInterrupt:
        print("**** 💀 Ctrl-C detected... killing process ****")
        pass
    except ExternalShutdownException:
        sys.exit(1)
    finally:
        print("**** 🪦 process dead ****")
        # Destroy node explicitly, dont wait for GC
        nau7802_pub.destroy_node()
        rclpy.try_shutdown()

if __name__ == "__main__":
    main()