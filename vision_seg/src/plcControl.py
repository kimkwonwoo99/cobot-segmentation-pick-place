#!/usr/bin/env python

import serial
import rospy
from std_msgs.msg import String

class MotorControl:
    def __init__(self):
        rospy.init_node('order_accept', anonymous=False)
        
        try:
            # self.ser = serial.Serial('/dev/ttyUSB0', 38400, timeout=1)
            rospy.loginfo("Serial port initialized")
        except serial.SerialException as e:
            rospy.logerr("Failed to initialize serial port: %s", e)
            rospy.signal_shutdown("Serial port error")
            return

        self.ordersub = rospy.Subscriber('/plc_control', String, self.rotate_callback)

    def rotate_callback(self, msg):
        try:
            if msg.data == 'stop':
                self.ser.write(b'\x0500WSS0108%DW000000000\x04')
                rospy.loginfo("Motor Stopped")
            elif msg.data == 'cw':
                self.ser.write(b'\x0500WSS0108%DW000000001\x04')
                rospy.loginfo("Motor Running CW")
            elif msg.data == 'ccw':
                self.ser.write(b'\x0500WSS0108%DW000000002\x04')
                rospy.loginfo("Motor Running CCW")
            else:
                rospy.logwarn("Unknown Order: %s", msg.data)
        except serial.SerialException as e:
            rospy.logerr("Serial communication error: %s", e)

    def close_serial(self):
        if self.ser.is_open:
            self.ser.close()
            rospy.loginfo("Serial port closed")

if __name__ == '__main__':
    try:
        cont = MotorControl()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("End of Control")
    finally:
        cont.close_serial()
