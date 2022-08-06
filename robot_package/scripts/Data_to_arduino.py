import rospy
from robot_package.msg import Control_signals
import serial

ser = serial.Serial("/dev/ttyACM0", 9600, timeout=1)


def callback(data):
    v = data.v
    w = data.w
    message = str(v) + "," + str(w) + " "
    ser.write(message.encode('utf-8'))


def send():
    rospy.init_node("Serial_Communication", anonymous=False)
    rospy.Subscriber("/control_signals", Control_signals, callback)
    rospy.spin()


if __name__ == '__main__':
    send()
