##https://www.adafruit.com/product/2472
import logging
import sys
import time
import rospy

from Adafruit_BNO055 import BNO055
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion, Vector3
covarVal = 1e-6
# Create and configure the BNO sensor connection.  Make sure only ONE of the
# below 'bno = ...' lines is uncommented:
# Raspberry Pi configuration with serial UART and RST connected to GPIO 18:
##bno = BNO055.BNO055(serial_port='/dev/ttyAMA0', rst=18)
# BeagleBone Black configuration with default I2C connection (SCL=P9_19, SDA=P9_20),
# and RST connected to pin P9_12:
#bno = BNO055.BNO055(rst='P9_12')


### Enable verbose debug logging if -v is passed as a parameter.
##if len(sys.argv) == 2 and sys.argv[1].lower() == '-v':
##    logging.basicConfig(level=logging.DEBUG)
##
### Initialize the BNO055 and stop if something went wrong.
##if not bno.begin():
##    raise RuntimeError('Failed to initialize BNO055! Is the sensor connected?')
def IMUPublisher():
    # Read the calibration status, 0=uncalibrated and 3=fully calibrated.
##    sys, gyro, accel, mag = bno.get_calibration_status()
##    # Print everything out.
##    print('Heading={0:0.2F} Roll={1:0.2F} Pitch={2:0.2F}\tSys_cal={3} Gyro_cal={4} Accel_cal={5} Mag_cal={6}'.format(
##          heading, roll, pitch, sys, gyro, accel, mag))
    # Other values you can optionally read:
    # Orientation as a quaternion:
    quaternion = Quaternion()
    quaternion.x, quaternion.y, quaternion.z, quaternion.w = [48,95,98,1.58]
##    quaternion.x, quaternion.y, quaternion.z, quaternion.w = bno.read_quaterion()
    # Sensor temperature in degrees Celsius:
    #temp_c = bno.read_temp()
    # Magnetometer data (in micro-Teslas):
    #x,y,z = bno.read_magnetometer()
    # Gyroscope data (in degrees per second):
    avel = Vector3()
    avel.x,avel.y,avel.z = [1.25,1.30,1.35]
##    avel.x,avel.y,avel.z = bno.read_gyroscope()
    # Accelerometer data (in meters per second squared):
    lacel = Vector3()
    lacel.x, lacel.y, lacel.z = [2.25,2.30,2.35]
##    lacel.x, lacel.y, lacel.z = bno.read_accelerometer()
    # Linear acceleration data (i.e. acceleration from movement, not gravity--
    # returned in meters per second squared):
    #x,y,z = bno.read_linear_acceleration()
    # Gravity acceleration data (i.e. acceleration just from gravity--returned
    # in meters per second squared):
    #x,y,z = bno.read_gravity()
    # Sleep for a second until the next reading.
    imu_msg = Imu()
    imu_msg.header.stamp = rospy.Time.now()
    imu_msg.header.frame_id = "Scorbot_Imu"
    imu_msg.orientation = quaternion
    imu_msg.angular_velocity = avel
    imu_msg.linear_acceleration = lacel
    imu_msg.orientation_covariance = [covarVal, 0, 0, 0, covarVal, 0, 0, 0, covarVal]
    imu_msg.angular_velocity_covariance = [covarVal, 0, 0, 0, covarVal, 0, 0, 0, covarVal]
    imu_msg.linear_acceleration_covariance = [covarVal, 0, 0, 0, covarVal, 0, 0, 0, covarVal]
    pub_imu.publish(imu_msg)
    time.sleep(0.1)
    
### Print system status and self test result.
##status, self_test, error = bno.get_system_status()
##print('System status: {0}'.format(status))
##print('Self test result (0x0F is normal): 0x{0:02X}'.format(self_test))
### Print out an error if system status is in error mode.
##if status == 0x01:
##    print('System error: {0}'.format(error))
##    print('See datasheet section 4.3.59 for the meaning.')
##
### Print BNO055 software revision and other diagnostic data.
##sw, bl, accel, mag, gyro = bno.get_revision()
##print('Software version:   {0}'.format(sw))
##print('Bootloader version: {0}'.format(bl))
##print('Accelerometer ID:   0x{0:02X}'.format(accel))
##print('Magnetometer ID:    0x{0:02X}'.format(mag))
##print('Gyroscope ID:       0x{0:02X}\n'.format(gyro))
##
##print('Reading BNO055 data, press Ctrl-C to quit...')
pub_imu = rospy.Publisher('imu_state', Imu, queue_size=10)
rospy.init_node('leyendoIMU', anonymous=True)
rate = rospy.Rate(20) # 10hz

if __name__ == '__main__':
    while not rospy.is_shutdown():
        try:
            IMUPublisher()
        except rospy.ROSInterruptException:
            pass
