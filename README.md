# Madgwick-filter-using-STM32F4
Implementation of Madgwick filter using STM32F4 and ICM9048 based on Quaternion arithmetics. 

To calibrate accelerometer and gyro sensors position ICM's X+ axis toward Earth's North pole. To calibrate magnetometer spin sensor arround itself for 15 seconds. Algorithm for magnetometer which is self developed is also based on Quaternion arithmetics.

Output should be represented as Tayt-Brian angles as yaw, pitch and roll. Quaternion norm used in this program is scalar-vector norm, so quaternion q looks like q=[scalar vectorx vectory vectorz].

libraries and sources with tm_ prefix are forked from github.com/MaJerle, Madgwick filter is courtesy of Sebastian Madgwick and everything else is self developed.
