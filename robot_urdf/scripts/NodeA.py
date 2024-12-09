import rospy
from geometry_msgs.msg import Twist

# Initialize the ROS node
rospy.init_node('NodeA')

# Set up the publisher to publish Twist messages on your desired topic
velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

# Create a Twist message instance
velocity_msg = Twist()

# Define the desired linear and angular velocities
linear_vel = 0.0  # Example: 0.5 m/s
angular_vel = 0.1  # Example: 0.1 rad/s

# Set the linear and angular velocities in the Twist message
velocity_msg.linear.x = linear_vel
velocity_msg.angular.z = angular_vel

# Publish the message in a loop
rate = rospy.Rate(10)  # 10 Hz

while not rospy.is_shutdown():
    velocity_publisher.publish(velocity_msg)
    rate.sleep()

