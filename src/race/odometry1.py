#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray


twist_msg = Twist()

def shutdown_hook():
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    twist_msg.linear.x = 0.0 # Detener el movimiento lineal
    twist_msg.angular.z = 0.0 # Detener el movimiento angular
    print('ShutDown')
    pub.publish(twist_msg)

def controls_callback(msg):
    twist_msg.linear.x = msg.data[0] # Modificar la velocidad lineal
    twist_msg.angular.z = msg.data[1] # Modificar la velocidad angular
    print('Velocities:['+str(msg.data[0])+','+str(msg.data[1])+']')
def controls():
    rospy.init_node('cmd_vel_publisher', anonymous=True)
    rospy.Subscriber('velocities',Float32MultiArray,controls_callback)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.on_shutdown(shutdown_hook) # Función que se llamará al cerrar el nodo
    rate = rospy.Rate(10) # Frecuencia de publicación del mensaje
    while not rospy.is_shutdown():        
        print('Escuchando')
        pub.publish(twist_msg)
        rate.sleep()
    rospy.spin()

if __name__ == '__main__':
    try:
        controls()
    
    except rospy.ROSInterruptException:
        pass