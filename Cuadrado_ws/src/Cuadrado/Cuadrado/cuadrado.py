from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point, Quaternion
from std_msgs.msg import String
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from math import radians, sqrt, pow, pi

class Square(Node):

    def __init__(self):

        def callback(self, msg):
            global roll, pitch, yaw
            orientation_q = msg.pose.pose.orientation
            orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
            (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
            p = msg.pose.pose.position
            print(p)
            print(yaw)
            return((Point(p),yaw))

        def talker():

            pub = rclpy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)
            odom_sub = rclpy.Subscriber('/odom', Odometry, callback)
        
            rclpy.init_node('talker', anonymous=True)

            rate = rclpy.Rate(10) # 10hz

            goal_distance = 0.5
            goal_angle = radians(90)
            lin_speed = 0.2
            ang_speed = 0.2

            for i in range(4):

                vel = Twist()
                vel.linear.x = lin_speed
                (position,rotation) = callback()

                x_inicio = position.x
                y_inicio = position.y

                distancia_recorrida = 0

                while distancia_recorrida < goal_distance:
                    pub.publish(vel)

                    # Get the current position
                    (position, rotation) = callback()
                            
                            # Compute the Euclidean distance from the start
                    distancia_recorrida = sqrt(pow((position.x - x_inicio), 2) + 
                                            pow((position.y - y_inicio), 2))

                vel = Twist()
                pub.publish(vel)

                vel.angular.z = ang_speed

                (position, rotation) = callback()

                ang_inicial = rotation
                ang_actual = 0

                while ang_actual < goal_angle:
                    pub.publish(vel)

                    (position, rotation) = callback()

                    ang_actual = rotation - ang_inicial
                    ang_inicial = ang_actual


                vel = Twist()
                pub.publish(vel)
                        

        self.talker()

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = Square()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if _name_ == '_main_':
    try:
        talker()
    except rclpy.ROSInterruptException:
        pass