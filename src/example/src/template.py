# import modules
import rospy

class ClassName():

    def __init__(self):

        rospy.init_node("Class name", anonymous = False)
        # rospu.Publisher/Subscriber("")

        rospy.spin()
        pass

    def callback(self):
        pass 

    def run(self):

        rospy.loginfo("Starting ... node")
        rospy.spin()

    def callback():
        pass

    
if __name__ == '__main__':

    try:
        node = ClassName()
        node.run()
    except rospy.ROSInterruptException:
        pass