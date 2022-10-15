class listener():
    def __init__(self):

        rospy.init_node('listener', anonymous=True)
    
        rospy.Subscriber('chatter', String, self.callback)
        print('black')
        pub = rospy.Publisher('my_counter', Int32, queue_size= 10)

        rate = rospy.Rate(2)
        
        count = 0

        while not rospy.is_shutdown():
            print('orange')
            pub.publish(count)
            count += 1
            rate.sleep()
    
        # rospy.spin()

    def callback(self, data):
        rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.data)
        print('blue')

if __name__ == '__main__':
    listener()