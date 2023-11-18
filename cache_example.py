import rospy
import message_filters
from std_msgs.msg import String
from rospy.rostime import Time

def cache_example():
    rospy.init_node('cache_example_node', anonymous=True)

    # 设置缓存大小，例如 10 个消息
    cache_size = 10
    sub = message_filters.Subscriber('your_topic', String)
    cache = message_filters.Cache(sub, cache_size)
    desired_time = Time.now()
    cached_messages = cache.getInterval(desired_time, desired_time)

    # 处理查询到的消息
    for msg in cached_messages:
        print("Cached message:", msg.data)

    rospy.spin()

if __name__ == '__main__':
    cache_example()
