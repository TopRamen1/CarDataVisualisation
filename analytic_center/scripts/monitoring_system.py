#!/usr/bin/env python
import yaml
import rospy, rostopic, rospkg
from dv_interfaces.msg import Monitoring
import csv
import time
import sys

class MonitoringSys:
    def __init__(self, config_path):
        rospy.init_node('dv_monitoring')
        with open(config_path) as f:
            self.config = yaml.load(f)
        self.topics = self.config.get('topics')
        self.refresh_rate = self.config.get('refresh_rate')
        self.mode = self.config.get('mode')
        self.ros_topic_hz = rostopic.ROSTopicHz(-1)
        self.subscribers = {}
        self.publishers = {'general':rospy.Publisher('/topic_stats', Monitoring, queue_size=10)}
        self.messages = {topic_name : {'msg': '', 'updated': False} for topic_name in self.topics}
        self.csv_file_name = time.strftime('%d_%m_%Y_%H_%M_%S'.format(time.gmtime()))
        
        # template for not published message
        self.info_template = Monitoring()
        for topic in self.topics:
            sub = rospy.Subscriber(topic, rospy.AnyMsg, self.msg_callback, callback_args=topic)
            self.subscribers[topic] = sub
            self.publishers[topic] = rospy.Publisher('{}_stats'.format(topic), Monitoring, queue_size=10)
            self.info_template.topic_name = topic
            self.info_template.stamp.secs = 0
            self.info_template.stamp.nsecs = 0
            self.info_template.msg_freq = 0.0
            self.messages[topic]['msg'] = self.info_template

    def msg_callback(self, msg, topic):
        self.ros_topic_hz.callback_hz(m=msg, topic=topic)
        topic_stats = self.ros_topic_hz.get_hz(topic=topic)

        if topic_stats is not None:
            topic_msg = Monitoring()
            topic_msg.topic_name = topic
            topic_msg.stamp = rospy.get_rostime()
            topic_msg.msg_freq = topic_stats[0]
            self.messages[topic]['msg'] = topic_msg
            self.messages[topic]['updated'] = True
    
    def publish_msg(self):
        for topic_name, topic_msg in self.messages.items():
            if not self.messages[topic_name]['updated']:
                self.info_template.topic_name = topic_name
                topic_msg['msg'] = self.info_template 

            self.publishers['general'].publish(topic_msg['msg'])
            self.publishers[topic_name].publish(topic_msg['msg'])
            self.messages[topic_name]['updated'] = False

    def write_csv(self, write_header=True):
        header = ['topic name', 'stamp.secs', 'stamp.nsecs', 'frequency']
        with open('{}/{}.csv'.format(rospack.get_path('analytic_center'), self.csv_file_name), 'a') as f:
            writer = csv.writer(f)
            if write_header:
                writer.writerow(header)
            else:
                for topic_msg in self.messages.values():
                    writer.writerow(
                        [topic_msg['msg'].topic_name, 
                        topic_msg['msg'].stamp.secs, 
                        topic_msg['msg'].stamp.nsecs, 
                        topic_msg['msg'].msg_freq])


if __name__ == "__main__":
    args = sys.argv
    rospack = rospkg.RosPack()
    config_path = rospack.get_path('analytic_center') + '/configs/{}'.format(args[1])
    sys_monitor = MonitoringSys(config_path=config_path)
    if sys_monitor.mode == 'silent':
        sys_monitor.write_csv()

    while not rospy.is_shutdown():

        if sys_monitor.mode == 'silent':
            rospy.sleep(1)
            sys_monitor.write_csv(write_header=False)
        else:
            rospy.sleep(sys_monitor.refresh_rate)
            sys_monitor.publish_msg()