#! /usr/bin/env python

__author__ = "Fabio Giuseppe Di Benedetto"

import socket
import select
import Queue
import re
import json
import subprocess
import rospy
from robotics_msgs.msg import Robot_Event
from robotics_msgs.msg import CB_Event
from robotics_msgs.srv import FIROS_Info, FIROS_InfoResponse


class RCMDriver:

    DEFAULT_NODE_NAME = "rcmdriver"
    DEFAULT_OUT_TOPIC = "/rcm/robot_event"
    DEFAULT_IN_TOPIC = "/firos/cb_event"
    DEFAULT_SERVICE = "/rcm/firos_info"
    DEFAULT_QUEUE_SIZE = 10
    #
    PARAM_PORT = "port="
    # these are constants duplicated from RCMDriver and used to deal with that
    # component
    COMMAND_KEY = "rcm_d_cmd"
    NOTIFY_CONNECTION = "r_connection"
    NOTIFY_DISCONNECTION = "r_disconnection"
    R_NAME_KEY = "r_name"
    ENTITY_CREATED = "e_created"
    ENTITY_REMOVED = "e_removed"
    ASK_INFO = "r_info"
    R_INFO_KEY = "r_info_data"
    # for topics extraction
    RT_NAME_KEY = "ros_topic_name"
    RT_MSG_KEY = "ros_topic_message"
    # commonly used flags
    READ_ONLY = select.POLLIN | select.POLLPRI | select.POLLHUP | select.POLLERR
    READ_WRITE = READ_ONLY | select.POLLOUT
    DEFAULT_DRIVER_PORT = 9998

    def __init__(self, port=DEFAULT_DRIVER_PORT):
        rospy.init_node(self.DEFAULT_NODE_NAME)
        argv = rospy.myargv()
        p_port = None
        if len(argv) > 1:
            # there are parameters passed
            for arg in argv:
                if self.PARAM_PORT in arg:
                    tmp = arg.replace(self.PARAM_PORT, "")
                    if tmp.isdigit():
                        # not digits means the port is wrong, so we don't set it
                        p_port = tmp
                        rospy.loginfo("rcm driver listen to rcm platform node on port %s" % p_port)
        self.rcm_driver_s_server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.rcm_driver_s_server.setblocking(0)
        self.rcm_driver_s_server_address = ("localhost", int(p_port) if p_port else port)
        self.rcm_driver_s_server.bind(self.rcm_driver_s_server_address)
        # listen for incoming connection (only one)
        self.rcm_driver_s_server.listen(1)
        self.connection = None
        self.client_address = None
        self._buf_size = 1024
        self.rcm_re_publisher = None
        self.rcm_fi_service = None
        self.robot_name = None
        self._robot_status = Queue.Queue()
        self._entity_status = Queue.Queue()
        # we get a polling object
        self.po = select.poll()
        # register the server on read only events
        self.po.register(self.rcm_driver_s_server, self.READ_ONLY)
        # map file descriptors to socket objects
        self.fd_to_socket = {self.rcm_driver_s_server.fileno(): self.rcm_driver_s_server}
        rt_name_extractor_str = '(?P<%s>[a-zA-Z0-9\_\/]+)' % self.RT_NAME_KEY
        rt_msg_extractor_str = '(?P<%s>[a-zA-Z0-9\_\/]+)' % self.RT_MSG_KEY
        elements_str = '[0-9]+'
        pub_str = 'publisher[s]?'
        sub_str = 'subscriber[s]?'
        rt_publisher_extractor_str = ' * %s \[%s\] %s %s' % \
                                     (rt_name_extractor_str, rt_msg_extractor_str, elements_str, pub_str)
        rt_subscriber_extractor_str = ' * %s \[%s\] %s %s' % \
                                      (rt_name_extractor_str, rt_msg_extractor_str, elements_str, sub_str)
        self.rt_publisher_extractor_re = re.compile(rt_publisher_extractor_str)
        self.rt_subscriber_extractor_re = re.compile(rt_subscriber_extractor_str)

    def cb_event_handler(self, data):
        if data.entity_name != self.robot_name:
            rospy.loginfo("firos changed the robot name from %s to %s" % (self.robot_name, data.entity_name))
        import json
        if data.entity_status == data.CREATED:
            # we notify the entity creation so that the node can go on (watchdog can start
            # to ping the other end to check the connection status)
            msg_dict = {self.COMMAND_KEY: self.ENTITY_CREATED, self.R_NAME_KEY: self.robot_name}
        else:
            # we notify the entity deletion so that the node can go on in the clean up
            msg_dict = {self.COMMAND_KEY: self.ENTITY_REMOVED, self.R_NAME_KEY: self.robot_name}
        request = json.dumps(msg_dict)
        self._entity_status.put_nowait(request)

    def firos_info_handler(self, request):
        if request.instance_name != self.robot_name:
            rospy.loginfo("firos changed the robot name from %s to %s" % (self.robot_name, request.instance_name))
        cmd = ['rostopic', 'list', '-v']
        tl_p = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        tl_p_out, tl_p_err = tl_p.communicate()
        rtn_code = tl_p.poll()
        topics = []
        if not rtn_code:
            # not errors
            m_pub_list = self.rt_publisher_extractor_re.finditer(tl_p_out)
            if m_pub_list is not None:
                for m in m_pub_list:
                    if not (m.group(self.RT_NAME_KEY) == self.DEFAULT_OUT_TOPIC
                            or m.group(self.RT_NAME_KEY) == self.DEFAULT_IN_TOPIC):
                        # we hide the topics used internally between rcm and firos
                        topics.append({"name": m.group(self.RT_NAME_KEY),
                                       "msg": m.group(self.RT_MSG_KEY),
                                       "type": "publisher"})
            m_sub_list = self.rt_subscriber_extractor_re.finditer(tl_p_out)
            if m_sub_list is not None:
                for m in m_sub_list:
                    if not (m.group(self.RT_NAME_KEY) == self.DEFAULT_OUT_TOPIC
                            or m.group(self.RT_NAME_KEY) == self.DEFAULT_IN_TOPIC):
                        # we hide the topics used internally between rcm and firos
                        topics.append({"name": m.group(self.RT_NAME_KEY),
                                       "msg": m.group(self.RT_MSG_KEY),
                                       "type": "subscriber"})
        result = json.dumps(topics)
        return FIROS_InfoResponse(result)

    def run(self):
        rospy.loginfo("---- rcm driver starting ----")
        # enable publishing Robot_Event messages on DEFAULT_OUT_TOPIC (/rcm/robot_event)
        self.rcm_re_publisher = rospy.Publisher(self.DEFAULT_OUT_TOPIC, Robot_Event,
                                                queue_size=self.DEFAULT_QUEUE_SIZE)
        # enable answering requests of FIROS_Info as service provider of DEFAULT_SERVICE (/rcm/firos_info)
        self.rcm_fi_service = rospy.Service(self.DEFAULT_SERVICE, FIROS_Info, self.firos_info_handler)
        # enable listening to DEFAULT_IN_TOPIC (/firos/cb_event) CB_Event messages
        rospy.Subscriber(self.DEFAULT_IN_TOPIC, CB_Event, callback=self.cb_event_handler)
        # 10hz
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            # we poll with a timeout of 0 millisecond so that we use the rate.sleep()
            # at the end to scan the time
            events = self.po.poll(0)
            for fd, flag in events:
                # retrieve the actual socket from its file descriptor
                s = self.fd_to_socket[fd]
                if flag & (select.POLLIN | select.POLLPRI):
                    # handle inputs
                    if s is self.rcm_driver_s_server:
                        # input for server means that someone is asking for connection
                        self.connection, self.client_address = s.accept()
                        rospy.loginfo("new connection from %s" % str(self.client_address))
                        self.connection.setblocking(0)
                        self.fd_to_socket[self.connection.fileno()] = self.connection
                        self.po.register(self.connection, self.READ_ONLY)
                    else:
                        # input for connection means that the client is sending something
                        # to the server
                        data = s.recv(self._buf_size)
                        if data:
                            # a readable client socket has data
                            rospy.loginfo("received '%s' from %s" % (data, s.getpeername()))
                            # try to decode
                            import json
                            try:
                                request = json.loads(data)
                            except Exception as e:
                                # if an error occurs means that the message is wrong (or is splitted in more
                                # the one part)
                                rospy.logerr("received non-json msg from rcm platform node: %s" % e)
                            else:
                                # we could decode the message
                                if request[self.COMMAND_KEY] == self.NOTIFY_CONNECTION \
                                        or request[self.COMMAND_KEY] == self.NOTIFY_DISCONNECTION:
                                    # this is the case of requests received about the robot status
                                    topic_msg = Robot_Event()
                                    topic_msg.instance_name = self.robot_name = request[self.R_NAME_KEY]
                                    if request[self.COMMAND_KEY] == self.NOTIFY_CONNECTION:
                                        topic_msg.instance_status = topic_msg.CONNECTED
                                        rospy.loginfo("publishing the connection of %s" % topic_msg.instance_name)
                                        # only when we receive the robot connection we could need output channel
                                        # so we add output channel for sending
                                        self.po.modify(s, self.READ_WRITE)
                                    elif request[self.COMMAND_KEY] == self.NOTIFY_DISCONNECTION:
                                        topic_msg.instance_status = topic_msg.DISCONNECTED
                                        rospy.loginfo("publishing the disconnection of %s" % topic_msg.instance_name)
                                    self.rcm_re_publisher.publish(topic_msg)
                elif flag & select.POLLHUP:
                    # handle client hung up
                    rospy.loginfo("closing %s after receiving HUP" % self.client_address)
                    self.po.unregister(s)
                    try:
                        s.shutdown()
                    except:
                        pass
                    finally:
                        s.close()
                elif flag & select.POLLOUT:
                    # handle output
                    # rcm driver can send only the entity status
                    try:
                        es_response = self._entity_status.get_nowait()
                    except Queue.Empty:
                        # no messages of entity status
                        pass
                    else:
                        rospy.loginfo("sending '%s' to %s" % (es_response, s.getpeername()))
                        s.sendall(es_response)
                elif flag & select.POLLERR:
                    # handle error
                    rospy.loginfo("handling exceptional condition for %s" % s.getpeername())
                    self.po.unregister(s)
                    try:
                        s.shutdown()
                    except:
                        pass
                    finally:
                        s.close()
            rate.sleep()
        rospy.loginfo("---- rcm driver ended ----")

if __name__ == '__main__':
    try:
        rcm_d = RCMDriver()
        rcm_d.run()
    except rospy.ROSInterruptException:
        pass
