__author__ = "Fabio Giuseppe Di Benedetto"

import select
import Queue
from rcmp_inter_communication import threading, socket, RCMPMessage, RCMPDispatcher
from rcmp_event import logging, RCMP_LOGGER_NAME, PNodeInstance
from rcmp_node import RCMPlatformNode, try_execution


class WDogManager:

    def __init__(self, pni, msm, error_event):
        self._logger = logging.getLogger(RCMP_LOGGER_NAME)
        # we keep track of the watchdog managed by the platform node instance
        # (only one on the robot side or one for each robot managed by the server
        # on the server side)
        self.wd = {}
        # lock for synchronization around watchdogs
        self.wd_lock = threading.RLock()
        self.pni = pni
        # event tied to the shutdown to stop the watchdogs
        self.pni_error_event = error_event
        self.msm = msm

    def add_watchdog(self, params):
        """Add a watchdog."""
        try:
            self.wd_lock.acquire()
            if params and PNodeInstance.PI_ADDRESS_KEY in params and params[PNodeInstance.PI_ADDRESS_KEY]:
                # we create the watchdog only if there isn't a watchdog monitoring the same address
                if params[PNodeInstance.PI_ADDRESS_KEY] not in self.wd:
                    # we create a dispatcher to use internally in the watchdog
                    dispatcher = RCMPDispatcher(self.pni, self.msm, self)
                    st_e = threading.Event()
                    self.wd[params[PNodeInstance.PI_ADDRESS_KEY]] = \
                        {WatchDog.WD_I_KEY: WatchDog(self.pni, dispatcher, st_e, self.pni_error_event,
                                                     params[PNodeInstance.PI_NAME_KEY]
                                                     if PNodeInstance.PI_NAME_KEY in params
                                                     else None, params[PNodeInstance.PI_ADDRESS_KEY]),
                         WatchDog.WD_STOP_EVENT_KEY: st_e}
            self._logger.debug("------ params ------")
            self._logger.debug(params)
            self._logger.debug("------ wd ------")
            self._logger.debug(self.wd)
            self._logger.debug("---------------")
        finally:
            self.wd_lock.release()

    def get_watchdog(self, params):
        """Get the watchdog monitoring a specified ip address."""
        try:
            self.wd_lock.acquire()
            return self.wd[params[PNodeInstance.PI_ADDRESS_KEY]][WatchDog.WD_I_KEY]
        finally:
            self.wd_lock.release()

    def is_watchdog_available(self, params):
        """Check if the watchdog corresponding to the specified ip address is available."""
        try:
            self.wd_lock.acquire()
            return params[PNodeInstance.PI_SOURCE_ADDRESS_KEY] in self.wd and \
                self.wd[params[PNodeInstance.PI_SOURCE_ADDRESS_KEY]][WatchDog.WD_I_KEY] and \
                self.wd[params[PNodeInstance.PI_SOURCE_ADDRESS_KEY]][WatchDog.WD_I_KEY].is_alive()
        finally:
            self.wd_lock.release()

    def delete_watchdog(self, params):
        """Delete a watchdog."""
        try:
            self.wd_lock.acquire()
            self.wd[params[PNodeInstance.PI_ADDRESS_KEY]][WatchDog.WD_STOP_EVENT_KEY].set()
            # wait the watchdog thread finish his work
            self.wd[params[PNodeInstance.PI_ADDRESS_KEY]][WatchDog.WD_I_KEY].join()
            # delete all what concerning the specified watchdog
            self.wd.pop(params[PNodeInstance.PI_ADDRESS_KEY])
            if not self.pni[RCMPlatformNode.PNI_IS_MASTER]:
                # while we are deleting the watchdogs on vms and robots we have to check the
                # last watchdog available: in this case no one check if the RCMPlatformNode
                # is offline from the platform so we have to put him in WAITING_PAIRED_STATE
                # that restart the ping to the master from the daemon
                if not (self.wd and len(self.wd.keys()) > 0):
                    # TODO the write at this level means that maybe we have to synchronize pni
                    self.pni[RCMPlatformNode.PNI_STATE] = RCMPlatformNode.WAITING_PAIRED_STATE
            self._logger.debug("------ params ------")
            self._logger.debug(params)
            self._logger.debug("------ wd ------")
            self._logger.debug(self.wd)
            self._logger.debug("---------------")
        finally:
            self.wd_lock.release()

    def delete_all_watchdogs(self):
        """Delete all watchdogs."""
        try:
            self.wd_lock.acquire()
            for wd_item in self.wd.keys():
                self.wd[wd_item][WatchDog.WD_STOP_EVENT_KEY].set()
                # wait the watchdog thread finish his work
                self.wd[wd_item][WatchDog.WD_I_KEY].join()
                # delete all what concerning the specified watchdog
                self.wd.pop(wd_item)
        finally:
            self.wd_lock.release()


class WatchDog(threading.Thread):
    WD_I_KEY = "wd_i"
    WD_STOP_EVENT_KEY = "wd_stop_e"

    def __init__(self, pni, dispatcher, stop_event, error_event, paired_node_name=None, paired_node_ip=None):
        threading.Thread.__init__(self)
        self._logger = logging.getLogger(RCMP_LOGGER_NAME)
        self.pni = pni
        self.stop_event = stop_event
        # this is used to notify to the platform node that something went wrong and
        # it has to restart its main daemon loop
        self.error_event = error_event
        self.paired_node_name = paired_node_name
        self.paired_node_ip = paired_node_ip
        self._logger.debug("Watchdog paired_node_name = %s, paired_node_ip = %s" % (paired_node_name, paired_node_ip))
        self.name = "WDog_%s_%s" % (self.pni[RCMPlatformNode.PNI_NAME], paired_node_name)
        self.dispatcher = dispatcher

    def run(self):
        self._logger.info("---- watchdog '%s' starting ----" % self.name)
        # rcm_d_c = None
        rcm_n2d_m = None
        # at this moment the robot is connected so we have to notify that to RCM driver
        if self.pni[RCMPlatformNode.PNI_RCM_DRIVER_PORT]:
            # this is done only in case we have rcm driver on this rcm node instance
            try:
                self._logger.info("watchdog '%s' running with rcm on port %d" %
                                  (self.name, self.pni[RCMPlatformNode.PNI_RCM_DRIVER_PORT]))
                rs_queue = Queue.Queue()
                es_queue = Queue.Queue()
                rcm_n2d_m = self.RCMNode2DriverManager(rs_queue, es_queue, self.pni[RCMPlatformNode.PNI_RCM_DRIVER_PORT])
                rcm_n2d_m.connect()
                threading.Thread(target=rcm_n2d_m.sniff).start()
                result = rcm_n2d_m.notify_robot_status(self.pni[RCMPlatformNode.PNI_NAME]
                                                       if self.pni[RCMPlatformNode.PNI_TYPE] == RCMPlatformNode.R_TYPE
                                                       else self.paired_node_name, True)
                if result is None:
                    self._logger.info("Entity creation status required too much time: don't wait anymore")
            except Exception as e:
                self._logger.error("watchdog error notifying robot connection to rcm driver: %s" % e)
        self._logger.info("'%s' connected to '%s': watchdog monitoring the connection" %
                          (self.pni[RCMPlatformNode.PNI_NAME], self.paired_node_name))
        # the first time we launch the watchdog we wait for 1 minute to give the master the time
        # to perform the service logic boot
        w_timeout = 30
        while not self.stop_event.isSet():
            stop_event_is_set = self.stop_event.wait(w_timeout)
            if not stop_event_is_set:
                # we are not stopping the watchdog so we have to do the operations
                result = RCMPMessage()
                try:
                    try_execution(self.ping_pni, args=(self.paired_node_ip, ), to_increment=0)
                    # the ping to the other peer went well so the connection is still active
                except Exception as e:
                    # the ping to the other peer failed
                    self._logger.debug("Watchdog ping to '%s' failed: %s" % (self.paired_node_name, e))
                    try:
                        if rcm_n2d_m:
                            # this is done only in case we have rcm_d_c because means that we are on the
                            # platform node instance which runs the rcm driver
                            result = rcm_n2d_m.notify_robot_status(self.pni[RCMPlatformNode.PNI_NAME]
                                                                   if self.pni[RCMPlatformNode.PNI_TYPE] ==
                                                                   RCMPlatformNode.R_TYPE else self.paired_node_name,
                                                                   True)
                            if result is None:
                                self._logger.info("Entity removal status required too much time: don't wait anymore")
                    except Exception as e:
                        self._logger.error("watchdog error notifying robot disconnection to rcm driver: %s" % e)
                    self._logger.info("'%s' disconnected from '%s'" %
                                      (self.pni[RCMPlatformNode.PNI_NAME], self.paired_node_name))
                    if self.pni[RCMPlatformNode.PNI_IS_MASTER]:
                        # we are on the master so we cannot be offline from the platform: these means that
                        # the other peer is offline
                        self.remote_paired_node_offline()
                    elif self.pni[RCMPlatformNode.PNI_MASTER_NODE_IP] == self.paired_node_ip:
                        # the other peer is the master that cannot be offline from the platform, so we are
                        # us that we are offline
                        self.local_paired_node_offline()
                    else:
                        # the master platform node is not involved between paired so to verify who is
                        # offline we try to ping the master
                        try:
                            try_execution(self.ping_pni, args=(self.pni[RCMPlatformNode.PNI_MASTER_NODE_IP], ),
                                          to_increment=0)
                            # the ping to the master went well so the other peer is offline
                            self.remote_paired_node_offline()
                        except Exception as e:
                            # the ping to the master failed so we are us offline
                            result.create_error_response("Watchdog error: %s" % e)
                            self.local_paired_node_offline()
                w_timeout = 10
        self._logger.debug("---- watchdog '%s' ended ----" % self.name)

    # the same as rcmp_n.py ping_p_node()
    def ping_pni(self, success_event, target_pni):
        """Ping the paired or the master platform node instance."""
        from rcmp_command import PING_PNODE_INSTANCE, RCMPCommandHandler
        from rcmp_platform_command import PNodeInstance
        self._logger.debug("Watchdog pinging %s with source %s" % (target_pni, self.pni[RCMPlatformNode.PNI_ADDRESS]))
        msg_dict = {RCMPCommandHandler.COMMAND_KEY: PING_PNODE_INSTANCE,
                    PNodeInstance.I_ADDRESS_KEY: target_pni,
                    PNodeInstance.PI_SOURCE_ADDRESS_KEY: self.pni[RCMPlatformNode.PNI_ADDRESS]}
        result = self.dispatcher.dispatch(msg_dict)
        if result.is_ok():
            success_event.set()

    def remote_paired_node_offline(self):
        self._logger.info("'%s' disconnected from the platform" % self.paired_node_name)
        # this platform node instance can reach the master so it is not disconnected
        # (or is the master)
        self.throw_rollback_update_event()
        # we have to notify locally that this watchdog has to be deleted
        self.throw_wd_stop_event()

    def local_paired_node_offline(self):
        self._logger.info("Local platform node '%s' disconnected from the platform" %
                          self.pni[RCMPlatformNode.PNI_NAME])
        self.pni[RCMPlatformNode.PNI_STATE] = RCMPlatformNode.DISCONNECTED_STATE
        # this event throw the restart of the daemon: when restarting the daemon
        # we kill all the local services and all the watchdogs on the node
        self.error_event.set()

    def throw_rollback_update_event(self):
        from rcmp_event import ROLLBACK_UPDATE_EVENT, RCMPEventHandler, PNodeInstance
        event = {RCMPEventHandler.EVENT_KEY: ROLLBACK_UPDATE_EVENT,
                 PNodeInstance.I_ADDRESS_KEY: self.pni[RCMPlatformNode.PNI_MASTER_NODE_IP]
                 if not self.pni[RCMPlatformNode.PNI_IS_MASTER] else self.pni[RCMPlatformNode.PNI_ADDRESS],
                 PNodeInstance.PI_REACHABLE_KEY: self.pni[RCMPlatformNode.PNI_TYPE]}
        if self.pni[RCMPlatformNode.PNI_TYPE] == RCMPlatformNode.R_TYPE:
            event[PNodeInstance.PI_S_NAME_KEY] = self.paired_node_name
            event[PNodeInstance.PI_S_ADDRESS_KEY] = self.paired_node_ip
            event[PNodeInstance.PI_R_NAME_KEY] = self.pni[RCMPlatformNode.PNI_NAME]
            event[PNodeInstance.PI_R_ADDRESS_KEY] = self.pni[RCMPlatformNode.PNI_ADDRESS]
        else:
            event[PNodeInstance.PI_S_NAME_KEY] = self.pni[RCMPlatformNode.PNI_NAME]
            event[PNodeInstance.PI_S_ADDRESS_KEY] = self.pni[RCMPlatformNode.PNI_ADDRESS]
            event[PNodeInstance.PI_R_NAME_KEY] = self.paired_node_name
            event[PNodeInstance.PI_R_ADDRESS_KEY] = self.paired_node_ip
        self._logger.info("Watchdog throwing rollback update event = %s" % event)
        self.dispatcher.dispatch(event)

    def throw_wd_stop_event(self):
        from rcmp_event import STOP_WD_EVENT, RCMPEventHandler, PNodeInstance
        event = {RCMPEventHandler.EVENT_KEY: STOP_WD_EVENT,
                 PNodeInstance.I_ADDRESS_KEY: self.pni[RCMPlatformNode.PNI_ADDRESS],
                 PNodeInstance.PI_ADDRESS_KEY: self.paired_node_ip}
        self._logger.info("Watchdog throwing stop wd event = %s" % event)
        self.dispatcher.dispatch(event)

    class RCMNode2DriverManager:

        # we are using 127.0.0.1 for the communication between rcm platform node and
        # rcm driver because is a local communication (there is always a platform node
        # in the same machine as the one where will be the rcm driver)
        DEFAULT_DRIVER_ADDRESS = socket.gethostbyname("localhost")
        DEFAULT_DRIVER_PORT = 9998

        # these are constants duplicated from RCMDriver and used to deal with that
        # component
        COMMAND_KEY = "rcm_d_cmd"
        NOTIFY_CONNECTION = "r_connection"
        NOTIFY_DISCONNECTION = "r_disconnection"
        R_NAME_KEY = "r_name"
        ENTITY_CREATED = "e_created"
        ENTITY_REMOVED = "e_removed"

        # commonly used flags
        READ_ONLY = select.POLLIN | select.POLLPRI | select.POLLHUP | select.POLLERR
        READ_WRITE = READ_ONLY | select.POLLOUT

        def __init__(self, rs_queue, es_queue, port=DEFAULT_DRIVER_PORT):
            self._connection = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self._driver_address = (self.DEFAULT_DRIVER_ADDRESS, port)
            self._buf_size = 1024
            # in milliseconds
            self._timeout = 5000
            # we get a polling object
            self._po = select.poll()
            # register the server on read only events
            self._po.register(self._connection, self.READ_WRITE)
            # map file descriptors to socket objects
            self._fd_to_socket = {self._connection.fileno(): self._connection}
            self._rs_queue = rs_queue
            self._es_queue = es_queue
            self._logger = logging.getLogger(RCMP_LOGGER_NAME)

        def connect(self):
            """Connect RCM platform node to RCM driver."""
            self._connection.connect(self._driver_address)

        def notify_robot_status(self, robot_name, connected):
            """Notify to RCM driver the status of the robot and wait until the driver answers."""
            import json
            if connected:
                # when we notify the connection of a robot means that we haven't
                # connected this client with the driver (and so the server)
                msg_dict = {self.COMMAND_KEY: self.NOTIFY_CONNECTION, self.R_NAME_KEY: robot_name}
            else:
                msg_dict = {self.COMMAND_KEY: self.NOTIFY_DISCONNECTION, self.R_NAME_KEY: robot_name}
            request = json.dumps(msg_dict)
            self._rs_queue.put_nowait(request)
            # we have to block until we receive the entity status or too much time has passed
            try:
                rs_notification = self._es_queue.get(timeout=20)
            except Queue.Empty:
                # no robot status notification
                return None
            else:
                return rs_notification

        def sniff(self):
            finished = False
            while not finished:
                # we poll with a timeout of 0 millisecond so that we use the rate.sleep()
                # at the end to scan the time
                events = self._po.poll(self._timeout)
                for fd, flag in events:
                    # retrieve the actual socket from its file descriptor
                    s = self._fd_to_socket[fd]
                    if flag & (select.POLLIN | select.POLLPRI):
                        # handle inputs
                        # input for connection means that the server is sending something
                        # to the client
                        data = s.recv(self._buf_size)
                        if data:
                            # a readable client socket has data
                            # try to decode
                            import json
                            try:
                                request = json.loads(data)
                            except Exception as e:
                                # if an error occurs means that the message is wrong (or is splitted in more
                                # the one part)
                                self._logger.error("received non-json msg from rcm platform node: %s" % e)
                            else:
                                # we could decode the message
                                if request[self.COMMAND_KEY] == self.ENTITY_CREATED \
                                        or request[self.COMMAND_KEY] == self.ENTITY_REMOVED:
                                    # this is the case of response about entity status
                                    self._es_queue.put_nowait(data)
                                    if request[self.COMMAND_KEY] == self.ENTITY_REMOVED:
                                        # once we receive the entity removed message we can exit from this loop
                                        finished = True
                    elif flag & select.POLLHUP:
                        # handle client hung up
                        self._logger.debug("closing %s:%d after receiving HUP" % self._driver_address)
                        # Stop listening for input on the connection
                        self._po.unregister(s)
                        try:
                            s.shutdown()
                        except:
                            pass
                        finally:
                            s.close()
                        # reconnect
                        self.connect()
                    elif flag & select.POLLOUT:
                        # handle output
                        # rcm driver can send only 2 types of messages: the entity status or a request
                        # of robot information, so we look at both the queues
                        try:
                            rs_notification = self._rs_queue.get_nowait()
                        except Queue.Empty:
                            # no robot status notification
                            pass
                        else:
                            self._logger.debug("sending '%s' to %s" % (rs_notification, s.getpeername()))
                            s.sendall(rs_notification)
                    elif flag & select.POLLERR:
                        # handle error
                        self._logger.debug("handling exceptional condition for %s" % s.getpeername())
                        # Stop listening for input on the connection
                        self._po.unregister(s)
                        try:
                            s.shutdown()
                        except:
                            pass
                        finally:
                            s.close()
                        # reconnect
                        self.connect()
