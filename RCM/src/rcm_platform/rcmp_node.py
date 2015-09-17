__author__ = "Fabio Giuseppe Di Benedetto"

import logging
import os.path
import socket
import signal
import threading
import netifaces

platform_folder = "/opt/rcm-platform"
log_folder_name = "log"
log_folder = os.path.join(platform_folder, log_folder_name)

RCMP_LOGGER_NAME = "RCMPlatformNodeLog"


class RCMPlatformNode:
    # tun0 is the default network interface used: we suppose to run on a machine that is
    # a client in a VPN created with OpenVPN
    NI_TUN = "tun0"
    # NI_ETH = "eth0"
    # this is specific for machine that has the main network interface in eth1
    NI_ETH = "eth1"
    NI_ADDR_KEY = "addr"
    # we have only 2 types of RCMPlatformNodes: S stands for Server and is the old
    # "master" and "vm" (we don't need the distinction between the two); R stands
    # for Robot and is the old "robot"
    S_TYPE = "S"
    R_TYPE = "R"
    # loads for master (fixed) and for other servers (starting point)
    S_M_LOAD = 0
    S_LOAD = 100
    DEFAULT_MASTER_NAME = "master"
    DEFAULT_INBOUND_OPEN_PORTS = "10100, 10101, 10102, 10103"
    PNI_NAME = "pni_name"
    PNI_ADDRESS = "pni_address"
    PNI_TYPE = "pni_type"
    PNI_IS_MASTER = "pni_is_master"
    PNI_MASTER_NODE_IP = "pni_master_node_ip"
    PNI_LOAD = "pni_load"
    PNI_PAIRED_NODE_NAME = "pni_paired_node_name"
    PNI_PAIRED_NODE_IP = "pni_paired_node_ip"
    PNI_STATE = "pni_state"
    PNI_INBOUND_OPEN_PORTS = "pni_inbound_open_ports"
    PNI_RCM_DRIVER_PORT = "pni_rcm_driver_port"
    ICS_SERVICE = "ics"
    EC_SERVICE = "ec"
    # phases
    ST_PHASE = "STARTING"
    CONF_PHASE = "CONFIGURATION"
    ST_S_PHASE = "START SERVICES"
    RESET_PHASE = "RESET"
    # PING_PHASE = "PING"
    PROV_PHASE = "PROVISIONING"
    DISC_PHASE = "DISCOVERY"
    PAIR_PHASE = "PAIRING"
    IDLE_PHASE = "IDLE"
    REBOOT_PHASE = "REBOOTING"
    END_PHASE = "ENDING"
    # these states are used only by not servers
    CONNECTED_STATE = "CONNECTED"
    WAITING_PAIRED_STATE = "WAITING PAIRED"
    PAIRED_STATE = "PAIRED"
    DISCONNECTED_STATE = "DISCONNECTED"

    def __init__(self):
        # we use the application common instance of the logger
        self._logger = logging.getLogger(RCMP_LOGGER_NAME)
        self.pni = {self.PNI_NAME: self.DEFAULT_MASTER_NAME, self.PNI_ADDRESS: None,
                    self.PNI_IS_MASTER: True, self.PNI_MASTER_NODE_IP: None,
                    self.PNI_TYPE: self.S_TYPE, self.PNI_LOAD: None,
                    self.PNI_PAIRED_NODE_NAME: None, self.PNI_PAIRED_NODE_IP: None,
                    self.PNI_STATE: self.DISCONNECTED_STATE, self.PNI_INBOUND_OPEN_PORTS: None,
                    self.PNI_RCM_DRIVER_PORT: None}
        self.pni_port = 9999
        self.ext_ip_address = None
        self.ext_port = 80
        self.ec_p = None
        self.rcm_services = None
        self.ics_service_runner = None
        self.dispatcher = None
        self.current_phase = self.ST_PHASE
        self.error_event = threading.Event()
        self.shutdown_event = threading.Event()
        self.listen_timeout = None

    def run(self):
        self._logger.info("---- rcmpd starting ----")
        # TODO at boot time we have to clean up the tables if we are the master (the other
        # nodes rely on watchdog to clean their internal memory ws and don't have robotics
        # data inside). In case of master he doesn't know that is disconnected because he
        # ping the master to know if he is disconnected when see that the other nodes are
        # disconnecting: we have to solve this problem and decide how to manage the reboot;
        # in reboot if he doesn't cleanup the table before has all the running service spaces
        # on the master already and still in the table, but this service space are no more
        # in the internal memory because of reboot
        # the first time we launch the platform node instance we wait for 0 seconds before trying to enter
        # the main loop
        timeout = 0
        count = 0
        while not self.shutdown_event.isSet():
            shutdown_event_is_set = self.shutdown_event.wait(timeout)
            if not shutdown_event_is_set:
                # we are not shutting down so we have to do the operations
                try:
                    if self.is_vpn_active():
                        # we are online in the VPN
                        # CONFIGURATION phase
                        self.configure(self.CONF_PHASE)
                        # START SERVICES phase
                        self.start_services(self.ST_S_PHASE)
                        # at this moment we are sure that we are starting, rebooting or restarting
                        # and the needed servers are up and running
                        if not self.pni[self.PNI_IS_MASTER]:
                            # we could be on virtual machines or robots
                            # vm and robots have to wait 30 seconds in the IDLE PHASE before doing their
                            # operations
                            self.listen_timeout = 30
                            # PROVISIONING phase
                            self.execute_phase(self.PROV_PHASE, self.provisioning)
                            self.pni[self.PNI_STATE] = self.CONNECTED_STATE
                            # at this point the provisioning succeeded
                            if self.pni[self.PNI_TYPE] != self.S_TYPE:
                                # we are on robots
                                # DISCOVERY phase
                                self.execute_phase(self.DISC_PHASE, self.discovery)
                                # at this point the discovery succeeded
                                # PAIRING phase
                                self.execute_phase(self.PAIR_PHASE, self.pairing)
                                # at this point the pairing succeeded
                            self.pni[self.PNI_STATE] = self.WAITING_PAIRED_STATE
                        else:
                            # we are on the master
                            # the master has to wait 60 minutes in the IDLE PHASE before doing its operations
                            self.listen_timeout = 60*60
                            self.pni[self.PNI_STATE] = self.CONNECTED_STATE
                        signal.signal(signal.SIGTERM, self.signal_handler)
                        self._logger.info("Local RCM platform node '%s' up and running" % self.pni[self.PNI_NAME])
                        self.listen(self.IDLE_PHASE)
                        self._logger.info("Local RCM platform node '%s' rebooting" % self.pni[self.PNI_NAME])
                        # if we exit the listen some error happened; for error we mean that the watchdog
                        # found that this node is offline and set the error_event to restart the daemon:
                        # to do that we need to stop the services otherwise the internal server port will
                        # be already used and we clear the event so that the error event is no more set
                        # and when the daemon come back in the listen can stay in the listen loop if no
                        # other error happens
                        self.error_event.clear()
                        # REBOOT phase
                        self.stop_services(self.REBOOT_PHASE)
                    else:
                        self._logger.info("Local RCM platform node is offline from VPN")
                except Exception as ex:
                    self._logger.info("Local RCM platform node error in %s phase: %s" % (self.current_phase, ex))
                    self.stop_services(self.REBOOT_PHASE)
                # the following time we run the main loop the platform node instance waits more than the initial
                # 0 seconds: an error or a disconnection occurred to force the main loop to be here
                count += 1
                self._logger.debug("Main loop repeat: %d" % count)
                timeout = 5
        self._logger.info("---- rcmpd ended ----")

    def execute_phase(self, phase, operation, args=(), max_retry=4, to_increment=1):
        self.current_phase = phase
        self._logger.info("Executing %s phase" % self.current_phase)
        try:
            try_execution(operation, args=args, max_retry=max_retry, to_increment=to_increment)
        except Exception as e:
            raise Exception("Phase %s failed (%s): restarting from the first phase" % (self.current_phase, e))

    def listen(self, phase):
        self.current_phase = phase
        self._logger.info("Executing %s phase" % self.current_phase)
        while not self.error_event.isSet():
            error_is_set = self.error_event.wait(self.listen_timeout)
            if not error_is_set:
                if self.pni[self.PNI_IS_MASTER]:
                    # only the master has to ping the nodes in robotics data
                    self.ping_platform_nodes()
                else:
                    # in case of vm and robot that are not paired we need a trick to know
                    # if they are online in the platform
                    if self.pni[self.PNI_STATE] == self.WAITING_PAIRED_STATE:
                        self._logger.info("-- TMP -- state: %s" % self.pni[self.PNI_STATE])
                        try:
                            try_execution(self.ping_p_node, args=(self.pni[self.PNI_MASTER_NODE_IP], ),
                                          max_retry=2, to_increment=1)
                        except Exception as e:
                            # we enter here only when try_execution() failed 2 times the self.ping_p_node
                            # which means that the ping failed and we have to clean up robotics data
                            self._logger.debug("Platform node instance '%s' is offline: %s" %
                                               (self.pni[self.PNI_NAME], e))
                            # we go out from the IDLE PHASE
                            self.error_event.set()

    def is_vpn_active(self):
        """Return if the VPN is active or not."""
        is_active = False
        if self.NI_TUN in netifaces.interfaces():
            ni_addresses = netifaces.ifaddresses(self.NI_TUN)[netifaces.AF_INET]
            if ni_addresses:
                # we suppose that we have only one address for this network interface
                # and we take the address of the first and only one in the list
                self.pni[self.PNI_ADDRESS] = ni_addresses[0][self.NI_ADDR_KEY]
            if self.pni[self.PNI_ADDRESS]:
                is_active = True
        return is_active

    def configure(self, phase):
        """Configure the platform node instance."""
        import ConfigParser
        self.current_phase = phase
        self._logger.info("Executing %s phase" % self.current_phase)
        self.rcm_services = {}
        cfg_file_path = os.path.join(platform_folder, "init.cfg")
        if os.path.exists(cfg_file_path):
            # if there is a configuration file means that we are not in the case of master
            # read the configuration file for the initial settings of the node
            cfg_parser = ConfigParser.ConfigParser()
            cfg_parser.read(cfg_file_path)
            if cfg_parser.has_option('main', 'name'):
                self.pni[self.PNI_NAME] = cfg_parser.get('main', 'name')
            if not self.pni[self.PNI_NAME]:
                # if the name is not provided we use the hostname
                self.pni[self.PNI_NAME] = socket.gethostname()
            self._logger.info("Local RCM platform node is named '%s'" % self.pni[self.PNI_NAME])
            if cfg_parser.has_option('main', 'inbound_open_ports'):
                    iop = cfg_parser.get('main', 'inbound_open_ports')
                    self.pni[self.PNI_INBOUND_OPEN_PORTS] = ",".join([i.strip() for i in iop.split(",")])
                    self._logger.info("Local RCM platform node has inbound_open_ports: %s" %
                                      self.pni[self.PNI_INBOUND_OPEN_PORTS])
            if cfg_parser.has_option('main', 'robot'):
                if cfg_parser.getboolean('main', 'robot'):
                    self.pni[self.PNI_TYPE] = self.R_TYPE
                else:
                    self.pni[self.PNI_TYPE] = self.S_TYPE
                    self.pni[self.PNI_LOAD] = self.S_LOAD
            self._logger.info("Local RCM platform node has type %s" % self.pni[self.PNI_TYPE])
            # if the ip_master is not provided we try to find it; we suppose that the machine
            # which is the VPN server is the master too
            if cfg_parser.has_option('main', 'ip_master'):
                self.pni[self.PNI_MASTER_NODE_IP] = cfg_parser.get('main', 'ip_master')
            if self.pni[self.PNI_MASTER_NODE_IP]:
                self.pni[self.PNI_MASTER_NODE_IP] = self.pni[self.PNI_MASTER_NODE_IP].strip()
            else:
                tmp = self.pni[self.PNI_ADDRESS].split('.')
                tmp[-1] = "1"
                self.pni[self.PNI_MASTER_NODE_IP] = '.'.join(tmp)
            self._logger.info("Master RCM platform node has address %s" % self.pni[self.PNI_MASTER_NODE_IP])
            self.pni[self.PNI_IS_MASTER] = False
        else:
            # on the master we don't need an init file but we have robotics data
            # and the external connector: for the inbound open ports we use a lite version
            # of the configuration file, but hidden and with only that information
            # if available
            cfg_file_path = os.path.join(platform_folder, ".init.cfg")
            if os.path.exists(cfg_file_path):
                cfg_parser = ConfigParser.ConfigParser()
                cfg_parser.read(cfg_file_path)
                if cfg_parser.has_option('main', 'inbound_open_ports'):
                    iop = cfg_parser.get('main', 'inbound_open_ports')
                    self.pni[self.PNI_INBOUND_OPEN_PORTS] = ",".join([i.strip() for i in iop.split(",")])
                    self._logger.info("Master RCM platform node has inbound_open_ports: %s" %
                                      self.pni[self.PNI_INBOUND_OPEN_PORTS])
            self._logger.info("Master RCM platform node is named '%s'" % self.pni[self.PNI_NAME])
            self.pni[self.PNI_LOAD] = self.S_M_LOAD
            if not self.pni[self.PNI_INBOUND_OPEN_PORTS]:
                # the lite configuration file for the master wasn't available so we use the default
                self.pni[self.PNI_INBOUND_OPEN_PORTS] = self.DEFAULT_INBOUND_OPEN_PORTS
                self._logger.info("Master RCM platform node uses default inbound_open_ports: %s" %
                                  self.pni[self.PNI_INBOUND_OPEN_PORTS])
            from rcmp_robotics_data import RCMPRoboticsDataManager
            rdm = RCMPRoboticsDataManager()
            rdm.init_db(self.pni)

    def start_services(self, phase):
        self.current_phase = phase
        self._logger.info("Executing %s phase" % self.current_phase)
        # starting local services
        self.start_service(self.ICS_SERVICE, (self.pni[self.PNI_ADDRESS], self.pni_port))
        if self.pni[self.PNI_IS_MASTER]:
            self.ext_ip_address = netifaces.ifaddresses(self.NI_ETH)[netifaces.AF_INET][0][self.NI_ADDR_KEY]
            if not self.ext_ip_address:
                raise IOError("No ip address available in network interface %s for external connector" %
                              self.NI_ETH)
            self.start_service(self.EC_SERVICE, (self.ext_ip_address, self.ext_port))

    def start_service(self, service, service_address=None):
        if service == self.ICS_SERVICE:
            from rcmp_inter_communication import RCMPInterCommunicationServer, RCMPPacketHandler
            self._logger.info("Running RCM platform node '%s' on %s:%d" % (self.pni[self.PNI_NAME], service_address[0],
                              service_address[1]))
            from rcmp_inter_communication import RCMPDispatcher
            from rcmp_ms import MServiceManager
            from rcmp_wd import WDogManager
            # pni keeps information from startup configuration
            msm = MServiceManager()
            self.dispatcher = RCMPDispatcher(self.pni, msm, WDogManager(self.pni, msm, self.error_event))
            self.rcm_services[self.ICS_SERVICE] = RCMPInterCommunicationServer(service_address, RCMPPacketHandler,
                                                                               self.dispatcher)
            self.ics_service_runner = threading.Thread(name=self.ICS_SERVICE,
                                                       target=self.rcm_services[self.ICS_SERVICE].serve_forever)
            # the parent process can't die until the thread non_daemon dies
            self.ics_service_runner.start()
            self._logger.info("Internal communication server started")
        elif service == self.EC_SERVICE:
            # the import of this object must be in run() or after because it imports
            # twisted that pre-opens some file descriptors and socket: if you import
            # here the open of the ContextDaemon of python-daemon is already done and
            # that component doesn't close the descriptors pre-opened by twisted
            from rcmp_ext_connector import RCMPExtConnector
            import multiprocessing
            self._logger.info("Running external connector on %s:%d" % (service_address[0], service_address[1]))
            ec = RCMPExtConnector()
            self.rcm_services[self.EC_SERVICE] = multiprocessing.Process(name=self.EC_SERVICE,
                                                                         target=ec.start, args=service_address)
            self._logger.info("External connector started")
            # the parent process can't die until the subprocess dies
            self.rcm_services[self.EC_SERVICE].start()

    def kill_all_local_service_items(self, success_event):
        """Kill all the local service items (service spaces, service launchers and service nodes)."""
        # before stopping the servers we need to stop all watchdogs doing
        # their work (the ping to check the connection)
        self.throw_stop_all_watchdogs_event()
        if not self.pni[self.PNI_IS_MASTER]:
            from rcmp_command import DELETE_ALL_L_SERVICE_SPACE, RCMPCommandHandler
            from rcmp_platform_command import PNodeInstance
            msg_dict = {RCMPCommandHandler.COMMAND_KEY: DELETE_ALL_L_SERVICE_SPACE,
                        PNodeInstance.I_ADDRESS_KEY: self.pni[RCMPlatformNode.PNI_ADDRESS]}
        else:
            # this platform node is the master and can perform the rollback_update
            from rcmp_event import ROLLBACK_UPDATE_ALL_EVENT, RCMPEventHandler, PNodeInstance
            msg_dict = {RCMPEventHandler.EVENT_KEY: ROLLBACK_UPDATE_ALL_EVENT,
                        PNodeInstance.I_ADDRESS_KEY: self.pni[self.PNI_ADDRESS]}
        try:
            result = self.dispatcher.dispatch(msg_dict)
            if result.is_ok():
                success_event.set()
            else:
                reason = "Unable to kill the local service items in the RCM platform node '%s': %s" % \
                         (self.pni[self.PNI_NAME], result.get_response_reason())
                self._logger.error(reason)
        except Exception as e:
            reason = "Unable to kill the local service items in the RCM platform node '%s': %s" % \
                     (self.pni[self.PNI_NAME], e)
            raise Exception(reason)

    def provisioning(self, success_event):
        """Inform the master platform node that this platform node is available in
        the platform."""
        from rcmp_command import AUTO_PROVISIONING_PNODE_INSTANCE, RCMPCommandHandler
        from rcmp_platform_command import PNodeInstance
        msg_dict = {RCMPCommandHandler.COMMAND_KEY: AUTO_PROVISIONING_PNODE_INSTANCE,
                    PNodeInstance.I_ADDRESS_KEY: self.pni[self.PNI_MASTER_NODE_IP],
                    PNodeInstance.PI_NAME_KEY: self.pni[self.PNI_NAME],
                    PNodeInstance.PI_ADDRESS_KEY: self.pni[self.PNI_ADDRESS],
                    PNodeInstance.PI_TYPE_KEY: self.pni[self.PNI_TYPE]}
        try:
            result = self.dispatcher.dispatch(msg_dict)
            if result.is_ok():
                success_event.set()
            else:
                reason = "Unable to notify '%s' existence to the master RCM platform node at %s: %s" % \
                         (self.pni[self.PNI_NAME], self.pni[self.PNI_MASTER_NODE_IP],
                          result.get_response_reason())
                self._logger.error(reason)
        except Exception as e:
            reason = "Unable to notify '%s' existence to the master RCM platform node at %s: %s" % \
                     (self.pni[self.PNI_NAME], self.pni[self.PNI_MASTER_NODE_IP], e)
            raise Exception(reason)

    def signal_handler(self, num, stack):
        self._logger.debug("Signal handler called")
        self.stop_services(self.END_PHASE)
        self._logger.debug("Services stopped")
        self.shutdown_event.set()
        self._logger.debug("Exiting from the main loop")

    def stop_services(self, phase):
        """Stop all the services of the platform instance."""
        self.current_phase = phase
        self._logger.info("Executing %s phase" % self.current_phase)
        if self.ics_service_runner and self.ics_service_runner.is_alive():
            self.execute_phase(self.RESET_PHASE, self.kill_all_local_service_items)
        for rcm_service in self.rcm_services.keys():
            self.stop_service(rcm_service)
        self.pni[self.PNI_STATE] = self.DISCONNECTED_STATE

    def stop_service(self, service):
        """Stop a service of the platform node instance."""
        if service == self.ICS_SERVICE:
            self.rcm_services[self.ICS_SERVICE].shutdown()
            self.ics_service_runner.join()
            self.ics_service_runner = None
            self.dispatcher = None
        elif service == self.EC_SERVICE:
            if self.rcm_services[self.EC_SERVICE] and self.rcm_services[self.EC_SERVICE].exitcode is None:
                self.rcm_services[self.EC_SERVICE].terminate()

    def discovery(self, success_event):
        """Discover the platform instance node of type S that is provided to pairing to. The
        platform instance node of type R throwing this message to the master node, receives
        the name and address of the other end with which it has to deal."""
        from rcmp_command import DISCOVERY_PNODE_INSTANCE, RCMPCommandHandler
        from rcmp_platform_command import PNodeInstance
        # as simple platform node we have to contact the master to notify our existence
        msg_dict = {RCMPCommandHandler.COMMAND_KEY: DISCOVERY_PNODE_INSTANCE,
                    PNodeInstance.I_ADDRESS_KEY: self.pni[self.PNI_MASTER_NODE_IP]}
        try:
            result = self.dispatcher.dispatch(msg_dict)
            if result.is_ok():
                reason = result.get_response_reason()
                self.pni[self.PNI_PAIRED_NODE_NAME] = reason[PNodeInstance.PI_NAME_KEY]
                self.pni[self.PNI_PAIRED_NODE_IP] = reason[PNodeInstance.PI_ADDRESS_KEY]
                success_event.set()
            else:
                reason = "Unable to discover a server: %s" % result.get_response_reason()
                self._logger.error(reason)
        except Exception as e:
            reason = "Unable to discover a server: %s" % e
            raise Exception(reason)

    def pairing(self, success_event):
        """Contact the the platform instance node of type S to pair with. The
        platform instance node of type R after receiving what node is assigned
        to it, throws this message to pair with and providing name and ip address."""
        from rcmp_event import PAIRING_EVENT, RCMPEventHandler, PNodeInstance
        # the platform node has discovered the platform node with which it has to be paired
        # so it has to notify itself to that node
        msg_dict = {RCMPEventHandler.EVENT_KEY: PAIRING_EVENT,
                    PNodeInstance.I_ADDRESS_KEY: self.pni[self.PNI_PAIRED_NODE_IP],
                    PNodeInstance.PI_R_NAME_KEY: self.pni[self.PNI_NAME],
                    PNodeInstance.PI_R_ADDRESS_KEY: self.pni[self.PNI_ADDRESS]}
        try:
            result = self.dispatcher.dispatch(msg_dict)
            if result.is_ok():
                success_event.set()
            else:
                reason = "Unable to pairing with '%s' at %s: %s" % \
                         (self.pni[self.PNI_PAIRED_NODE_NAME], self.pni[self.PNI_PAIRED_NODE_IP],
                          result.get_response_reason())
                self._logger.error(reason)
        except Exception as e:
            reason = "Unable to pairing with '%s' at %s: %s" % \
                     (self.pni[self.PNI_PAIRED_NODE_NAME], self.pni[self.PNI_PAIRED_NODE_IP], e)
            raise Exception(reason)

    def ping_platform_nodes(self):
        """Ping all the platform node instances available on the platform. The platform node
        instances not provisioned in the platform are not considered available."""
        from rcmp_command import READ_PNODE_INSTANCE_LIST, RCMPCommandHandler
        from rcmp_platform_command import PNodeInstance
        pi_list = None
        msg_dict = {RCMPCommandHandler.COMMAND_KEY: READ_PNODE_INSTANCE_LIST,
                    PNodeInstance.I_ADDRESS_KEY: self.pni[self.PNI_ADDRESS],
                    PNodeInstance.PI_EXTENDED_KEY: True}
        result = self.dispatcher.dispatch(msg_dict)
        if result.is_ok():
            pi_list = result.get_response_reason()
        else:
            reason = "Unable to retrieve the RCM platform nodes: %s" % result.get_response_reason()
            self._logger.error(reason)
        if pi_list:
            available_pni = 0
            reachable_pni = 0
            for pi in pi_list:
                if pi[PNodeInstance.PI_ADDRESS_KEY] and pi[PNodeInstance.PI_ADDRESS_KEY] != self.pni[self.PNI_ADDRESS]:
                    available_pni += 1
                    # we will ping only the platform node instances that are not the master
                    # (the current platform node is the master because we arrive here in
                    # ping_platform_nodes only with the master) or not already provisioned
                    try:
                        try_execution(self.ping_p_node, args=(pi[PNodeInstance.PI_ADDRESS_KEY], ),
                                      max_retry=2, to_increment=1)
                        # if someone pass the try_execution of the ping means that some platform node is reachable
                        # so we (as master platform node) are online
                        reachable_pni += 1
                    except Exception as e:
                        # we enter here only when try_execution() failed 2 times the self.ping_p_node
                        # which means that the ping failed and we have to clean up robotics data
                        self._logger.debug("Platform node instance '%s' not more available: %s" %
                                           (pi[PNodeInstance.PI_NAME_KEY], e))
                        # TODO add cases for deleting vm (robot and vm are managed locally)
            if available_pni > 0 and reachable_pni == 0:
                # in case the master discover that he is offline has to clean up himself
                self._logger.debug("No more platform node instances available")
                # self.error_event.set()

    def ping_p_node(self, success_event, pi_address):
        """Ping a platform node instance."""
        from rcmp_command import PING_PNODE_INSTANCE, RCMPCommandHandler
        from rcmp_platform_command import PNodeInstance
        # self._logger.debug("Pinging %s" % pi_address)
        msg_dict = {RCMPCommandHandler.COMMAND_KEY: PING_PNODE_INSTANCE,
                    PNodeInstance.I_ADDRESS_KEY: pi_address}
        result = self.dispatcher.dispatch(msg_dict)
        if result.is_ok():
            success_event.set()

    def throw_stop_all_watchdogs_event(self):
        """Kill all the watchdogs running on the platform node."""
        from rcmp_event import STOP_ALL_WD_EVENT, RCMPEventHandler, PNodeInstance
        event = {RCMPEventHandler.EVENT_KEY: STOP_ALL_WD_EVENT,
                 PNodeInstance.I_ADDRESS_KEY: self.pni[RCMPlatformNode.PNI_ADDRESS]}
        self._logger.info("Throwing stop all watchdog event = %s" % event)
        result = self.dispatcher.dispatch(event)


def try_execution(operation, args=(), max_retry=2, to_increment=2):
    success_event = threading.Event()
    retry = 0
    # first time we wait for 0 seconds before trying to do the operation
    timeout = 0
    while not success_event.isSet():
        # wait at least the amount of the timeout (in seconds)
        success_is_set = success_event.wait(timeout)
        if not success_is_set:
            # try to perform the operation
            operation(success_event, *args)
            if retry < max_retry:
                # we increment the timeout every time we have tried the operation
                # so that the next try is delayed to give more time
                timeout += to_increment
                retry += 1
            else:
                # after max_retry we exit with exception
                raise Exception("Execution failed %s %s" % (retry, "time" if retry < 2 else "times"))
