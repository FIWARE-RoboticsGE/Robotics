__author__ = "Fabio Giuseppe Di Benedetto"

import os

from rcmp_command import RCMPlatformNode, RCMPCommandAgent, RCMPMessage
from rcmp_service_command import ServiceSpace


class PNodeInstance(RCMPCommandAgent):
    M_ADDRESS_KEY = "m_address"
    PI_NAME_KEY = "pi_name"
    PI_ADDRESS_KEY = "pi_address"
    PI_TYPE_KEY = "pi_type"
    PI_LOAD_KEY = "pi_load"
    # PI_PORT_KEY = "pi_port"
    PI_FULL_KEY = "pi_full"
    PI_EXTENDED_KEY = "pi_ext"
    PI_SLG_KEY = "pi_slg"
    PI_SOURCE_ADDRESS_KEY = "pi_source_address"
    # for specific results
    RES_CONNECTED = "connected"
    RES_PAIRED = "paired"
    RES_SERVICE_LOGIC = "service_logic"
    RES_INSTALLED_SN_KEY = "installed_sn"
    RES_EXISTING_SS_KEY = "existing_ss"
    # for platform instance read
    ROS_ROOT_KEY = "ROS_ROOT"
    ROS_DISTRO_KEY = "ROS_DISTRO"
    ROS_PACKAGE_PATH_KEY = "ROS_PACKAGE_PATH"
    LD_LIBRARY_PATH_KEY = "LD_LIBRARY_PATH"
    DEFAULT_LAUNCH_DIR_NAME = "launch"
    DEFAULT_SCRIPTS_DIR_NAME = "scripts"
    AVAILABLE_LAUNCHERS = "a_launchers"
    AVAILABLE_NODE_TYPES = "a_node_types"

    def __init__(self, pni, wdm):
        RCMPCommandAgent.__init__(self, pni)
        self._who = "platform node instance"
        self.ros_d_ld_lib_path_list = []
        self.ros_ws_ld_lib_path_list = []
        self.wdm = wdm

    # --- API ACCESSIBLE FROM HANDLER ---

    # the following commands can only be called in the RCM platform node master (
    # from the master to the master or from another platform node to the master)

    def manual_provisioning(self, params):
        """Provide a platform node name for a platform node of type R not already connected."""
        self._logger.info("%s called" % params)
        result = self.execute(operation=self.target_on_manual_provisioning, params=params)
        self._logger.info(result.get_txt())
        return result

    def automatic_provisioning(self, params):
        """Provide the connection of the platform node to the platform. This is called automatically in
        the main loop of the platform node."""
        self._logger.info("%s called" % params)
        result = self.execute(operation=self.target_on_automatic_provisioning, params=params)
        self._logger.info(result.get_txt())
        return result

    def rollback_provisioning(self, params):
        """Remove the platform node from the platform."""
        self._logger.info("%s called" % params)
        result = self.execute(operation=self.target_on_rollback_provisioning, params=params)
        self._logger.info(result.get_txt())
        return result

    def read_list(self, params):
        self._logger.info("%s called" % params)
        result = self.execute(operation=self.target_on_read_list, params=params)
        self._logger.info(result.get_txt())
        return result

    def discovery(self, params):
        """Discover the server with which the robot has to be bound."""
        self._logger.info("%s called" % params)
        result = self.execute(operation=self.target_on_discovery, params=params)
        self._logger.info(result.get_txt())
        return result

    # the following commands can be called in all types of platform nodes (from
    # master to platform node, from master to master, from platform node to master
    # and from platform node to platform node)

    # not standard management of this function (not execute() used)
    def read(self, params):
        """Read the information about the service space using the names of the platform
        node instance and the service space"""
        self._logger.info("%s called" % params)
        result = RCMPMessage()
        try:
            res = {}
            if self.pni[RCMPlatformNode.PNI_IS_MASTER]:
                res[self.RES_EXISTING_SS_KEY] = self.get_service_spaces(params)
            if self.I_ADDRESS_KEY in params and params[self.I_ADDRESS_KEY] and \
                    params[self.I_ADDRESS_KEY] != self.pni[RCMPlatformNode.PNI_ADDRESS]:
                # the read must be executed by another platform node
                d_res = self.delegate(params)
                if d_res.is_ok():
                    # the search of installed nodes has gone well
                    res.update(d_res.get_response_reason())
                else:
                    raise Exception(d_res.get_response_reason())
                result.create_ok_response(res)
            else:
                # this is the platform node that has to do the read
                res[self.RES_INSTALLED_SN_KEY] = self.find_packages_info()
                result.create_ok_response(res)
        except Exception as e:
            reason = "The %s read failed: %s" % (self._who, e)
            result.create_error_response(reason)
        self._logger.info(result.get_txt())
        return result

    def ping(self, params):
        """Ping the platform node to know if it is reachable and alive."""
        try:
            # self._logger.info("%s called" % params)
            result = self.execute(operation=self.target_on_ping, params=params, t_out=5.0)
        except Exception as e:
            reason = "Ping failed: %s" % e
            result = RCMPMessage()
            result.create_error_response(reason)
        # self._logger.info(result.get_txt())
        return result

    # --- MANUAL PROVISIONING ---

    def target_on_manual_provisioning(self, params):
        from rcmp_robotics_data import execute_robotics_data_query
        return execute_robotics_data_query(self.create_pi_name, params,
                                           err_reason="The manual provisioning of %s with name %s failed" %
                                                      (self._who, params[self.PI_NAME_KEY]))

    def create_pi_name(self, rdm, params):
        result = RCMPMessage()
        if self.PI_SLG_KEY in params:
            sl_id = rdm.get_sl_id_from_name(params[self.PI_SLG_KEY])
            if not (sl_id and sl_id[0]):
                reason = "Service logic '%s' doesn't exist" % params[self.PI_SLG_KEY]
                result.create_error_response(reason)
                return result
        else:
            reason = "A robot without service logic cannot be provisioned"
            result.create_error_response(reason)
            return result
        row = rdm.get_pi_ip_address_from_name(params[self.PI_NAME_KEY])
        if row:
            # there is a row with the passed name: we cannot have duplicated names
            reason = "A %s with name %s already exists" % (self._who, params[self.PI_NAME_KEY])
            result.create_error_response(reason)
        else:
            # no row with that platform instance name
            rdm.insert_platform_instance(params[self.PI_NAME_KEY], None, RCMPlatformNode.R_TYPE,
                                         s_logic=params[self.PI_SLG_KEY])
            reason = "The %s with name %s has been provisioned correctly" % (
                self._who, params[self.PI_NAME_KEY])
            result.create_ok_response(reason)
        return result

    # --- AUTOMATIC PROVISIONING ---

    def target_on_automatic_provisioning(self, params):
        from rcmp_robotics_data import execute_robotics_data_query
        return execute_robotics_data_query(self.create_pi, params,
                                           err_reason="The provisioning of %s with name '%s' failed" %
                                                      (self._who, params[self.PI_NAME_KEY]))

    def create_pi(self, rdm, params):
        result = RCMPMessage()
        row = rdm.get_pi_ip_address_from_name(params[self.PI_NAME_KEY])
        if params[self.PI_TYPE_KEY] == RCMPlatformNode.R_TYPE:
            # we are trying to provision a robot: in this case the name must be provided
            # by a user using the manual provisioning before this automatic provisioning
            # and will succeed only if there is already the name but it isn't mapped with
            # an ip address
            if row:
                # there is a row with the passed name
                if row[0]:
                    # this platform instance has already an ip address
                    if row[0] == params[self.PI_ADDRESS_KEY] and \
                            row[1] and row[1] == params[self.PI_TYPE_KEY]:
                        # the ip address is the same of the passed one
                        reason = "The %s with name '%s' was already provisioned correctly with address %s" % (
                            self._who, params[self.PI_NAME_KEY], params[self.PI_ADDRESS_KEY])
                        result.create_ok_response(reason)
                    else:
                        reason = "The %s with name '%s' already exists" % (self._who, params[self.PI_NAME_KEY])
                        result.create_error_response(reason)
                else:
                    rdm.update_ip_address_into_platform_instance(params[self.PI_ADDRESS_KEY],
                                                                 params[self.PI_NAME_KEY])
                    reason = "The %s with name '%s' has been provisioned correctly with address %s" % (
                        self._who, params[self.PI_NAME_KEY], params[self.PI_ADDRESS_KEY])
                    result.create_ok_response(reason)
            else:
                # no row with that platform instance name
                reason = "The %s with name '%s' doesn't exist" % (self._who, params[self.PI_NAME_KEY])
                result.create_error_response(reason)
        else:
            # we are trying to provision a vm: in this case will succeed only if there isn't
            # a name yet
            if not row:
                rdm.insert_platform_instance(params[self.PI_NAME_KEY], params[self.PI_ADDRESS_KEY],
                                             RCMPlatformNode.S_TYPE, RCMPlatformNode.S_LOAD)
                reason = "The %s with name '%s' has been provisioned correctly with address %s" % (
                    self._who, params[self.PI_NAME_KEY], params[self.PI_ADDRESS_KEY])
                result.create_ok_response(reason)
            elif row and row[0] and row[0] == params[self.PI_ADDRESS_KEY] and \
                    row[1] and row[1] == params[self.PI_TYPE_KEY]:
                # if the name already exists and the address is the same of the current
                # one we can consider the provisioning done correctly
                reason = "The %s with name '%s' was already provisioned correctly with address %s" % (
                    self._who, params[self.PI_NAME_KEY], params[self.PI_ADDRESS_KEY])
                result.create_ok_response(reason)
            else:
                reason = "The %s with name '%s' already exists but has a different ip address" % (
                    self._who, params[self.PI_NAME_KEY])
                result.create_error_response(reason)
        return result

    # --- ROLLBACK PROVISIONING ---

    def target_on_rollback_provisioning(self, params):
        from rcmp_robotics_data import execute_robotics_data_query
        return execute_robotics_data_query(self.delete_pi, params,
                                           err_reason="The provisioning rollback of %s with name %s failed" %
                                                      (self._who, params[self.PI_NAME_KEY]))

    def delete_pi(self, rdm, params):
        # in case of internal call: for robots we roll back half while for servers we roll back fully
        # in case of external call
        result = RCMPMessage()
        # for sure we are on the master because we are working with the robotics data
        if params[self.PI_NAME_KEY] == self.pni[RCMPlatformNode.PNI_NAME]:
            reason = "The %s with name '%s' cannot be deleted" % (self._who, params[self.PI_NAME_KEY])
            result.create_error_response(reason)
            return result
        row = rdm.get_pi_ip_address_from_name(params[self.PI_NAME_KEY])
        if row and row[1] and row[1] == RCMPlatformNode.S_TYPE:
            # we are trying to remove vms: this can be done only if full is required
            if self.PI_FULL_KEY in params and params[self.PI_FULL_KEY] is True:
                rdm.delete_platform_instance(params[self.PI_NAME_KEY], params[self.PI_FULL_KEY])
                reason = "The provisioning of the %s with name '%s' has been rolled back completely" % \
                         (self._who, params[self.PI_NAME_KEY])
            else:
                reason = "No rollback provisioning of the %s with name '%s' has to be done" % \
                         (self._who, params[self.PI_NAME_KEY])
            result.create_ok_response(reason)
        elif row and row[1] and row[1] == RCMPlatformNode.R_TYPE:
            # we are trying to remove robots
            if self.PI_EXTENDED_KEY in params and params[self.PI_EXTENDED_KEY]:
                # we are in the new case of fiware user in which robots will be removed completely
                if not row[0]:
                    # we can do the full delete of robot only in case is not connected that is
                    # when it doesn't have row[0] (the ip address)
                    rdm.delete_platform_instance(params[self.PI_NAME_KEY], True)
                    reason = "The provisioning of the %s with name '%s' has been rolled back completely" % \
                             (self._who, params[self.PI_NAME_KEY])
                    result.create_ok_response(reason)
                else:
                    reason = "The rollback provisioning of the %s with name '%s' cannot be done " \
                             "because is still connected" % (self._who, params[self.PI_NAME_KEY])
                    result.create_error_response(reason)
            elif self.PI_FULL_KEY in params and params[self.PI_FULL_KEY] is True:
                # we can do the full delete of robot only in case is not connected that is
                # when it doesn't have row[0] (the ip address)
                rdm.delete_platform_instance(params[self.PI_NAME_KEY], params[self.PI_FULL_KEY])
                reason = "The provisioning of the %s with name '%s' has been rolled back completely" % \
                         (self._who, params[self.PI_NAME_KEY])
                result.create_ok_response(reason)
            else:
                rdm.delete_platform_instance(params[self.PI_NAME_KEY], False)
                reason = "The provisioning of the %s with name '%s' has been rolled back half" % \
                         (self._who, params[self.PI_NAME_KEY])
                result.create_ok_response(reason)
        else:
            reason = "The %s with name '%s' was already deleted" % (self._who, params[self.PI_NAME_KEY])
            result.create_error_response(reason)
        return result

    # --- READ LIST ---

    def target_on_read_list(self, params):
        from rcmp_robotics_data import execute_robotics_data_query
        return execute_robotics_data_query(self.get_pi_list, params,
                                           err_reason="The read of the list of %ss failed" % self._who)

    def get_pi_list(self, rdm, params):
        full = None
        if self.PI_FULL_KEY in params and params[self.PI_FULL_KEY] or \
           self.PI_EXTENDED_KEY in params and params[self.PI_EXTENDED_KEY]:
            full = True
        rows = rdm.get_platform_instance_list(full)
        pi_list = []
        if rows:
            for row in rows:
                if row:
                    if self.PI_EXTENDED_KEY in params and params[self.PI_EXTENDED_KEY]:
                        pi_list.append({self.PI_NAME_KEY: row[0], self.PI_TYPE_KEY: row[1],
                                        self.PI_ADDRESS_KEY: row[2]})
                    elif self.PI_FULL_KEY in params and params[self.PI_FULL_KEY]:
                        if row[1] and row[1] == RCMPlatformNode.R_TYPE:
                            if row[4]:
                                sl_row = rdm.get_sl_name_from_id(row[4])
                            pi_r_row = rdm.get_connection_from_connection_pi_r(row[0])
                            pi_list.append({self.PI_NAME_KEY: row[0], self.PI_TYPE_KEY: row[1],
                                            self.RES_CONNECTED: row[2] is not None,
                                            self.RES_SERVICE_LOGIC: sl_row[0] if (sl_row and sl_row[0]) else "",
                                            self.RES_PAIRED: pi_r_row[0] if (pi_r_row and pi_r_row[0]) else ""})
                        else:
                            pi_list.append({self.PI_NAME_KEY: row[0], self.PI_TYPE_KEY: row[1],
                                            self.RES_CONNECTED: row[2] is not None, self.PI_LOAD_KEY: row[3]})
                    else:
                        if row[2]:
                            sl_row = rdm.get_sl_name_from_id(row[2])
                        pi_r_row = rdm.get_connection_from_connection_pi_r(row[0])
                        pi_list.append({self.PI_NAME_KEY: row[0], self.RES_CONNECTED: row[1] is not None,
                                        self.RES_SERVICE_LOGIC: sl_row[0] if (sl_row and sl_row[0]) else "",
                                        self.RES_PAIRED: (pi_r_row and pi_r_row[0]) is not None})
        result = RCMPMessage()
        result.create_ok_response(pi_list)
        return result

    # --- DISCOVERY ---

    def target_on_discovery(self, params):
        from rcmp_robotics_data import execute_robotics_data_query
        return execute_robotics_data_query(self.get_pi, params, err_reason="The discovery failed")

    def get_pi(self, rdm, params):
        result = RCMPMessage()
        rows = rdm.get_unloaded_s_pi()
        if rows:
            for row in rows:
                if row and row[0] and row[1]:
                    # row[0] is not important, but row[1] should keep the ip address so it
                    # has to have a value
                    reason = {self.PI_NAME_KEY: row[0], self.PI_ADDRESS_KEY: row[1]}
                    result.create_ok_response(reason)
                    return result
        # we haven't found a server to use
        reason = "No servers available."
        result.create_error_response(reason)
        return result

    def get_service_spaces(self, params):
        # Take PI_NAME_KEY from params and get all the service spaces running on that RCM
        # platform node instance. Add I_ADDRESS_KEY in params with the ip address of this
        # instance
        from rcmp_robotics_data import execute_int_robotics_data_query
        if self.PI_NAME_KEY not in params or (self.PI_NAME_KEY in params and not params[self.PI_NAME_KEY]):
            raise SyntaxError("%s is missing in the request" % self.PI_NAME_KEY)
        return execute_int_robotics_data_query(self.get_ss, params,
                                               "Unable to provide the list of service spaces "
                                               "for the %s named '%s'" % (self._who, params[self.PI_NAME_KEY]))

    def get_ss(self, rdm, params):
        # we need the address of the platform node too
        row = rdm.get_pi_ip_address_from_name(params[self.PI_NAME_KEY])
        if row and row[0]:
            params[self.I_ADDRESS_KEY] = row[0]
        else:
            raise ValueError("'%s' doesn't exist" % params[self.PI_NAME_KEY])
        rows = rdm.get_service_space_list_from_pi(params[self.PI_NAME_KEY])
        ss_list = []
        if rows:
            for row in rows:
                if row:
                    ss_list.append({ServiceSpace.SS_NAME_KEY: row[0], ServiceSpace.SS_PORT_KEY: row[1]})
        return ss_list

    def find_packages_info(self):
        import string
        # get the ros home
        path_list = os.environ[self.ROS_ROOT_KEY].split(os.path.sep)
        ros_home = string.join(path_list[:path_list.index(os.environ[self.ROS_DISTRO_KEY])+1], os.path.sep)
        # there are two types of paths in both ROS_PACKAGE_PATH and LD_LIBRARY_PATH, workspaces paths
        # and ros distro paths
        # divide lib dir for distribution from that for workspaces: this is done only at the
        # beginning to optimize the access to LD_LIBRARY_PATH
        ld_lib_path_list = os.environ[self.LD_LIBRARY_PATH_KEY].split(os.pathsep)
        for ld_lib_path in ld_lib_path_list:
            if os.path.exists(ld_lib_path) and os.path.isdir(ld_lib_path):
                if ros_home in ld_lib_path:
                    self.ros_d_ld_lib_path_list.append(ld_lib_path)
                else:
                    self.ros_ws_ld_lib_path_list.append(ld_lib_path)
        ros_packages = {}
        # search for packages: ROS_PACKAGE_PATH contains the paths for all the directories that can contain
        # the packages; the packages are folder that contain an xml file defining the package and can have
        # a launch file in case we have a launcher for that package
        ros_pkg_path_list = os.environ[self.ROS_PACKAGE_PATH_KEY].split(os.pathsep)
        for ros_pkg_path in ros_pkg_path_list:
            if os.path.exists(ros_pkg_path) and os.path.isdir(ros_pkg_path):
                if ros_home in ros_pkg_path:
                    is_in_ros_d = True
                else:
                    is_in_ros_d = False
                # all the directories in these paths are ROS packages
                for pkg_name in os.listdir(ros_pkg_path):
                    pkg_path = os.path.join(ros_pkg_path, pkg_name)
                    if os.path.isdir(pkg_path):
                        # only the directories are packages
                        a_launchers = []
                        launch_filter = os.path.join(pkg_path, "*/*.launch")
                        import glob
                        for launch_path in glob.glob(launch_filter):
                            prefix_to_remove = os.path.sep.join([pkg_path, ""])
                            launch_name = launch_path.replace(prefix_to_remove, "")
                            launch_prefix = os.path.sep.join([self.DEFAULT_LAUNCH_DIR_NAME, ""])
                            if launch_name.startswith(launch_prefix):
                                launch_name = launch_name.replace(launch_prefix, "")
                            a_launchers.append(launch_name)
                        a_node_types = self.find_node_types(pkg_name, pkg_path, is_in_ros_d)
                        if a_launchers or a_node_types:
                            ros_packages[pkg_name] = {self.AVAILABLE_LAUNCHERS: a_launchers,
                                                      self.AVAILABLE_NODE_TYPES: a_node_types}
        return ros_packages

    def find_node_types(self, pkg_name, pkg_path, is_in_ros_d):
        a_node_types = []
        # we use 2 branches only because we want decrease the number of directory accesses
        if is_in_ros_d:
            # these are ros distribution so we search the nodes only in the folders
            # of the distribution
            for ros_d_ld_lib_path in self.ros_d_ld_lib_path_list:
                pkg_lib_path = os.path.join(ros_d_ld_lib_path, pkg_name)
                # we go only in the folders that we found before that are packages
                if os.path.exists(pkg_lib_path) and os.path.isdir(pkg_lib_path):
                    # in the directory of each package there are executable files or
                    # python modules that represent the node types (nodes we can run)
                    for node_type in os.listdir(pkg_lib_path):
                        node_type_path = os.path.join(pkg_lib_path, node_type)
                        if not os.path.isdir(node_type_path):
                            a_node_types.append(node_type)
        else:
            # the nodes of python type or in general of script type are in the same folder
            # identifying the package: we search them only in case of workspaces because
            # in distribution folder the scripts are mainly for debug and test (in general
            # not node types); the only folder we search in is script* and take all there
            # is
            n_type_filter = os.path.join(pkg_path, "scripts/*")
            import glob
            for n_type_path in glob.glob(n_type_filter):
                prefix_to_remove = os.path.sep.join([pkg_path, ""])
                n_type = n_type_path.replace(prefix_to_remove, "")
                scripts_prefix = os.path.sep.join([self.DEFAULT_SCRIPTS_DIR_NAME, ""])
                if n_type.startswith(scripts_prefix):
                    n_type = n_type.replace(scripts_prefix, "")
                a_node_types.append(n_type)
            for ros_ws_ld_lib_path in self.ros_ws_ld_lib_path_list:
                pkg_lib_path = os.path.join(ros_ws_ld_lib_path, pkg_name)
                # we go only in the folders that we found before that are packages
                if os.path.exists(pkg_lib_path) and os.path.isdir(pkg_lib_path):
                    # in the directory of each package there are executable files or
                    # python modules that represent the node types (nodes we can run)
                    for node_type in os.listdir(pkg_lib_path):
                        node_type_path = os.path.join(pkg_lib_path, node_type)
                        if not os.path.isdir(node_type_path):
                            a_node_types.append(node_type)
        return a_node_types

    # --- PING ---

    def target_on_ping(self, params):
        reason = "Ping ack"
        result = RCMPMessage()
        if PNodeInstance.PI_SOURCE_ADDRESS_KEY in params and params[PNodeInstance.PI_SOURCE_ADDRESS_KEY] and \
                not self.wdm.is_watchdog_available(params):
            # this is a conditional ping that check if the watchdog corresponding to the source of
            # the ping is already available
            reason = "The watchdog on %s paired with %s is no more available" % \
                     (self.pni[RCMPlatformNode.PNI_ADDRESS], params[PNodeInstance.PI_SOURCE_ADDRESS_KEY])
            result.create_error_response(reason)
        else:
            result.create_ok_response(reason)
        return result
