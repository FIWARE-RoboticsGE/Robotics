__author__ = "Fabio Giuseppe Di Benedetto"

import subprocess
import select
import re
import os
import copy

from rcmp_node import signal, threading
from rcmp_command import RCMPlatformNode, RCMPCommandAgent, RCMPMessage


class SLogicInstance(RCMPCommandAgent):
    SLG_NAME_KEY = "slg_name"
    SLG_SS_SIDE_KEY = "ss_side"
    SLG_SN_SIDE_KEY = "sn_side"
    SLG_SL_SIDE_KEY = "sl_side"
    SLG_SN_LIST_KEY = "sn_list"
    SLG_SL_LIST_KEY = "sl_list"

    def __init__(self, pni):
        RCMPCommandAgent.__init__(self, pni)
        self._who = "service logic"

    # --- API ACCESSIBLE FROM HANDLER ---

    # the following commands can only be called in the RCM platform node master
    # (from the master to the master)

    def manual_provisioning(self, params):
        """Provide a new service logic usable in the platform."""
        self._logger.info("%s called" % params)
        result = self.execute(operation=self.target_on_manual_provisioning, params=params)
        self._logger.info(result.get_txt())
        return result

    def rollback_provisioning(self, params):
        """Remove the specified service logic name."""
        self._logger.info("%s called" % params)
        result = self.execute(operation=self.target_on_rollback_provisioning, params=params)
        self._logger.info(result.get_txt())
        return result

    def read_list(self, params):
        """Read the list of service logics available in the platform."""
        self._logger.info("%s called" % params)
        result = self.execute(operation=self.target_on_read_list, params=params)
        self._logger.info(result.get_txt())
        return result

    def read(self, params):
        """Read the information about the specified service logic name."""
        self._logger.info("%s called" % params)
        result = self.execute(operation=self.target_on_read, params=params)
        self._logger.info(result.get_txt())
        return result

    # --- MANUAL PROVISIONING ---

    def target_on_manual_provisioning(self, params):
        from rcmp_robotics_data import execute_robotics_data_query
        return execute_robotics_data_query(self.create_slg, params,
                                           err_reason="The provisioning of %s with name %s failed" %
                                                      (self._who, params[self.SLG_NAME_KEY]))

    def create_slg(self, rdm, params):
        self._logger.debug("-- TMP -- create_slg params: %s" % params)
        result = RCMPMessage()
        slg_id = rdm.get_sl_id_from_name(params[self.SLG_NAME_KEY])
        if slg_id and slg_id[0]:
            reason = "The %s '%s' already exists" % (self._who, params[self.SLG_NAME_KEY])
            result.create_error_response(reason)
            return result
        # default destination for the service space is the server
        ss_side = RCMPlatformNode.S_TYPE
        if self.SLG_SS_SIDE_KEY in params and params[self.SLG_SS_SIDE_KEY] and \
                (params[self.SLG_SS_SIDE_KEY] == RCMPlatformNode.S_TYPE or
                 params[self.SLG_SS_SIDE_KEY] == RCMPlatformNode.R_TYPE):
            # this option is available only using slg_full and putting ss_side
            # from the web service
            self._logger.debug("-- TMP -- service space side provided: %s" % params[self.SLG_SS_SIDE_KEY])
            ss_side = params[self.SLG_SS_SIDE_KEY]
        rdm.insert_service_logic(params[self.SLG_NAME_KEY], ss_side)
        try:
            if self.SLG_SN_LIST_KEY in params and params[self.SLG_SN_LIST_KEY]:
                self._logger.debug("-- TMP -- service node list provided")
                for slg_item in params[self.SLG_SN_LIST_KEY]:
                    self._logger.debug("-- TMP -- service node: %s" % slg_item)
                    if ServiceNode.SN_PACKAGE_KEY in slg_item and slg_item[ServiceNode.SN_PACKAGE_KEY] and \
                       ServiceNode.SN_TYPE_KEY in slg_item and slg_item[ServiceNode.SN_TYPE_KEY]:
                        self._logger.debug("-- TMP -- service node '%s' '%s'" %
                                           (slg_item[ServiceNode.SN_PACKAGE_KEY], slg_item[ServiceNode.SN_TYPE_KEY]))
                        sn_name = slg_item[ServiceNode.SN_NAME_KEY] \
                            if ServiceNode.SN_NAME_KEY in slg_item and slg_item[ServiceNode.SN_NAME_KEY] \
                            else None
                        self._logger.debug("-- TMP -- service node name: %s" % sn_name)
                        sn_params = slg_item[ServiceNode.SN_PARAMS_KEY] \
                            if ServiceNode.SN_PARAMS_KEY in slg_item and slg_item[ServiceNode.SN_PARAMS_KEY] \
                            else None
                        self._logger.debug("-- TMP -- service node params: %s" % sn_params)
                        if self.SLG_SN_SIDE_KEY in slg_item and slg_item[self.SLG_SN_SIDE_KEY] and \
                                (slg_item[self.SLG_SN_SIDE_KEY] == RCMPlatformNode.S_TYPE or
                                 slg_item[self.SLG_SN_SIDE_KEY] == RCMPlatformNode.R_TYPE):
                            sn_side = slg_item[self.SLG_SN_SIDE_KEY]
                            self._logger.debug("-- TMP -- service node side: %s" % sn_side)
                        else:
                            # the side is not provided correctly so the insertion fails and
                            # we have to come back to the original situation
                            self.delete_slg(rdm, params)
                            reason = "The side where start the service node with package '%s' and " \
                                     "type '%s' in the %s named '%s' is not provided" % \
                                     (slg_item[ServiceNode.SN_PACKAGE_KEY], slg_item[ServiceNode.SN_TYPE_KEY],
                                      self._who, params[self.SLG_NAME_KEY])
                            result.create_error_response(reason)
                            return result
                        si_id = rdm.get_service_item(slg_item[ServiceNode.SN_PACKAGE_KEY],
                                                     slg_item[ServiceNode.SN_TYPE_KEY])
                        if not (si_id and si_id[0]):
                            self._logger.debug("-- TMP -- service node is new")
                            rdm.insert_service_item(slg_item[ServiceNode.SN_PACKAGE_KEY],
                                                    slg_item[ServiceNode.SN_TYPE_KEY],
                                                    ServiceRCMPCommandAgent.SLI_N_TYPE)
                        rdm.insert_service_logic_item(params[self.SLG_NAME_KEY],
                                                      slg_item[ServiceNode.SN_PACKAGE_KEY],
                                                      slg_item[ServiceNode.SN_TYPE_KEY],
                                                      sn_side, sn_name, sn_params)
            if self.SLG_SL_LIST_KEY in params and params[self.SLG_SL_LIST_KEY]:
                self._logger.debug("-- TMP -- service launcher list provided")
                for slg_item in params[self.SLG_SL_LIST_KEY]:
                    if ServiceLauncher.SL_PACKAGE_KEY in slg_item and slg_item[ServiceLauncher.SL_PACKAGE_KEY] and \
                       ServiceLauncher.SL_F_LAUNCHER_KEY in slg_item and slg_item[ServiceLauncher.SL_F_LAUNCHER_KEY]:
                        sl_name = slg_item[ServiceLauncher.SL_NAME_KEY] \
                            if ServiceLauncher.SL_NAME_KEY in slg_item and slg_item[ServiceLauncher.SL_NAME_KEY] \
                            else None
                        sl_params = slg_item[ServiceLauncher.SL_PARAMS_KEY] \
                            if ServiceLauncher.SL_PARAMS_KEY in slg_item and slg_item[ServiceLauncher.SL_PARAMS_KEY] \
                            else None
                        if self.SLG_SL_SIDE_KEY in slg_item and slg_item[self.SLG_SL_SIDE_KEY] and \
                                (slg_item[self.SLG_SL_SIDE_KEY] == RCMPlatformNode.S_TYPE or
                                 slg_item[self.SLG_SL_SIDE_KEY] == RCMPlatformNode.R_TYPE):
                            sl_side = slg_item[self.SLG_SL_SIDE_KEY]
                        else:
                            # the side is not provided correctly so the insertion fails and
                            # we have to come back to the original situation
                            self.delete_slg(rdm, params)
                            reason = "The side where start the service launcher with package '%s' and " \
                                     "launch file '%s' in the %s named '%s' is not provided" % \
                                     (slg_item[ServiceLauncher.SL_PACKAGE_KEY],
                                      slg_item[ServiceLauncher.SL_F_LAUNCHER_KEY],
                                      self._who, params[self.SLG_NAME_KEY])
                            result.create_error_response(reason)
                            return result
                        si_id = rdm.get_service_item(slg_item[ServiceLauncher.SL_PACKAGE_KEY],
                                                     slg_item[ServiceLauncher.SL_F_LAUNCHER_KEY])
                        if not (si_id and si_id[0]):
                            # we create the service item only if doesn't already exist
                            rdm.insert_service_item(slg_item[ServiceLauncher.SL_PACKAGE_KEY],
                                                    slg_item[ServiceLauncher.SL_F_LAUNCHER_KEY],
                                                    ServiceRCMPCommandAgent.SLI_L_TYPE)
                        rdm.insert_service_logic_item(params[self.SLG_NAME_KEY],
                                                      slg_item[ServiceLauncher.SL_PACKAGE_KEY],
                                                      slg_item[ServiceLauncher.SL_F_LAUNCHER_KEY],
                                                      sl_side, sl_name, sl_params)
            reason = "The %s with name '%s' has been provisioned correctly" % (self._who, params[self.SLG_NAME_KEY])
            result.create_ok_response(reason)
        except Exception as e:
            # in case of error after the creation of the service logic we have to rollback
            # and then raise the exception caught
            self.delete_slg(rdm, params)
            raise e
        return result

    # --- ROLLBACK PROVISIONING ---

    def target_on_rollback_provisioning(self, params):
        from rcmp_robotics_data import execute_robotics_data_query
        return execute_robotics_data_query(self.delete_slg, params,
                                           err_reason="The provisioning rollback of %s with name '%s' failed" %
                                                      (self._who, params[self.SLG_NAME_KEY]))

    def delete_slg(self, rdm, params):
        result = RCMPMessage()
        slg_id = rdm.get_sl_id_from_name(params[self.SLG_NAME_KEY])
        if slg_id and slg_id[0]:
            sl_uses = rdm.get_sl_uses(params[self.SLG_NAME_KEY])
            if sl_uses and sl_uses[0] and sl_uses[0] > 0:
                reason = "The %s '%s' is used by a platform instance and cannot be removed" % \
                         (self._who, params[self.SLG_NAME_KEY])
                result.create_error_response(reason)
            else:
                si_list = rdm.get_service_items_from_sl_name(params[self.SLG_NAME_KEY])
                if si_list:
                    for si_id in si_list:
                        if si_id[0]:
                            si_uses = rdm.get_service_item_uses(si_id[0])
                            if si_uses and si_uses[0] and si_uses[0] == 1:
                                # only the service logic that we are trying to remove uses this service item
                                # so we can remove even the service item
                                rdm.delete_service_item(si_id[0])
                            rdm.delete_service_logic_item(params[self.SLG_NAME_KEY], si_id[0])
                rdm.delete_service_logic(params[self.SLG_NAME_KEY])
                reason = "The provisioning of the %s with name '%s' has been rolled back completely" % \
                         (self._who, params[self.SLG_NAME_KEY])
                result.create_ok_response(reason)
        else:
            reason = "The %s '%s' doesn't exist" % (self._who, params[self.SLG_NAME_KEY])
            result.create_error_response(reason)
        return result

    # --- READ LIST ---

    def target_on_read_list(self, params):
        from rcmp_robotics_data import execute_robotics_data_query
        return execute_robotics_data_query(self.get_slg_list, params,
                                           err_reason="The read of the list of %ss failed" % self._who)

    def get_slg_list(self, rdm, params):
        rows = rdm.get_service_logic_list()
        slg_list = []
        if rows:
            for row in rows:
                if row:
                    slg_list.append({self.SLG_NAME_KEY: row[0]})
        result = RCMPMessage()
        result.create_ok_response(slg_list)
        return result

    # --- READ ---

    def target_on_read(self, params):
        from rcmp_robotics_data import execute_robotics_data_query
        return execute_robotics_data_query(self.get_slg_info, params,
                                           err_reason="The read of the %s with name '%s' failed" %
                                                      (self._who, params[self.SLG_NAME_KEY]))

    def get_slg_info(self, rdm, params):
        result = RCMPMessage()
        row = rdm.get_sl_ss_pi_type_target_from_sl_name(params[self.SLG_NAME_KEY])
        if row and row[0]:
            slg_sn_list = []
            slg_sl_list = []
            slg = {self.SLG_SS_SIDE_KEY: row[0], self.SLG_SN_LIST_KEY: slg_sn_list,
                   self.SLG_SL_LIST_KEY: slg_sl_list}
            slg_items = rdm.get_sli_from_sl_name(params[self.SLG_NAME_KEY])
            if slg_items:
                for slgi in slg_items:
                    if slgi[0] and slgi[1] and slgi[2] and slgi[3]:
                        # all this parameters are mandatory
                        # sli[0] is the name of the package for that item, sli[1] is the tag of the item
                        # representing the node type in case of node or the launcher name in case of launcher
                        # sli[2] is service_item type (node or launcher) and is used to know what type of
                        # service we have to launch
                        # sli[3] is service_logic_item si_pi_type_target (meaning where to launch
                        # the item and can be S or R)
                        if slgi[2] == ServiceSpace.SLI_N_TYPE:
                            # node
                            sn = {ServiceNode.SN_PACKAGE_KEY: slgi[0], ServiceNode.SN_TYPE_KEY: slgi[1],
                                  self.SLG_SN_SIDE_KEY: slgi[3]}
                            # the following are optional parameters: sli[4] is a name for the service item
                            # and sli[5] is a list of params to pass to the service
                            if slgi[4]:
                                sn[ServiceNode.SN_NAME_KEY] = slgi[4]
                            if slgi[5]:
                                sn[ServiceNode.SN_PARAMS_KEY] = slgi[5]
                            slg_sn_list.append(sn)
                        elif slgi[2] == ServiceSpace.SLI_L_TYPE:
                            # launcher
                            sl = {ServiceLauncher.SL_PACKAGE_KEY: slgi[0],
                                  ServiceLauncher.SL_F_LAUNCHER_KEY: slgi[1],
                                  self.SLG_SL_SIDE_KEY: slgi[3]}
                            # the following are optional parameters: sli[4] is a name for the service item
                            # and sli[5] is a list of params to pass to the service
                            if slgi[4]:
                                sl[ServiceLauncher.SL_NAME_KEY] = slgi[4]
                            if slgi[5]:
                                sl[ServiceLauncher.SL_PARAMS_KEY] = slgi[5]
                            slg_sl_list.append(sl)
                    else:
                        # some mandatory parameter missing in the service logic
                        self._logger.warning("Service logic item with mandatory parameter missing exists")
            result.create_ok_response(slg)
        else:
            reason = "The %s with name '%s' doesn't exist" % (self._who, params[self.SLG_NAME_KEY])
            result.create_error_response(reason)
        return result


class ServiceRCMPCommandAgent(RCMPCommandAgent):
    RMASTER_URI_KEY = "ROS_MASTER_URI"
    SS_NAME_KEY = "ss_name"
    SS_ADDRESS_KEY = "ss_address"
    SS_PORT_KEY = "ss_port"
    SS_EXT_ADDRESS_KEY = "ss_ext_address"
    SS_EXT_PORT_KEY = "ss_ext_port"
    OUT_KEY = "out"
    ERR_KEY = "err"
    # service logic items keys
    SLI_N_TYPE = "N"
    SLI_L_TYPE = "L"
    # keys for data extraction from output
    P_NAME_KEY = "process_name"
    P_PID_KEY = "process_pid"
    RL_S_ADDR_KEY = "roslaunch_server_address"
    RM_ADDRESS_KEY = "ros_master_address"
    ROUT_NAME_KEY = "rosout_name"
    RN_NAME_KEY = "ros_node_name"
    RN_ADDR_KEY = "ros_node_address"
    RN_PID_KEY = "ros_node_pid"
    # keys for s_data
    PS_KEY = "ps"
    PS_PEEP_STOP_KEY = "ps_peep_stop"
    PS_PEEPER_KEY = "ps_peeper"
    # to check if is a launcher
    LAUNCH_EXT_TAG = ".launch"

    def __init__(self, pni, msm):
        RCMPCommandAgent.__init__(self, pni)
        # there can be nodes that doesn't write on stdout an initial msg
        # so we can't put a too much long timeout: too short is even a problem
        # because it doesn't give time to the node to do the operation and
        # so it could happen that the check doesn't find the node running on
        # ROS yet: remember that the socket timeout must be more than this
        # timeout otherwise the socket fails before the operation at the other
        # end of the communication channel
        self._timeout = 5
        self._max_retry = 2
        # variables for data extraction from output stream
        # A valid name in ROS has the following characteristics:
        # First character is an alpha character ([a-z|A-Z]), tilde (~) or forward slash (/)
        # Subsequent characters can be alphanumeric ([0-9|a-z|A-Z]), underscores (_), or forward slashes (/)
        # ~ means private and is not used in this case, so we avoid to find in re
        self.p_name_extractor_str = '(?P<%s>[a-zA-Z0-9\_\/]+)' % self.P_NAME_KEY
        # startup phase management
        self.p_pid_extractor_str = '(?P<%s>[0-9]+)' % self.P_PID_KEY
        self.rl_s_addr_extractor_str = '(?P<%s>[a-zA-Z0-9\_\-\.\/\:]+)' % self.RL_S_ADDR_KEY
        self.rm_addr_extractor_str = '(?P<%s>[a-zA-Z0-9\_\-\.\/\:]+)' % self.RM_ADDRESS_KEY
        self.ro_name_extractor_str = '(?P<%s>[a-zA-Z0-9\_\/]+)' % self.ROUT_NAME_KEY
        self.startup_p_info_extractor_str = 'process\[%s(\-?[0-9]+)?\]: started with pid \[%s\]' % (
            self.p_name_extractor_str, self.p_pid_extractor_str)
        self.roslaunch_s_addr_extractor_str = 'started roslaunch server %s' % self.rl_s_addr_extractor_str
        self.rosmaster_addr_extractor_str = 'ROS_MASTER_URI=%s' % self.rm_addr_extractor_str
        self.rosout_name_extractor_str = '(started )?core service \[%s\]( found)?' % self.ro_name_extractor_str
        self.rn_name_extractor_str = '(?P<%s>[a-zA-Z0-9\_\/]+)' % self.RN_NAME_KEY
        self.rn_addr_extractor_str = '(https?\:\/+)?(?P<%s>[a-zA-Z0-9\_\-\.\/]+)(\:[0-9]*\/?)' % self.RN_ADDR_KEY
        self.rn_list_extractor_str = '\/%s' % self.rn_name_extractor_str
        self.rn_list_wa_extractor_str = '%s\s+\/%s' % (self.rn_addr_extractor_str, self.rn_name_extractor_str)
        self.rn_pid_extractor_str = 'Pid:\s+(?P<%s>[0-9]+)' % self.RN_PID_KEY
        # process[<P_NAME_KEY>-<process_counter>]: started with pid [<P_PID_KEY>]
        self.startup_p_info_extractor_re = re.compile(self.startup_p_info_extractor_str)
        # started roslaunch server <RL_S_ADDR_KEY>
        self.roslaunch_s_addr_extractor_re = re.compile(self.roslaunch_s_addr_extractor_str)
        # ROS_MASTER_URI=<RM_ADDRESS_KEY>
        self.rosmaster_addr_extractor_re = re.compile(self.rosmaster_addr_extractor_str)
        # started core service [ROUT_NAME_KEY]
        # core service [ROUT_NAME_KEY] found
        self.rosout_name_extractor_re = re.compile(self.rosout_name_extractor_str)
        # shutdown phase management
        self.shutdown_p_name_extractor_str = '\[%s(\-?[0-9]+)?\] killing on exit' % self.p_name_extractor_str
        # [<P_NAME_KEY>-<process_counter>] killing on exit
        self.shutdown_p_name_extractor_re = re.compile(self.shutdown_p_name_extractor_str)
        self.shutdown_seq_str = 'shutting down processing monitor'
        self.shutdown_seq_end_str = 'done'
        # /<RN_NAME_KEY>
        self.rn_list_extractor_re = re.compile(self.rn_list_extractor_str)
        # <RN_ADDR_KEY><padding>/<RN_NAME_KEY>
        self.rn_list_wa_extractor_re = re.compile(self.rn_list_wa_extractor_str)
        # Pid:<padding><RN_PID_KEY>
        self.rn_pid_extractor_re = re.compile(self.rn_pid_extractor_str)
        # deep copy of os.environ in case we have to change the environment variable
        # exclusively for that specific subprocess
        self.env = None
        self.msm = msm

    # --- API ACCESSIBLE FROM HANDLER ---

    def start(self, params):
        try:
            self._logger.info("%s called" % params)
            result = self.execute(m_operation=self.master_on_cmd, operation=self.target_on_start, params=params)
        except Exception as e:
            reason = "The %s start failed: %s" % (self._who, e)
            result = RCMPMessage()
            result.create_error_response(reason)
        self._logger.info(result.get_txt())
        return result

    def stop(self, params):
        try:
            self._logger.info("%s called" % params)
            result = self.execute(m_operation=self.master_on_cmd, operation=self.target_on_stop, params=params)
        except Exception as e:
            reason = "The %s stop failed: %s" % (self._who, e)
            result = RCMPMessage()
            result.create_error_response(reason)
        self._logger.info(result.get_txt())
        return result

    def stop_all_local(self, params):
        try:
            self._logger.info("%s called" % params)
            result = self.execute(operation=self.target_on_stop, params=params)
        except Exception as e:
            reason = "The stop all %ss failed: %s" % (self._who, e)
            result = RCMPMessage()
            result.create_error_response(reason)
        self._logger.info(result.get_txt())
        return result

    def read_list(self, params):
        """Read all the services (nodes, launchers) running in this RCM platform node instance."""
        try:
            self._logger.info("%s called" % params)
            result = self.execute(m_operation=self.master_on_cmd, operation=self.target_on_read_list, params=params)
        except Exception as e:
            reason = "The read of the %ss failed: %s" % (self._who, e)
            result = RCMPMessage()
            result.create_error_response(reason)
        self._logger.info(result.get_txt())
        return result

    # --- GENERAL PURPOSE ---

    def read_all_so_far(self, stream_out, stream_err, p=None):
        # WARNING this only works on linux
        data_out = ""
        data_err = ""
        # waiting at most the timeout
        readable_list = select.select([stream_out, stream_err], [], [], self._timeout)[0]
        while readable_list:
            # some readable object are available for the read
            for readable in readable_list:
                # we read one byte at time (so doesn't block on read)
                if readable.fileno() == stream_out.fileno():
                    data_out += readable.readline()
                if readable.fileno() == stream_err.fileno():
                    data_err += readable.readline()
            if p and p.poll() is not None:
                # if the process goes down the streams remain always readable so we have
                # to break from the while
                readable_list = None
            else:
                # this is with timeout 0 so never blocks: this is because we have to finish
                # the read we already started
                readable_list = select.select([stream_out, stream_err], [], [], 0)[0]
        return data_out, data_err

    def get_i_entry(self, params):
        """Take I_NAME_KEY from params and change it with I_ADDRESS_KEY."""
        from rcmp_robotics_data import execute_int_robotics_data_query
        if self.I_NAME_KEY not in params:
            raise SyntaxError("%s is missing in the request" % self.I_NAME_KEY)
        return execute_int_robotics_data_query(self.get_i, params,
                                               "Unable to provide the platform node instance address "
                                               "for '%s'" % params[self.I_NAME_KEY])

    def get_i(self, rdm, params):
        row = rdm.get_pi_ip_address_from_name(params[self.I_NAME_KEY])
        if row and row[0]:
            params[self.I_ADDRESS_KEY] = row[0]
        else:
            # we haven't found the platform node instance
            reason = "The platform node instance '%s' is not available" % (params[self.I_NAME_KEY])
            raise IOError(reason)

    def get_ss_entry(self, params):
        """Take SS_NAME_KEY from params and change it with SS_ADDRESS_KEY and SS_PORT_KEY."""
        from rcmp_robotics_data import execute_int_robotics_data_query
        if self.SS_NAME_KEY not in params:
            raise SyntaxError("%s is missing in the request" % self.SS_NAME_KEY)
        return execute_int_robotics_data_query(self.get_ss, params,
                                               "Unable to provide the address and port for "
                                               "the %s named '%s'" % (self._who, params[self.SS_NAME_KEY]))

    def get_ss(self, rdm, params):
        row = rdm.get_ss_ip_address_port(params[self.SS_NAME_KEY])
        if row:
            ss_address, ss_port = row
            if ss_address:
                params[self.SS_ADDRESS_KEY] = ss_address
            if ss_port:
                params[self.SS_PORT_KEY] = str(ss_port)
            # TODO this is custom for firos
            self.get_custom_info(rdm, params)
        else:
            # we haven't found the platform node instance
            reason = "The service space '%s' is not available" % (params[self.SS_NAME_KEY])
            raise IOError(reason)

    def get_custom_info(self, rdm, params):
        from rcmp_command import RCMPCommandHandler, READ_SERVICE_SPACE
        if RCMPCommandHandler.COMMAND_KEY in params and \
           params[RCMPCommandHandler.COMMAND_KEY] == READ_SERVICE_SPACE:
            ss_ext_port = rdm.get_ss_external_access_used(params[self.SS_NAME_KEY])
            if ss_ext_port and ss_ext_port[0]:
                params[self.SS_EXT_PORT_KEY] = str(ss_ext_port[0])

    def master_on_cmd(self, params):
        if self.I_NAME_KEY in params and params[self.I_NAME_KEY] and \
                not (self.I_ADDRESS_KEY in params and params[self.I_ADDRESS_KEY]):
            self.get_i_entry(params)
        if self.SS_NAME_KEY in params and params[self.SS_NAME_KEY] and \
                not (self.SS_ADDRESS_KEY and self.SS_PORT_KEY in params and
                     params[self.SS_ADDRESS_KEY] and params[self.SS_PORT_KEY]):
            self.get_ss_entry(params)

    def op_replicator(self, params, s_list, operation):
        """Replicate a specified operation on a list of services."""
        failed = False
        result_list = []
        for s_i in s_list:
            if s_i:
                # the operation needs the service item so we pass it
                res = operation(params, s_i)
                result_list.append(res.get_dict())
                if not res.is_ok():
                    # if even one operation returns an error the ss_multiplier will return an error
                    failed = True
        result = RCMPMessage()
        if failed:
            result.create_error_response(result_list)
        else:
            result.create_ok_response(result_list)
        return result

    # --- START ---

    def target_on_start(self, params):
        # this is the platform node that has to do the start
        s_data = {}
        cmd = self.prepare_start(params)
        self._logger.info("Executing command %s" % cmd)
        s_data[self.PS_KEY] = subprocess.Popen(cmd, env=self.env, stdout=subprocess.PIPE,
                                               stderr=subprocess.PIPE)
        rtn_code = s_data[self.PS_KEY].poll()
        result = self.check_start(rtn_code, params, s_data)
        return result

    def prepare_start(self, params):
        return []

    def check_start(self, rtn_code, key_mask, s_data):
        pass

    # --- STOP ---

    def target_on_stop(self, params):
        # this is the platform node that has to do the start
        s_data = self.msm.get_managed_service_data(params)
        if isinstance(s_data, list):
            # s_data is a list so we have to stop all the elements: this can happen
            # when we search to stop using package and type without a name
            result = self.op_replicator(params, s_data, self.stop_item)
        else:
            result = self.stop_item(params, s_data)
        return result

    def try_stop(self, rtn_code, s_data):
        return

    def stop_item(self, params, s_data):
        result = RCMPMessage()
        if s_data:
            # the params correspond to something
            rtn_code = s_data[self.PS_KEY].poll()
            if rtn_code is not None:
                # the rtn_code is 0 or 1 if the process returned
                # in this case the subprocess finished its execution and we can use communicate()
                # to obtain information: in this case we use it to flush the pipes
                # cmd_out, cmd_err = s_data[self.PS_KEY].communicate()
                # TODO in this case we have to do the ros cleanup because maybe ros remained dirty
                s_data[self.PS_KEY].communicate()
                reason = "The %s was already stopped" % self._who
                result.create_ok_response(reason)
            else:
                # the subprocess still running
                result = self.try_stop(rtn_code, s_data)
            if result.is_ok():
                # the cmd went right so we delete s_data in the managed services
                self.msm.delete_managed_service(params, s_data)
        else:
            # there is nothing corresponding to the params passed
            reason = "No %s to stop" % self._who
            result.create_ok_response(reason)
        return result

    def get_running_r_node_info(self, cmd, extractor, rrn):
        c_rrn_p = subprocess.Popen(cmd, env=self.env, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        c_rrn_p_out, c_rrn_p_err = c_rrn_p.communicate()
        # self._logger.debug("Communicate returns: csn_p_out = %s - csn_p_err = %s" % (c_rrn_p_out, c_rrn_p_err))
        rtn_code = c_rrn_p.poll()
        if rtn_code:
            # errors on subprocess std_err
            raise Exception(c_rrn_p_err)
        extractor(c_rrn_p_out, rrn)

    def extract_r_node_list(self, msg, rrn):
        # we take all the matches because in the same msg we have multiple row with
        # information with the same pattern
        # return {rn_name: None}
        m_list = self.rn_list_extractor_re.finditer(msg)
        if m_list is not None:
            for m in m_list:
                rn_name = m.group(self.RN_NAME_KEY)
                if rn_name not in rrn:
                    rrn[rn_name] = None

    def extract_r_node_list_with_address(self, msg, rrn):
        # we take all the matches because in the same msg we have multiple row with
        # information with the same pattern
        # return {rn_name: {self.RN_ADDR_KEY: rn_address}}
        m_list = self.rn_list_wa_extractor_re.finditer(msg)
        if m_list is not None:
            for m in m_list:
                rn_name = m.group(self.RN_NAME_KEY)
                if rn_name not in rrn:
                    rrn[rn_name] = {self.RN_ADDR_KEY: m.group(self.RN_ADDR_KEY)}

    # --- READ LIST ---

    def target_on_read_list(self, params):
        # this is the platform node instance that has to do the read
        result = RCMPMessage()
        nodes = []
        result.create_ok_response(nodes)
        return result

    def peep_process(self, st_e, p):
        """Peep the process output and error to flush the pipes."""
        # loops until we receive the stop_event
        while not st_e.isSet():
            # 30 second of timeout suffices because this function has only
            # to flush the stdout and stderr of the process: it can run in
            # low priority
            stop_event_is_set = st_e.wait(30)
            # it passes here at least every 30 second
            if not stop_event_is_set:
                # peep the process pipes
                if p:
                    # TODO see if we want throw an event in case of something on stderr
                    sub_p_out, sub_p_err = self.read_all_so_far(p.stdout, p.stderr)
                    if sub_p_out:
                        self._logger.debug("peeper '%s' read sub_p_out = %s" %
                                           (threading.current_thread().name, sub_p_out))
                    if sub_p_err:
                        self._logger.debug("peeper '%s' read sub_p_err = %s" %
                                           (threading.current_thread().name, sub_p_err))
                else:
                    # TODO see if we want throw an event
                    self._logger.info("process %d prematurely terminated" % p.pid)
        self._logger.debug("peeper '%s' exiting" % threading.current_thread().name)
        if p:
            # TODO see if we want throw an event in case of something on stderr
            sub_p_out, sub_p_err = self.read_all_so_far(p.stdout, p.stderr)
            if sub_p_out:
                self._logger.debug("peeper '%s' read sub_p_out = %s" %
                                   (threading.current_thread().name, sub_p_out))
            if sub_p_err:
                self._logger.debug("peeper '%s' read sub_p_err = %s" %
                                   (threading.current_thread().name, sub_p_err))
        else:
            # TODO see if we want throw an event
            self._logger.info("process %d prematurely terminated" % p.pid)
        self._logger.debug("peeper '%s' finishing" % threading.current_thread().name)


class ServiceNode(ServiceRCMPCommandAgent):
    SN_PACKAGE_KEY = "sn_package"
    SN_TYPE_KEY = "sn_type"
    SN_NAME_KEY = "sn_name"
    SN_PARAMS_KEY = "sn_params"
    SN_AUTO_GEN_NAME_KEY = "sn_auto_gen_name"
    SN_LOCAL_KEY = "sn_local"
    RCMD_PACKAGE = "rcm"
    RCMD_TYPE = "rcmdriver.py"
    RCMD_PARAM_PORT_KEY = "port"
    FIROS_PACKAGE = "firos"
    FIROS_TYPE = "core.py"
    FIROS_FAILURE = "CB_COMMUNICATION_FAILED"

    def __init__(self, pni, msm):
        ServiceRCMPCommandAgent.__init__(self, pni, msm)
        self._who = "service node"
        self.node_name_extractor_str = 'node name /%s' % self.p_name_extractor_str
        self.node_name_extractor_re = re.compile(self.node_name_extractor_str)
        self._max_retry = 5

    # --- START ---

    def prepare_start(self, params):
        # prepare the launch of a command
        cmd = ["rosrun", params[self.SN_PACKAGE_KEY], params[self.SN_TYPE_KEY]]
        # --- custom for the launch of rcm_driver
        if params[self.SN_PACKAGE_KEY] == self.RCMD_PACKAGE and params[self.SN_TYPE_KEY] == self.RCMD_TYPE:
            # custom solution to open rcm driver with a free port
            from rcmp_inter_communication import check_port
            addr = None
            while not addr:
                # we do this until we reach an available port
                addr = check_port("", 0)
            params[self.SN_PARAMS_KEY] = "%s=%s" % (self.RCMD_PARAM_PORT_KEY, str(addr[1]))
        # ---
        if self.SN_PARAMS_KEY in params:
            # sn_params are optional
            sn_params = params[self.SN_PARAMS_KEY].split()
            cmd += sn_params
        if self.SN_NAME_KEY in params:
            cmd.append("__name:=%s" % params[self.SN_NAME_KEY])
        # prepare environment
        self.env = copy.deepcopy(os.environ)
        # this is needed because the roscore target is specified in params
        self.env[self.RMASTER_URI_KEY] = str("http://%s:%s" % (params[self.SS_ADDRESS_KEY], params[self.SS_PORT_KEY]))
        return cmd

    def check_start(self, rtn_code, key_mask, s_data):
        is_startup_completed = False
        result = RCMPMessage()
        retry = 0
        errors = ""
        while rtn_code is None and not is_startup_completed:
            sub_p_out, sub_p_err = self.read_all_so_far(s_data[self.PS_KEY].stdout, s_data[self.PS_KEY].stderr,
                                                        s_data[self.PS_KEY])
            self._logger.debug("read_all_so_far returns: sub_p_out = %s - sub_p_err = %s" % (sub_p_out, sub_p_err))
            # if not self.extract_startup_info(sub_p_out, s_data):
            #     # there was nothing to extract from std output: we give other
            #     # opportunities to see if there will be something in the future
            #     # (the number of retry depends on self._max_retry)
            #     if not sub_p_err:
            #         # we take out and err simultaneously so we have to increase the counter only
            #         # if we don't have anything useful in out and anything at all on the err
            #         retry += 1
            #         self._logger.debug("nothing to extract from stdout (%d)" % retry)
            #     else:
            #         self._logger.debug("nothing to extract from stdout but sub_p_err: %s" % sub_p_err)
            if sub_p_err:
                # there was an error but the process is up: we collect all the error we encounter
                # and then put them together in the response
                errors = "%s%s" % (errors, sub_p_err)
            if key_mask[self.SN_PACKAGE_KEY] == self.FIROS_PACKAGE and \
                    key_mask[self.SN_TYPE_KEY] == self.FIROS_TYPE:
                # in firos case we have to keep looking for the FIROS_FAILURE: this error
                # is put on std_err and the process will be shut down so we have only to
                # wait if this happen
                if retry == self._max_retry:
                    is_startup_completed = True
                else:
                    # we use the retry mechanism only in case of error so that we can have all the error
                    # available on the log
                    retry += 1
            elif not errors or retry == self._max_retry:
                is_startup_completed = True
                if retry == self._max_retry:
                    self._logger.debug("Some error found: %s" % errors)
            else:
                # we use the retry mechanism only in case of error so that we can have all the error
                # available on the log
                retry += 1
            rtn_code = s_data[self.PS_KEY].poll()
        if rtn_code is not None:
            # the rtn_code is 0 or 1 if the process returned; in this case the sub process finished
            # its execution and we can use communicate() to obtain information about what went wrong
            if not errors:
                # if we don't have errors means that the error isn't taken from the std err yet
                cmd_out, errors = s_data[self.PS_KEY].communicate()
            reason = "The %s start failed with error message %s" % (self._who, errors)
            result.create_error_response(reason)
        else:
            # the sub process has done all properly so we check if the node
            # is connected to the service space
            # we always launch the check_status and if we had something on stderr (the only case
            # in which result is already set), we take the reason of the result and add it as
            # warning to the result of the check_status
            result = self.check_status(key_mask, s_data)
            if result.is_ok():
                # the cmd went right so we add s_data in the managed services
                s_data[self.PS_PEEP_STOP_KEY] = threading.Event()
                s_data[self.PS_PEEPER_KEY] = threading.Thread(name="NPeeper_%d" % s_data[self.PS_KEY].pid,
                                                              target=self.peep_process,
                                                              args=(s_data[self.PS_PEEP_STOP_KEY],
                                                                    s_data[self.PS_KEY]))
                s_data[self.PS_PEEPER_KEY].start()
                self.msm.add_managed_service(key_mask, s_data)
                reason = "The %s has started" % self._who
                result.create_ok_response(reason)
                if errors:
                    result.add_response_warning(errors)
                # TODO try to think of something more general
                if key_mask[self.SN_PACKAGE_KEY] == self.RCMD_PACKAGE and key_mask[self.SN_TYPE_KEY] == self.RCMD_TYPE:
                    # custom solution to know if this is the platform node that runs rcm driver
                    self.pni[RCMPlatformNode.PNI_RCM_DRIVER_PORT] = \
                        int(key_mask[self.SN_PARAMS_KEY].replace("%s=" % self.RCMD_PARAM_PORT_KEY, ""))
            else:
                # the process is up but at ROS level the node is not up (the check failed) so
                # to not leave the state in an inconsistent way, we kill the process
                # the nodes are to be closed using SIGINT otherwise they leave ROS dirty
                s_data[self.PS_KEY].send_signal(signal.SIGINT)
                if errors:
                    reason = "The %s started with error message %s and has been terminated after the check " \
                             "status failure" % (self._who, errors)
                else:
                    reason = "The %s started and has been terminated after the check status failure" % self._who
                result.create_error_response(reason)
        return result

    def extract_start_info(self, msg):
        # at start time of the service node when the subprocess try to
        # provide some information will be the name of the node we are
        # trying to start
        # actually the rosrun doesn't write the sys stdout but
        # uses rosout to provide the information: this means that we
        # never pass here
        ret = None
        m = self.node_name_extractor_re.search(msg)
        if m is not None:
            ret = m.group(self.P_NAME_KEY)
        return ret

    def check_status(self, key_mask, s_data):
        import time
        result = RCMPMessage()
        try:
            retry = 0
            max_check_retry = 3
            check_timeout = 2
            if self.SN_NAME_KEY in key_mask and key_mask[self.SN_NAME_KEY]:
                # we have a name so we have only to use <rosnode list> and check if the node is available
                # between the nodes in that ros
                cmd = ["rosnode", "list"]
                while retry < max_check_retry:
                    running_r_nodes = {}
                    self.get_running_r_node_info(cmd, self.extract_r_node_list, running_r_nodes)
                    if key_mask[self.SN_NAME_KEY] in running_r_nodes.keys():
                        # the node is in the list of node
                        reason = "The %s '%s' started" % (self._who, key_mask[self.SN_NAME_KEY])
                        result.create_ok_response(reason)
                        return result
                    # before retrying we wait a timeout to give time to the service node to communicate
                    # with ROS layer
                    time.sleep(check_timeout)
                    retry += 1
                # we arrive here only if it fails all retries
                reason = "The %s '%s' failed starting" % (self._who, key_mask[self.SN_NAME_KEY])
                result.create_error_response(reason)
            else:
                # we don't have a name so we have to check the pid instead the name
                cmd = ['rosnode', 'list', '-a']
                while retry < max_check_retry:
                    running_r_nodes = {}
                    self.get_running_r_node_info(cmd, self.extract_r_node_list_with_address, running_r_nodes)
                    for running_r_node in running_r_nodes.keys():
                        # if is a node running in the current machine we have to extract the pid
                        # to check if is the process that we are searching
                        if running_r_nodes[running_r_node][self.RN_ADDR_KEY] == self.pni[RCMPlatformNode.PNI_ADDRESS]:
                            # we are checking only the nodes running on this platform node instance
                            s_cmd = ['rosnode', 'info', running_r_node]
                            rrn = {running_r_node: None}
                            self.get_running_r_node_info(s_cmd, self.extract_r_node_pid, rrn)
                            if rrn[running_r_node] and s_data[self.PS_KEY].pid == int(rrn[running_r_node]):
                                # the node is in the list of node
                                key_mask[self.SN_NAME_KEY] = running_r_node
                                # the name of node is not forced by the user, but taken from ros so we set the flag
                                # to advice that we can search using only with sn_package and sn_type
                                key_mask[self.SN_AUTO_GEN_NAME_KEY] = True
                                reason = "The %s '%s' started" % (self._who, key_mask[self.SN_NAME_KEY])
                                result.create_ok_response(reason)
                                return result
                    # before retrying we wait a timeout to give time to the service node to communicate
                    # with ROS layer
                    time.sleep(check_timeout)
                    retry += 1
                # if we arrive here is because the pid of the node isn't between those on ros
                reason = "The %s failed starting" % self._who
                result.create_error_response(reason)
        except Exception as e:
            reason = "The %s start failed: %s" % (self._who, e)
            result.create_error_response(reason)
        return result

    def extract_r_node_pid(self, msg, rrn):
        # update rrn with the pid extracted from the msg
        # received rrn = {rn_name: None}
        # return {rn_name: rn_pid}
        m = self.rn_pid_extractor_re.search(msg)
        if m is not None:
            # the pid is a number, so we cast it
            rrn[rrn.keys()[0]] = int(m.group(self.RN_PID_KEY))

    # --- STOP ---

    def try_stop(self, rtn_code, s_data):
        sub_p_out, sub_p_err = None, None
        # we throw the stop event for the peeper
        s_data[self.PS_PEEP_STOP_KEY].set()
        # wait the peeper thread finish his work
        s_data[self.PS_PEEPER_KEY].join()
        # now the peeper has finished, we can stop the process
        # and take the final messages
        while rtn_code is None:
            # the nodes are to be closed using SIGINT otherwise they leave ROS dirty
            if self.pni[RCMPlatformNode.PNI_STATE] == RCMPlatformNode.DISCONNECTED_STATE and \
                    not(self.SN_LOCAL_KEY in s_data and s_data[self.SN_LOCAL_KEY]):
                # we are disconnected from the platform and the node is connected to a service space
                # that is not local: this is a special case because during the shutdown of nodes
                # ros tries to communicate with the master on the roscore (the service space) so
                # in case is unreachable freeze for 2 minutes (network timeout) before completing
                # the operation; we avoid the standard flow (ros catches only SIGINT and SIGTERM)
                # using SIGKILL; to avoid zombies we find the children of the process and terminate
                # them appropriately then kill the parent (the ros node)
                get_ps_children_pid = ['ps', '--ppid', '%d' % s_data[self.PS_KEY].pid, '-o', 'pid=']
                p = subprocess.Popen(get_ps_children_pid, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
                sub_p_out, sub_p_err = p.communicate()
                if sub_p_out:
                    pid_list = sub_p_out.split()
                    for pid in pid_list:
                        if pid.isdigit():
                            os.kill(int(pid), signal.SIGTERM)
                s_data[self.PS_KEY].kill()
            else:
                s_data[self.PS_KEY].send_signal(signal.SIGINT)
            sub_p_out, sub_p_err = s_data[self.PS_KEY].communicate()
            rtn_code = s_data[self.PS_KEY].poll()
        reason = "The %s has stopped" % self._who
        result = RCMPMessage()
        result.create_ok_response(reason)
        if sub_p_err:
            # there was an error during the exit
            result.add_response_warning(sub_p_err)
        return result

    # --- READ LIST ---

    def target_on_read_list(self, params):
        # this is the platform node instance that has to do the read
        result = RCMPMessage()
        nodes = []
        # we take the managed service relative to the specified service space
        s_data = self.msm.get_managed_service(params)
        if s_data and ServiceSpace.SN_LIST in s_data and s_data[ServiceSpace.SN_LIST]:
            for sn in s_data[ServiceSpace.SN_LIST]:
                nodes.append({self.SN_PACKAGE_KEY: sn[self.SN_PACKAGE_KEY],
                              self.SN_TYPE_KEY: sn[self.SN_TYPE_KEY],
                              self.SN_NAME_KEY: sn[self.SN_NAME_KEY],
                              self.I_NAME_KEY: self.pni[RCMPlatformNode.PNI_NAME]})
        result.create_ok_response(nodes)
        return result


class ServiceLauncher(ServiceRCMPCommandAgent):
    SL_PACKAGE_KEY = "sl_package"
    SL_F_LAUNCHER_KEY = "sl_file_launcher"
    SL_NAME_KEY = "sl_name"
    SL_PARAMS_KEY = "sl_params"
    SL_AUTO_GEN_NAME_KEY = "sl_auto_gen_name"
    A_PSS_KEY = "a_pss"
    N_NAME_KEY = "n_name"
    N_PID_KEY = "n_pid"
    N_SFLAG_KEY = "n_sflag"
    RLS_ADDR_KEY = "rls_addr"
    RM_ADDR_KEY = "rm_addr"
    RO_NAME_KEY = "ro_name"
    A_R_NODES_KEY = "associated_r_nodes"

    def __init__(self, pni, msm):
        ServiceRCMPCommandAgent.__init__(self, pni, msm)
        self._who = "service launcher"

    # --- START ---

    def prepare_start(self, params):
        # prepare the launch of a command
        cmd = ["roslaunch", params[self.SL_PACKAGE_KEY], params[self.SL_F_LAUNCHER_KEY]]
        if self.SL_PARAMS_KEY in params:
            # sl_params are optional
            sl_params = params[self.SL_PARAMS_KEY].split()
            cmd += sl_params
        # prepare environment
        self.env = copy.deepcopy(os.environ)
        # this is needed because the roscore target is specified in params
        self.env[self.RMASTER_URI_KEY] = "http://%s:%s" % (params[self.SS_ADDRESS_KEY], params[self.SS_PORT_KEY])
        return cmd

    def check_start(self, rtn_code, key_mask, s_data):
        is_startup_completed = False
        result = RCMPMessage()
        # we start from -1 because we use retry as an indicator of many times we retry
        # after the first boot sequence conclusion so the first conclusion will be the
        # retry = 0
        retry = -1
        errors = ""
        while rtn_code is None and not is_startup_completed:
            sub_p_out, sub_p_err = self.read_all_so_far(s_data[self.PS_KEY].stdout, s_data[self.PS_KEY].stderr,
                                                        s_data[self.PS_KEY])
            # self._logger.debug("read_all_so_far returns: sub_p_out = %s - sub_p_err = %s" % (sub_p_out, sub_p_err))
            if not self.extract_startup_info(sub_p_out, s_data):
                # there was nothing to extract from std output: we give other
                # opportunities to see if there will be something in the future
                # (the number of retry depends on self._max_retry)
                if not sub_p_err:
                    # we take out and err simultaneously so we have to increase the counter only
                    # if we don't have anything useful in out and anything at all on the err
                    retry += 1
            if sub_p_err:
                # there was an error but the process is up: we collect all the error we encounter
                # and then put them together in the response
                errors = "%s%s" % (errors, sub_p_err)
                self._logger.debug("Some error found: %s" % errors)
            if retry == self._max_retry:
                is_startup_completed = True
            else:
                # if we have to do a retry we decrease the timeout used in the read_all_so_far
                # so that we don't wait as much as before if there isn't nothing
                self._timeout = 2
            rtn_code = s_data[self.PS_KEY].poll()
        if rtn_code is not None:
            # the rtn_code is 0 or 1 if the process returned; in this case the sub process finished
            # its execution and we can use communicate() to obtain information about what went wrong
            if not errors:
                # if we don't have errors means that the error isn't taken from the std err yet
                cmd_out, errors = s_data[self.PS_KEY].communicate()
            reason = "The %s start failed with error message %s" % (self._who, errors)
            result.create_error_response(reason)
        else:
            # we consider the launcher always started and if we had something on stderr
            # (the only case in which result is already set), we take the reason of the result
            # and add it as warning to the positive result
            if self.A_PSS_KEY in s_data:
                started_ps = []
                for s_ps in s_data[self.A_PSS_KEY]:
                    started_ps.append(s_ps[self.N_NAME_KEY])
                reason = "The %s has started with %s" % (self._who, started_ps)
            else:
                reason = "The %s has started" % self._who
            s_data[self.PS_PEEP_STOP_KEY] = threading.Event()
            s_data[self.PS_PEEPER_KEY] = threading.Thread(name="LPeeper_%d" % s_data[self.PS_KEY].pid,
                                                          target=self.peep_process,
                                                          args=(s_data[self.PS_PEEP_STOP_KEY],
                                                                s_data[self.PS_KEY]))
            s_data[self.PS_PEEPER_KEY].start()
            self.msm.add_managed_service(key_mask, s_data)
            result.create_ok_response(reason)
            if errors:
                result.add_response_warning(errors)
        return result

    def extract_startup_info(self, msg, s_data):
        # we try to retrieve information about what the start is doing; we have 2
        # types of situations, the standard situation when the service launcher
        # starts without any additional node (simple service space launch) and more
        # complex situation in which the service launcher starts additional nodes
        # (service space launch or service launcher start): in the simple case the
        # last information we receive should be rosout; in the complex one we have
        # the launch of some nodes after rosout
        if not msg:
            # nothing to extract from
            ret = False
        else:
            # we take all the matches because in the same msg we can have multiple
            # startup information
            m_list = self.startup_p_info_extractor_re.finditer(msg)
            if m_list is not None:
                for m in m_list:
                    if self.A_PSS_KEY not in s_data:
                        # not associated processes available so we initialize them
                        s_data[self.A_PSS_KEY] = []
                    s_data[self.A_PSS_KEY].append(
                        {self.N_NAME_KEY: m.group(self.P_NAME_KEY), self.N_PID_KEY: m.group(self.P_PID_KEY)})
                    self._logger.info("The subprocess started %s with pid %s" % (
                        s_data[self.A_PSS_KEY][-1][self.N_NAME_KEY], s_data[self.A_PSS_KEY][-1][self.N_PID_KEY]))
            m = self.roslaunch_s_addr_extractor_re.search(msg)
            if m is not None:
                s_data[self.RLS_ADDR_KEY] = m.group(self.RL_S_ADDR_KEY)
                self._logger.info("The subprocess started the roslaunch server at %s" % s_data[self.RLS_ADDR_KEY])
            m = self.rosmaster_addr_extractor_re.search(msg)
            if m is not None:
                s_data[self.RM_ADDR_KEY] = m.group(self.RM_ADDRESS_KEY)
                self._logger.info("The subprocess started or uses rosmaster at %s" % s_data[self.RM_ADDR_KEY])
            m = self.rosout_name_extractor_re.search(msg)
            if m is not None:
                s_data[self.RO_NAME_KEY] = m.group(self.ROUT_NAME_KEY)
                self._logger.info("The subprocess started or uses output with name %s" % s_data[self.RO_NAME_KEY])
                # this is the last step of the ros boot: after this step
                # the launcher can only add new nodes
            ret = True
        return ret

    # --- STOP ---

    def try_stop(self, rtn_code, s_data):
        sub_p_out, sub_p_err = None, None
        # we throw the stop event for the peeper
        s_data[self.PS_PEEP_STOP_KEY].set()
        # wait the peeper thread finish his work
        s_data[self.PS_PEEPER_KEY].join()
        # now the peeper has finished, we can stop the process
        # and take the final messages
        while rtn_code is None:
            # the launchers are to be closed using SIGINT otherwise they leave ROS dirty
            s_data[self.PS_KEY].send_signal(signal.SIGINT)
            sub_p_out, sub_p_err = s_data[self.PS_KEY].communicate()
            rtn_code = s_data[self.PS_KEY].poll()
        # self._logger.debug("communicate returns: sub_p_out = %s - sub_p_err = %s" % (sub_p_out, sub_p_err))
        self.extract_shutdown_info(sub_p_out, s_data)
        if self.A_PSS_KEY in s_data:
            # if at the start the information about what was launched are
            # missing, we can match them with that we can receive at this
            # point, so we can do it
            stopped_ps = []
            for s_ps in s_data[self.A_PSS_KEY]:
                if self.N_SFLAG_KEY in s_ps and s_ps[self.N_SFLAG_KEY]:
                    stopped_ps.append(s_ps[self.N_NAME_KEY])
            reason = "The %s has stopped with %s" % (self._who, stopped_ps)
        else:
            reason = "The %s has stopped" % self._who
        result = RCMPMessage()
        result.create_ok_response(reason)
        if sub_p_err:
            # there was an error during the exit
            result.add_response_warning(sub_p_err)
        return result

    def extract_shutdown_info(self, msg, s_data):
        # at this moment the shutdown went well so this is only to retrieve
        # much information about what the shutdown has done
        ret = False
        # we take all the matches because msg contains all the final messages
        m_list = self.shutdown_p_name_extractor_re.finditer(msg)
        if m_list is not None:
            for m in m_list:
                if self.A_PSS_KEY in s_data:
                    # if at the start the information about what was launched are
                    # missing, we can match them with that we can receive at this
                    # point, so we can do it
                    for n in s_data[self.A_PSS_KEY]:
                        if n[self.N_NAME_KEY] == m.group(self.P_NAME_KEY):
                            # in case a node has been stopped we set the stop flag
                            n[self.N_SFLAG_KEY] = True
        if self.shutdown_seq_str in msg and self.shutdown_seq_end_str in msg:
            ret = True
        return ret

    # --- READ LIST ---

    def target_on_read_list(self, params):
        # this is the platform node instance that has to do the read
        result = RCMPMessage()
        launchers = []
        # we take the managed service relative to the specified service space
        s_data = self.msm.get_managed_service(params)
        if s_data and ServiceSpace.SL_LIST in s_data and s_data[ServiceSpace.SL_LIST]:
            for sl in s_data[ServiceSpace.SL_LIST]:
                # here we need only the names of the nodes managed by the launcher
                a_r_nodes = []
                if self.A_PSS_KEY in sl and sl[self.A_PSS_KEY]:
                    for rn in sl[self.A_PSS_KEY]:
                        if self.N_NAME_KEY in rn and rn[self.N_NAME_KEY]:
                            a_r_nodes.append(rn[self.N_NAME_KEY])
                launchers.append({self.SL_PACKAGE_KEY: sl[self.SL_PACKAGE_KEY],
                                  self.SL_F_LAUNCHER_KEY: sl[self.SL_F_LAUNCHER_KEY],
                                  self.SL_NAME_KEY: sl[self.SL_NAME_KEY],
                                  self.A_R_NODES_KEY: a_r_nodes,
                                  self.I_NAME_KEY: self.pni[RCMPlatformNode.PNI_NAME]})
        result.create_ok_response(launchers)
        return result


class ServiceSpace(ServiceLauncher):
    SS_OWNED = "ss_owned"
    SN_LIST = "sn_list"
    SL_LIST = "sl_list"
    RUNNING_LAUNCHERS = "r_launchers"
    RUNNING_NODES = "r_nodes"
    PUBLIC_IP = "public_ip"
    INBOUND_PORT = "inbound_port"
    GET_PUBLIC_IP_WS_ADDRESS = "ip.42.pl/raw"

    def __init__(self, pni, msm):
        ServiceLauncher.__init__(self, pni, msm)
        # we overwrite the self._who in ServiceLauncher
        self._who = "service space"
        # this is used to give more chance to find other info after the boot
        # but in case of service space we don't need
        self._max_retry = 0

    # --- API ACCESSIBLE FROM HANDLER ---

    def create(self, params):
        """Create service spaces."""
        self._logger.info("%s called" % params)
        result = RCMPMessage()
        try:
            if self.pni[RCMPlatformNode.PNI_IS_MASTER]:
                # the master is the only one platform node that can access to the robotics
                # data and has to take from the database the needed information to perform
                # the real request: we create the entry before knowing the result of the
                # operation because so shouldn't happen that another creation on the same
                # instance but a different service space name can steal the port (avoiding
                # inconsistency between ss_name and port); if we have an error during the
                # create we don't have to delete the entry because this means that it didn't
                # succeed doing that (there was another entry with that name and must remain
                # there or nothing has been written)
                self.create_ss_entry(params)
                try:
                    # to create a service space we have to launch it
                    result = self.start(params)
                finally:
                    # only if there is an error during the start we have to contact robotics
                    # data to notify that the creation failed and we have to rollback the
                    # creation of the entry of the service space
                    if not result.is_ok():
                        # the real creation failed
                        try:
                            self.delete_ss_entry(params)
                        except IOError as ioe:
                            self._logger.warning(ioe)
            else:
                # the only part that must be done at a not master side is the start
                # launch
                result = self.start(params)
        except Exception as e:
            reason = "The %s create failed: %s" % (self._who, e)
            result.create_error_response(reason)
        self._logger.info(result.get_txt())
        return result

    def delete(self, params):
        """Delete service spaces."""
        self._logger.info("%s called" % params)
        result = RCMPMessage()
        try:
            res = self.pre_read(params)
            if res is None:
                result = self.delete_one(params)
            else:
                result = self.op_replicator(params, res, self.delete_one)
        except Exception as e:
            reason = "The %s delete failed: %s" % (self._who, e)
            result.create_error_response(reason)
        # TODO this is already in delete_one
        # finally:
        #     if self.pni[RCMPlatformNode.PNI_IS_MASTER]:
        #         # self.master_on_delete_finally(result, params)
        #         # the master is the only one platform node that can access to the
        #         # robotics data and so if the deletion went well it has to rollback
        #         # the creation of the entry of the service space
        #         if result.is_ok():
        #             # the deletion went well
        #             try:
        #                 self.delete_ss_entry(params)
        #             except IOError as ioe:
        #                 self._logger.warning(ioe)
        #                 # we add at the response the info about the failure completing
        #                 # the command
        #                 result.add_response_warning(ioe)
        self._logger.info(result.get_txt())
        return result

    def delete_all_local(self, params):
        try:
            self._logger.info("%s called" % params)
            result = self.execute(operation=self.target_on_delete_all_local, params=params)
        except Exception as e:
            reason = "The delete all %ss failed: %s" % (self._who, e)
            result = RCMPMessage()
            result.create_error_response(reason)
        self._logger.info(result.get_txt())
        return result

    def delete_local(self, params):
        try:
            self._logger.info("%s called" % params)
            result = self.execute(operation=self.target_on_delete_local, params=params)
        except Exception as e:
            reason = "The delete all %ss failed: %s" % (self._who, e)
            result = RCMPMessage()
            result.create_error_response(reason)
        self._logger.info(result.get_txt())
        return result

    def read_list(self, params):
        """Read all the service spaces on a RCM platform node instance."""
        try:
            self._logger.info("%s called" % params)
            result = self.execute(m_operation=self.target_on_read_list, params=params)
        except Exception as e:
            reason = "The read of the %ss failed: %s" % (self._who, e)
            result = RCMPMessage()
            result.create_error_response(reason)
        self._logger.info(result.get_txt())
        return result

    def read(self, params):
        """Read a service space."""
        try:
            self._logger.info("%s called" % params)
            res = self.pre_read(params)
            if res is None:
                result = self.read_one(params)
            else:
                result = self.op_replicator(params, res, self.read_one)
        except Exception as e:
            reason = "The %s read failed: %s" % (self._who, e)
            result = RCMPMessage()
            result.create_error_response(reason)
        self._logger.info(result.get_txt())
        return result

    def read_public_ip(self, params):
        """Read the public ip of the service space."""
        try:
            self._logger.info("%s called" % params)
            result = self.execute(operation=self.target_on_read_public_ip, params=params)
        except Exception as e:
            reason = "The %s read of public ip failed: %s" % (self._who, e)
            result = RCMPMessage()
            result.create_error_response(reason)
        self._logger.info(result.get_txt())
        return result

    # --- GENERAL PURPOSE ---

    def create_ss_entry(self, params):
        from rcmp_robotics_data import execute_int_robotics_data_query
        if self.SS_NAME_KEY not in params:
            raise SyntaxError("%s is missing in the request" % self.SS_NAME_KEY)
        if self.I_NAME_KEY not in params:
            raise SyntaxError("%s is missing in the request" % self.I_NAME_KEY)
        return execute_int_robotics_data_query(self.create_ss, params,
                                               "Unable to provide a port for the %s" % self._who)

    def create_ss(self, rdm, params):
        row = rdm.get_ss_ip_address_port(params[self.SS_NAME_KEY])
        if row:
            # the name is already used
            reason = "The name '%s' is already used by another %s" % \
                     (params[self.SS_NAME_KEY], self._who)
            raise IOError(reason)
        row = rdm.get_pi_ip_address_from_name(params[self.I_NAME_KEY])
        if row and row[0]:
            params[self.I_ADDRESS_KEY] = row[0]
            # at this point we don't need the self.I_NAME_KEY anymore
            del params[self.I_NAME_KEY]
        else:
            raise ValueError("'%s' doesn't exist" % params[self.I_NAME_KEY])
        # create a new entry in the service space table
        new_port = rdm.insert_service_space_with_next_port(params[self.SS_NAME_KEY],
                                                           params[self.I_ADDRESS_KEY])
        # at this point we don't need the parameter associated with self.SS_NAME_KEY
        # del params[self.SS_NAME_KEY]
        params[self.SS_ADDRESS_KEY] = params[self.I_ADDRESS_KEY]
        # at db level port is an int meanwhile here we have it as string
        # so we have to cast
        params[self.SS_PORT_KEY] = str(new_port)

    def delete_ss_entry(self, params):
        from rcmp_robotics_data import execute_int_robotics_data_query
        if (self.SS_ADDRESS_KEY in params and params[self.SS_ADDRESS_KEY]) and \
                (self.SS_PORT_KEY in params and params[self.SS_PORT_KEY]):
            # self.SS_ADDRESS_KEY and self.SS_PORT_KEY are not in params only
            # when the creation could not generate a new entry: in that case there
            # is nothing to delete anyway
            return execute_int_robotics_data_query(self.delete_ss, params,
                                                   "Unable to delete the %s entry" % self._who)

    def delete_ss(self, rdm, params):
        # delete the entry in the service space table
        # at db level port is an int meanwhile here we have it as string
        # so we have to cast
        rdm.delete_service_space(params[self.SS_ADDRESS_KEY], int(params[self.SS_PORT_KEY]))

    def pre_read(self, params):
        """Get the service space list. This is a preparation step for read and delete and returns a list object."""
        if self.pni[RCMPlatformNode.PNI_IS_MASTER]:
            # this pre process of params is done only in the master
            if self.I_NAME_KEY in params and params[self.I_NAME_KEY] and \
                    not (self.I_ADDRESS_KEY in params and params[self.I_ADDRESS_KEY]):
                self.get_i_entry(params)
            # params[self.I_ADDRESS_KEY] couldn't be set because the user didn't put
            # self.I_NAME_KEY in the request
            if self.SS_NAME_KEY in params and params[self.SS_NAME_KEY] and \
                    not (self.SS_ADDRESS_KEY and self.SS_PORT_KEY in params and
                         params[self.SS_ADDRESS_KEY] and params[self.SS_PORT_KEY]):
                self.get_ss_entry(params)
                # now we should have SS_ADDRESS_KEY and self.SS_PORT_KEY
                if self.SS_ADDRESS_KEY in params and params[self.SS_ADDRESS_KEY]:
                    if self.I_ADDRESS_KEY in params and params[self.I_ADDRESS_KEY]:
                        if params[self.I_ADDRESS_KEY] != params[self.SS_ADDRESS_KEY]:
                            # additional check on the user params: the service space exists
                            # but the user has specified the wrong platform node where it runs on
                            raise ValueError("The %s '%s' is not running in the platform node '%s'." %
                                             (self._who, params[self.SS_NAME_KEY], params[self.I_NAME_KEY]))
                        # --- missing else case here ---
                    else:
                        params[self.I_ADDRESS_KEY] = params[self.SS_ADDRESS_KEY]
                else:
                    raise ValueError("The %s '%s' is not available." % (self._who, params[self.SS_NAME_KEY]))
            elif not (self.SS_ADDRESS_KEY and self.SS_PORT_KEY in params and
                      params[self.SS_ADDRESS_KEY] and params[self.SS_PORT_KEY]):
                # this is the case in which the user didn't put self.SS_NAME_KEY in the
                # request
                if self.I_ADDRESS_KEY in params and params[self.I_ADDRESS_KEY]:
                    # in this case we read all the service spaces in the self.I_ADDRESS to do
                    # the operation on them
                    result = self.read_list(params)
                    if result.is_ok():
                        result.get_response_reason()
                        return result
                    else:
                        # in case of error during the read_list we raise an exception with
                        # that reason
                        raise ValueError(result.get_response_reason())
                else:
                    raise ValueError("No parameters specified.")
        return None

    def op_replicator(self, params, ss_list, operation):
        """Replicate a specified operation on a list of service spaces."""
        # this is a specific version for the service spaces and overrides the generic version:
        # the main differences are the params changes and the operation parameters (the specified
        # operation doesn't need the service item)
        failed = False
        result_list = []
        for ss in ss_list:
            if ss:
                params[self.SS_NAME_KEY] = ss
                res = operation(params)
                result_list.append(res)
                if not res.is_ok:
                    # if even one operation returns an error the ss_multiplier will return an error
                    failed = True
                # reset the params used for the operation
                if self.SS_ADDRESS_KEY in params:
                    del params[self.SS_ADDRESS_KEY]
                # if self.SS_ADDRESS_KEY in params:
                if self.SS_PORT_KEY in params:
                    del params[self.SS_PORT_KEY]
                # if self.SS_ADDRESS_KEY in params:
                if self.SS_NAME_KEY in params:
                    del params[self.SS_NAME_KEY]
        result = RCMPMessage()
        if failed:
            result.create_error_response(result_list)
        else:
            result.create_ok_response(result_list)
        return result

    def local_op_replicator(self, params, ms_ss_list, operation):
        """Replicate a specified operation on a list of service spaces found locally in the data structure."""
        failed = False
        result_list = []
        for ms_ss in ms_ss_list:
            params[self.SS_ADDRESS_KEY] = ms_ss[self.SS_ADDRESS_KEY]
            params[self.SS_PORT_KEY] = ms_ss[self.SS_PORT_KEY]
            res = operation(params)
            result_list.append(res.get_dict())
            if not res.is_ok:
                # if even one operation returns an error the ss_multiplier will return an error
                failed = True
        result = RCMPMessage()
        if failed:
            result.create_error_response(result_list)
        else:
            result.create_ok_response(result_list)
        return result

    # --- START ---

    def prepare_start(self, params):
        # prepare the launch of a command
        # this is needed because the port is passed by the master and can be any:
        # if you only change the environment variable roscore uses the default port
        # instead the specified one and write on std_err that the port used doesn't
        # match the one in the environment; if you only change the -p parameter
        # roscore uses that port correctly but write on std_err that the port used
        # doesn't match the one in the environment
        cmd = ['roscore', '-p', '%s' % params[self.SS_PORT_KEY]]
        self.env = copy.deepcopy(os.environ)
        self.env[self.RMASTER_URI_KEY] = "http://%s:%s" % (params[self.SS_ADDRESS_KEY], params[self.SS_PORT_KEY])
        return cmd

    # --- DELETE ---

    def delete_one(self, params):
        """Delete a specified service space."""
        result = RCMPMessage()
        try:
            # the delete is the stop of the service space plus
            # the delete of the ss entry in the db on the master
            result = self.stop(params)
        finally:
            if self.pni[RCMPlatformNode.PNI_IS_MASTER]:
                # the master is the only one platform node that can access to the
                # robotics data and so if the deletion went well it has to rollback
                # the creation of the entry of the service space
                if result.is_ok():
                    # the deletion went well
                    self.delete_ss_entry(params)
        return result

    # --- DELETE ALL LOCAL ---

    def target_on_delete_all_local(self, params):
        # this is the platform node that has to do the start
        s_data = self.msm.get_managed_service_data(params)
        if s_data:
            if isinstance(s_data, list):
                # s_data is a list so we have to stop all the elements: this can happen
                # when we search to stop using package and type without a name
                result = self.local_op_replicator(params, s_data, self.target_on_delete_local)
            else:
                params[self.SS_ADDRESS_KEY] = s_data[self.SS_ADDRESS_KEY]
                params[self.SS_PORT_KEY] = s_data[self.SS_PORT_KEY]
                result = self.target_on_delete_local(params)
        else:
            # there is nothing to delete
            result = RCMPMessage()
            result.create_ok_response("Nothing local to delete")
        return result

    # --- DELETE LOCAL ---

    def target_on_delete_local(self, params):
        from rcmp_command import RCMPCommandHandler, KILL_ALL_L_SERVICE_NODE, KILL_ALL_L_SERVICE_LAUNCHER, \
            DELETE_SERVICE_SPACE
        failed = False
        result_list = []
        l_msg_dict = {RCMPCommandHandler.COMMAND_KEY: KILL_ALL_L_SERVICE_LAUNCHER,
                      self.I_ADDRESS_KEY: params[self.I_ADDRESS_KEY],
                      self.SS_ADDRESS_KEY: params[self.SS_ADDRESS_KEY],
                      self.SS_PORT_KEY: params[self.SS_PORT_KEY]}
        # instead of using the usual way (through socket call), here we call directly the object to perform
        # the operation: in this way the command could require less time to do the operation and success
        # before the socket timeout forces the failure of the original command
        s_launcher = ServiceLauncher(self.pni, self.msm)
        res = s_launcher.stop_all_local(l_msg_dict)
        if not res.is_ok:
            # if even one operation returns an error the ss_multiplier will return an error
            failed = True
        result_list.append(res.get_dict())
        n_msg_dict = {RCMPCommandHandler.COMMAND_KEY: KILL_ALL_L_SERVICE_NODE,
                      self.I_ADDRESS_KEY: params[self.I_ADDRESS_KEY],
                      self.SS_ADDRESS_KEY: params[self.SS_ADDRESS_KEY],
                      self.SS_PORT_KEY: params[self.SS_PORT_KEY]}
        # instead of using the usual way (through socket call), here we call directly the object to perform
        # the operation: in this way the command could require less time to do the operation and success
        # before the socket timeout forces the failure of the original command
        s_node = ServiceNode(self.pni, self.msm)
        res = s_node.stop_all_local(n_msg_dict)
        if not res.is_ok:
            # if even one operation returns an error the ss_multiplier will return an error
            failed = True
        result_list.append(res.get_dict())
        result = RCMPMessage()
        if not failed:
            ss_deleted = False
            if params[self.SS_ADDRESS_KEY] == params[self.I_ADDRESS_KEY]:
                # this is a local service space, so we have to delete completely
                cmd = {RCMPCommandHandler.COMMAND_KEY: DELETE_SERVICE_SPACE,
                       self.I_ADDRESS_KEY: params[self.I_ADDRESS_KEY],
                       self.SS_ADDRESS_KEY: params[self.SS_ADDRESS_KEY],
                       self.SS_PORT_KEY: params[self.SS_PORT_KEY]}
                # this function in case of master perform the deletion of the entry too
                res = self.delete_one(cmd)
                if res.is_ok():
                    result_list.append(res.get_dict())
                    ss_deleted = True
                else:
                    failed = False
            if not failed:
                result.create_ok_response(result_list)
                if self.pni[RCMPlatformNode.PNI_IS_MASTER]:
                    # when we are on the master the delete local service space has to perform
                    # the cleanup of the service space table if it isn't already done (not
                    # local service spaces)
                    if not ss_deleted:
                        # we haven't done the complete deletion of the service space, so we
                        # remove the entry in the robotics data
                        self.delete_ss_entry(params)
            else:
                result.create_error_response(result_list)
        else:
            result.create_error_response(result_list)
        return result

    # --- READ LIST ---

    def target_on_read_list(self, params):
        from rcmp_robotics_data import execute_robotics_data_query
        if self.I_NAME_KEY not in params:
            raise SyntaxError("%s is missing in the request" % self.I_NAME_KEY)
        return execute_robotics_data_query(self.get_ss_4_r_list, params,
                                           err_reason="The read of the list of %ss failed" % self._who)

    def get_ss_4_r_list(self, rdm, params):
        # Take I_NAME_KEY from params and get all the service spaces running on that RCM
        # platform node instance. Add I_ADDRESS_KEY in params with the ip address of this
        # instance
        result = RCMPMessage()
        row = rdm.get_pi_ip_address_from_name(params[self.I_NAME_KEY])
        if row and row[0]:
            params[self.I_ADDRESS_KEY] = row[0]
        else:
            raise ValueError("'%s' doesn't exist" % params[self.I_NAME_KEY])
        rows = rdm.get_service_space_list_from_pi(params[self.I_NAME_KEY])
        ss_list = []
        if rows:
            for row in rows:
                if row:
                    ss_list.append({ServiceSpace.SS_NAME_KEY: row[0]})
        result.create_ok_response(ss_list)
        return result

    # --- READ ---

    def read_one(self, params):
        """Read the information about the service space."""
        try:
            result = self.execute(m_operation=self.get_ss_entry, operation=self.target_on_read_one, params=params)
        except Exception as e:
            reason = "The %s read failed: %s" % (self._who, e)
            result = RCMPMessage()
            result.create_error_response(reason)
        return result

    def target_on_read_one(self, params):
        result = RCMPMessage()
        # we prepare the environment variable to point to the right service space:
        # this is common for all the following commands we launch from here on
        self.env = copy.deepcopy(os.environ)
        self.env[self.RMASTER_URI_KEY] = "http://%s:%s" % (params[self.SS_ADDRESS_KEY],
                                                           params[self.SS_PORT_KEY])
        # first: get running ros nodes
        cmd = ['rosnode', 'list', '-a']
        running_r_nodes = {}
        self.get_running_r_node_info(cmd, self.extract_r_node_list_with_address, running_r_nodes)
        rn_address_list = []
        for running_r_node in running_r_nodes.keys():
            # we collect the addresses where the nodes are running to contact the RCM
            # platform node and receive the missing information
            rn_address_list.append(running_r_nodes[running_r_node][self.RN_ADDR_KEY])
        ret = self.read_service_launchers_and_nodes(rn_address_list, params)
        # self.add_r_nodes_additional_info(ret)
        result.create_ok_response(ret)
        return result

    def read_service_launchers_and_nodes(self, rn_address_list, params):
        """Take a list of addresses where nodes attached to the service space we are analyzing
        are running and contact the RCM platform instance node to read all the information
        available for those nodes."""
        from rcmp_command import RCMPCommandHandler, READ_SERVICE_NODE, READ_SERVICE_LAUNCHER, \
            READ_P_IP_SERVICE_SPACE
        l_msg_dict = {RCMPCommandHandler.COMMAND_KEY: READ_SERVICE_LAUNCHER,
                      self.SS_ADDRESS_KEY: params[self.SS_ADDRESS_KEY],
                      self.SS_PORT_KEY: params[self.SS_PORT_KEY]}
        n_msg_dict = {RCMPCommandHandler.COMMAND_KEY: READ_SERVICE_NODE,
                      self.SS_ADDRESS_KEY: params[self.SS_ADDRESS_KEY],
                      self.SS_PORT_KEY: params[self.SS_PORT_KEY]}
        # using set we avoid duplicates
        rn_addresses = set(rn_address_list)
        res = {self.RUNNING_LAUNCHERS: [], self.RUNNING_NODES: [],
               self.INBOUND_PORT: params[self.SS_EXT_PORT_KEY]
               if self.SS_EXT_PORT_KEY in params and params[self.SS_EXT_PORT_KEY] else "",
               self.PUBLIC_IP: ""}
        # TODO this is the public ip of the service space, so if you run a node in a place different
        # than the one where the service space runs and the node need to know the public address
        # this doesn't work
        if self.SS_EXT_PORT_KEY in params and params[self.SS_EXT_PORT_KEY]:
            # we return the public ip address only in case we have an external port
            s_msg_dict = {RCMPCommandHandler.COMMAND_KEY: READ_P_IP_SERVICE_SPACE,
                          self.SS_ADDRESS_KEY: params[self.SS_ADDRESS_KEY],
                          self.SS_PORT_KEY: params[self.SS_PORT_KEY]}
            r = self.read_public_ip(s_msg_dict)
            if r.is_ok():
                res[self.PUBLIC_IP] = r.get_response_reason()
        s_node = ServiceNode(self.pni, self.msm)
        s_launcher = ServiceLauncher(self.pni, self.msm)
        while rn_addresses:
            l_msg_dict[self.I_ADDRESS_KEY] = n_msg_dict[self.I_ADDRESS_KEY] = rn_addresses.pop()
            # get the launchers
            result = s_launcher.read_list(l_msg_dict)
            if result.is_ok():
                res[self.RUNNING_LAUNCHERS] += result.get_response_reason()
            else:
                # in case of error during the read we put the error reason in the list of launchers
                res[self.RUNNING_LAUNCHERS].append(result.get_response_reason())
            # get the nodes
            result = s_node.read_list(n_msg_dict)
            if result.is_ok():
                res[self.RUNNING_NODES] += result.get_response_reason()
            else:
                # in case of error during the read we put the error reason in the list of launchers
                res[self.RUNNING_NODES].append(result.get_response_reason())
        return res

    def add_r_nodes_additional_info(self, running_s_nodes):
        """Take the list of service launchers and nodes and use the node names to contact ROS
        platform to have additional information"""
        # running_s_nodes = {self.RUNNING_LAUNCHERS: [], self.RUNNING_NODES: []}
        # second: get running ros node info from ros
        if running_s_nodes[self.RUNNING_LAUNCHERS]:
            for running_s_node in running_s_nodes[self.RUNNING_LAUNCHERS]:
                if running_s_node and ServiceLauncher.A_R_NODES_KEY in running_s_node:
                    for ass_r_node in running_s_node[ServiceLauncher.A_R_NODES_KEY]:
                        cmd = ['rosnode', 'info', ass_r_node]
                        rrn = {ass_r_node}
                        self.get_running_r_node_info(cmd, self.extract_r_node_info, rrn)
        if running_s_nodes[self.RUNNING_NODES]:
            for running_s_node in running_s_nodes[self.RUNNING_NODES]:
                if running_s_node and ServiceNode.SN_NAME_KEY in running_s_node:
                    cmd = ['rosnode', 'info', running_s_node[ServiceNode.SN_NAME_KEY]]
                    rrn = {running_s_node[ServiceNode.SN_NAME_KEY]}
                    self.get_running_r_node_info(cmd, self.extract_r_node_info, rrn)

    def extract_r_node_info(self, msg, rrn):
        # TODO not implemented yet
        return []

    # --- READ PUBLIC IP ---

    def target_on_read_public_ip(self, params):
        import urllib2
        base_url = "http://%s" % self.GET_PUBLIC_IP_WS_ADDRESS
        public_ip = ""
        try:
            response = urllib2.urlopen(base_url)
            if response.getcode() == 200:
                public_ip = response.read()
        except:
            # we don't do anything and the public_ip will be ""
            pass
        finally:
            result = RCMPMessage()
            result.create_ok_response(public_ip)
