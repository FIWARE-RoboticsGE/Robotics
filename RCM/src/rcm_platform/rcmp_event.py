__author__ = "Fabio Giuseppe Di Benedetto"

from rcmp_node import logging, RCMP_LOGGER_NAME, threading, RCMPlatformNode
from rcmp_inter_communication import RCMPInterCommunicationClient, RCMPMessage

PAIRING_EVENT = 'PGE'
UPDATE_EVENT = 'UE'
PAIRED_EVENT = 'PDE'
ROLLBACK_UPDATE_EVENT = 'RUE'
ROLLBACK_UPDATE_ALL_EVENT = 'RUAE'
# CLEAN_UP_EVENT = 'CUE'
STOP_WD_EVENT = 'SWE'
STOP_ALL_WD_EVENT = 'SAWE'


class RCMPEventAgent:
    """The generic agent that execute the event."""

    I_ADDRESS_KEY = "i_address"
    I_NAME_KEY = "i_name"

    def __init__(self, pni):
        self._logger = logging.getLogger(RCMP_LOGGER_NAME)
        # this is for generic return messages
        self._who = ""
        self.pni = pni

    # def call(self, params):
    #     """Call the internal server to notify an event to the RCM platform node instance specified in the request.
    #     Return True if the event reach the target."""
    #     c = RCMPInterCommunicationClient(address=self.pni[RCMPlatformNode.PNI_ADDRESS])
    #     try:
    #         c.connect()
    #         req = RCMPMessage(params)
    #         c.send(req.get_txt())
    #         res = c.receive()
    #         if res:
    #             result = RCMPMessage()
    #             result.set_content(res)
    #             return result.is_ok()
    #         else:
    #             self._logger.error("%s received no response" % req.get_txt())
    #     finally:
    #         c.close()
    #     # if we arrive here the call failed
    #     return False

    def notify(self, params):
        """Notify an event to the RCM platform node instance specified in the request. Return an RCMPMessage."""
        result = RCMPMessage()
        self._logger.debug("event notify with params: %s" % params)
        # the events are asynchronous so we don't need a timeout to wait the response
        c = RCMPInterCommunicationClient(params[self.I_ADDRESS_KEY], t_out=2.0)
        try:
            c.connect()
            # propagate the request as it is
            c.send(RCMPMessage(params).get_txt())
            # take the result of the operation
            # res = c.receive()
            res = c.receive_all_so_far()
            if res:
                result.set_content(res)
            else:
                reason = "No response received"
                result.create_error_response(reason)
        except Exception as e:
            reason = "Error notifying the event to the RCM platform node at %s: %s" % (params[self.I_ADDRESS_KEY], e)
            self._logger.error(reason)
            result.create_error_response(reason)
        finally:
            try:
                c.close()
            except Exception as e:
                # this exception doesn't change the result but we want to trace in the log
                self._logger.warning("Error closing the notifying client: %s" % e)
        return result

    def execute(self, operation=None, params=None):
        """Execute the required operation with the parameters passed. Return an RCMPMessage."""
        result = RCMPMessage()
        if operation:
            if params and self.I_ADDRESS_KEY in params and params[self.I_ADDRESS_KEY] and \
               params[self.I_ADDRESS_KEY] != self.pni[RCMPlatformNode.PNI_ADDRESS]:
                # perform the delegation to another platform node because the target
                # is not the current one
                result = self.notify(params)
            else:
                # perform the operation (the target is the current platform node)
                result = operation(params)
        return result


class PNodeInstance(RCMPEventAgent):
    PI_NAME_KEY = "pi_name"
    PI_ADDRESS_KEY = "pi_address"
    PI_S_NAME_KEY = "pi_s_name"
    PI_S_ADDRESS_KEY = "pi_s_address"
    PI_R_NAME_KEY = "pi_r_name"
    PI_R_ADDRESS_KEY = "pi_r_address"
    PI_REACHABLE_KEY = "pi_reachable"
    PI_SOURCE_ADDRESS_KEY = "pi_source_address"

    def __init__(self, pni, wdm, msm):
        RCMPEventAgent.__init__(self, pni)
        self._who = "platform node instance"
        self.wdm = wdm
        self.msm = msm

    def pairing(self, params):
        """Notify a platform node that has been selected to be a server for
        another platform node of R type. params["pi_r_name"] and
        params["pi_r_address"] are needed."""
        result = self.execute(operation=self.target_on_pairing, params=params)
        return result

    def update(self, params):
        """Notify a platform node master that two platform nodes have been coupled.
        params["pi_s_name"], params["pi_s_address"], params["pi_r_name"] and
        params["pi_r_address"] are needed."""
        result = self.execute(operation=self.target_on_update, params=params)
        return result

    def paired(self, params):
        """Notify the platform nodes that has been paired that the master know about that
        and saved that connection. params["pi_name"] and params["pi_address"] are needed."""
        result = self.execute(operation=self.target_on_paired, params=params)
        return result

    def rollback_update(self, params):
        """Notify a platform node master that two platform nodes have been uncoupled.
        params["pi_s_name"], params["pi_s_address"], params["pi_r_name"],
        params["pi_r_address"] and params["pi_reachable"] are needed."""
        result = self.execute(operation=self.target_on_rollback_update, params=params)
        return result

    def rollback_update_all(self, params):
        """Notify a platform node master that all platform nodes have been uncoupled."""
        result = self.execute(operation=self.target_on_rollback_update_all, params=params)
        return result

    def stop_wd(self, params):
        """Stop a watchdog. params["pi_name"] is needed."""
        result = self.execute(operation=self.target_on_stop_wd, params=params)
        return result

    def stop_all_wd(self, params):
        """Stop all watchdogs."""
        result = self.execute(operation=self.target_on_stop_all_wd, params=params)
        return result

    # --- PAIRING ---

    def target_on_pairing(self, params):
        # when a platform node target receives a pairing event means that it is paired
        # to the platform node of type R that has sent him the event
        # it has to notify the master about this situation so sends an update event
        self.throw_update_event(params)
        reason = "Pairing ack"
        result = RCMPMessage()
        result.create_ok_response(reason)
        return result

    def throw_update_event(self, params):
        event = {RCMPEventHandler.EVENT_KEY: UPDATE_EVENT,
                 self.I_ADDRESS_KEY: self.pni[RCMPlatformNode.PNI_MASTER_NODE_IP]
                 if not self.pni[RCMPlatformNode.PNI_IS_MASTER] else self.pni[RCMPlatformNode.PNI_ADDRESS],
                 self.PI_S_NAME_KEY: self.pni[RCMPlatformNode.PNI_NAME],
                 self.PI_S_ADDRESS_KEY: self.pni[RCMPlatformNode.PNI_ADDRESS],
                 self.PI_R_NAME_KEY: params[self.PI_R_NAME_KEY],
                 self.PI_R_ADDRESS_KEY: params[self.PI_R_ADDRESS_KEY]}
        self._logger.info("Throwing update event = %s" % event)
        threading.Thread(target=self.update, args=(event, )).start()

    # --- UPDATE ---

    def target_on_update(self, params):
        # when the master receives the update means that the connection between the 2 peers
        # has accepted and we must finalize the flow saving this connection and starting
        # the service logic associated with the robot in this connection
        # the platform node target of an update is always the master
        threading.Thread(target=self.execute_update, args=(params, )).start()
        reason = "Update ack"
        result = RCMPMessage()
        result.create_ok_response(reason)
        return result

    def execute_update(self, params):
        from rcmp_robotics_data import execute_int_robotics_data_query
        if self.start_service_logic(params):
            # if the service logic launch went well we have to save this binding
            execute_int_robotics_data_query(self.create_conn, params,
                                            "Unable to update the connection between %s and %s" %
                                            (params[self.PI_S_NAME_KEY], params[self.PI_R_NAME_KEY]))
            # now the master know about the connection between the two ends of the pair and has to throw
            # a paired event to both
            self.throw_paired_event(params)

    def start_service_logic(self, params):
        # TODO this operation must be atomic: check if the solution is ok
        from rcmp_robotics_data import execute_int_robotics_data_query
        result = True
        ss_pi_type_target = execute_int_robotics_data_query(self.get_sl_ss_pi_type_target, params,
                                                            "Unable to get the service space target from "
                                                            "service logic of '%s'" % params[self.PI_R_NAME_KEY])
        if ss_pi_type_target:
            ssl_flow = []
            self._logger.debug("-- TMP -- Update has to create the service space on %s" % ss_pi_type_target)
            from rcmp_command import CREATE_SERVICE_SPACE, DELETE_SERVICE_SPACE, \
                START_SERVICE_NODE, KILL_SERVICE_NODE, START_SERVICE_LAUNCHER, KILL_SERVICE_LAUNCHER, \
                RCMPCommandHandler
            from rcmp_service_command import ServiceSpace, ServiceNode, ServiceLauncher
            s_space = ServiceSpace(self.pni, self.msm)
            s_node = ServiceNode(self.pni, self.msm)
            s_launcher = ServiceLauncher(self.pni, self.msm)
            # the update starts the service logic associated with the platform
            # node instance of type R at manual provisioning time
            cmd = {RCMPCommandHandler.COMMAND_KEY: CREATE_SERVICE_SPACE,
                   ServiceSpace.I_NAME_KEY: params[self.PI_S_NAME_KEY] if ss_pi_type_target == RCMPlatformNode.S_TYPE
                   else params[self.PI_R_NAME_KEY], ServiceSpace.SS_NAME_KEY: params[self.PI_R_NAME_KEY]}
            res = s_space.create(cmd)
            result = res.is_ok()
            if result:
                # the service space creation went well so we can add it in ssl_flow
                ssl_flow.append(cmd)
                # once started the service space associated with the service logic
                # we have to start all the remaining items of the service logic (nodes
                # and launchers)
                sl_items = execute_int_robotics_data_query(self.get_sl_sli, params,
                                                           "Unable to get the service logic items from "
                                                           "service logic of '%s'" % params[self.PI_R_NAME_KEY])
                if sl_items:
                    for sli in sl_items:
                        if sli[0] and sli[1] and sli[2] and sli[3]:
                            # all this parameters are mandatory
                            # sli[0] is the name of the package for that item, sli[1] is the tag of the item
                            # representing the node type in case of node or the launcher name in case of launcher
                            # sli[2] is service_item type (node or launcher) and is used to know what type of
                            # service we have to launch
                            # sli[3] is service_logic_item si_pi_type_target (meaning where to launch
                            # the item and can be S or R)
                            cmd = {ServiceSpace.I_NAME_KEY: params[self.PI_S_NAME_KEY]
                                   if sli[3] == RCMPlatformNode.S_TYPE else params[self.PI_R_NAME_KEY],
                                   ServiceSpace.SS_NAME_KEY: params[self.PI_R_NAME_KEY]}
                            if sli[2] == ServiceSpace.SLI_N_TYPE:
                                # node
                                cmd[RCMPCommandHandler.COMMAND_KEY] = START_SERVICE_NODE
                                cmd[ServiceNode.SN_PACKAGE_KEY] = sli[0]
                                cmd[ServiceNode.SN_TYPE_KEY] = sli[1]
                                # the following are optional parameters: sli[4] is a name for the service item
                                # and sli[5] is a list of params to pass to the service
                                if sli[4]:
                                    cmd[ServiceNode.SN_NAME_KEY] = sli[4]
                                if sli[5]:
                                    cmd[ServiceNode.SN_PARAMS_KEY] = sli[5]
                                if sli[0] == ServiceNode.FIROS_PACKAGE and sli[1] == ServiceNode.FIROS_TYPE:
                                    firos_connected_cb = False
                                    try:
                                        while not firos_connected_cb:
                                            ss_ext_port = execute_int_robotics_data_query(self.get_ss_ext_port, cmd,
                                                                                          "Unable to get an inbound "
                                                                                          "free port for '%s' '%s' "
                                                                                          "from service logic of '%s'"
                                                                                          %
                                                                                          (sli[0], sli[1],
                                                                                           params[self.PI_R_NAME_KEY]))
                                            if ss_ext_port:
                                                res = s_node.start(cmd)
                                                if not res.is_ok() and \
                                                        ServiceNode.FIROS_FAILURE in res.get_response_reason():
                                                    # only in case of FIROS_FAILURE we retry to launch firos
                                                    firos_connected_cb = False
                                                else:
                                                    firos_connected_cb = True
                                                # result is needed for appending the result in the rollback trace
                                                result = res.is_ok()
                                            else:
                                                # no port found but not firos error so no more firos retry
                                                firos_connected_cb = True
                                                result = False
                                    except Exception as e:
                                        self._logger.error(e)
                                        result = False
                                else:
                                    # the custom case of rcm rcmdriver.py is different because need to be managed on the
                                    # machine where we launch that node because has to check for a local free port
                                    if sli[0] == "webrtc" and sli[1] == "webRTC_master.py":
                                        cmd[ServiceNode.SN_PARAMS_KEY] = "%s r_name=%s" % \
                                                                         (cmd[ServiceNode.SN_PARAMS_KEY],
                                                                          params[self.PI_R_NAME_KEY]) \
                                            if cmd[ServiceNode.SN_PARAMS_KEY] else "r_name=%s" % \
                                                                                   params[self.PI_R_NAME_KEY]
                                    res = s_node.start(cmd)
                                    result = res.is_ok()
                            elif sli[2] == ServiceSpace.SLI_L_TYPE:
                                # launcher
                                cmd[RCMPCommandHandler.COMMAND_KEY] = START_SERVICE_LAUNCHER
                                cmd[ServiceLauncher.SL_PACKAGE_KEY] = sli[0]
                                cmd[ServiceLauncher.SL_F_LAUNCHER_KEY] = sli[1]
                                # the following are optional parameters: sli[4] is a name for the service item
                                # and sli[5] is a list of params to pass to the service
                                if sli[4]:
                                    cmd[ServiceLauncher.SL_NAME_KEY] = sli[4]
                                if sli[5]:
                                    cmd[ServiceLauncher.SL_PARAMS_KEY] = sli[5]
                                res = s_launcher.start(cmd)
                                result = res.is_ok()
                            if result:
                                # the command went well so we add it in ssl_flow
                                ssl_flow.append(cmd)
                            else:
                                # we found and error so we exit from the loop without adding ssl_flow
                                # element
                                break
                        else:
                            # some mandatory parameter missing in the service logic
                            result = False
                if not result:
                    # something went wrong so we have to rollback the service space creation
                    while len(ssl_flow) > 0:
                        # we pop (return the entry and delete from the list) from the last entry
                        # in the list that hold all the cmd went well before the error
                        cmd = ssl_flow.pop(-1)
                        if cmd[RCMPCommandHandler.COMMAND_KEY] == CREATE_SERVICE_SPACE:
                            cmd[RCMPCommandHandler.COMMAND_KEY] = DELETE_SERVICE_SPACE
                            s_space.delete(cmd)
                        elif cmd[RCMPCommandHandler.COMMAND_KEY] == START_SERVICE_NODE:
                            cmd[RCMPCommandHandler.COMMAND_KEY] = KILL_SERVICE_NODE
                            s_node.stop(cmd)
                        elif cmd[RCMPCommandHandler.COMMAND_KEY] == START_SERVICE_LAUNCHER:
                            cmd[RCMPCommandHandler.COMMAND_KEY] = KILL_SERVICE_LAUNCHER
                            s_launcher.stop(cmd)
        self._logger.debug("-- TMP -- result: %s" % result)
        return result

    def get_ss_ext_port(self, rdm, params):
        from rcmp_service_command import ServiceSpace
        ss_ext_port = rdm.get_inbound_free_port(params[ServiceSpace.I_NAME_KEY], params[ServiceSpace.SS_NAME_KEY])
        if not ss_ext_port:
            reason = "All inbound ports in the range are already used"
            raise IOError(reason)
        params[ServiceSpace.SS_EXT_PORT_KEY] = ss_ext_port
        rdm.update_used_inbound_port(params[ServiceSpace.SS_NAME_KEY], ss_ext_port)
        return ss_ext_port

    def get_sl_ss_pi_type_target(self, rdm, params):
        row = rdm.get_sl_ss_pi_type_target_from_name(params[self.PI_R_NAME_KEY])
        return row[0] if row and row[0] else None

    def get_sl_sli(self, rdm, params):
        return rdm.get_sli_from_connection_pi_r(params[self.PI_R_NAME_KEY])

    def create_conn(self, rdm, params):
        # TODO for now the name of the service space is the name of the robot
        # in case of service logic without a target where to run the service space, we
        # don't put any service space name
        row = rdm.get_sl_ss_pi_type_target_from_name(params[self.PI_R_NAME_KEY])
        rdm.insert_connection(params[self.PI_S_NAME_KEY], params[self.PI_R_NAME_KEY],
                              params[self.PI_R_NAME_KEY] if row and row[0] else None)

    def throw_paired_event(self, params):
        # the paired event is sent to both the peer in the connection
        event_s = {RCMPEventHandler.EVENT_KEY: PAIRED_EVENT, self.I_ADDRESS_KEY: params[self.PI_S_ADDRESS_KEY],
                   self.PI_NAME_KEY: params[self.PI_R_NAME_KEY], self.PI_ADDRESS_KEY: params[self.PI_R_ADDRESS_KEY]}
        self._logger.info("Throwing paired event = %s" % event_s)
        # threading.Thread(target=self.notify, args=(event_s, )).start()
        threading.Thread(target=self.paired, args=(event_s, )).start()
        event_r = {RCMPEventHandler.EVENT_KEY: PAIRED_EVENT, self.I_ADDRESS_KEY: params[self.PI_R_ADDRESS_KEY],
                   self.PI_NAME_KEY: params[self.PI_S_NAME_KEY], self.PI_ADDRESS_KEY: params[self.PI_S_ADDRESS_KEY]}
        self._logger.info("Throwing paired event = %s" % event_r)
        threading.Thread(target=self.paired, args=(event_r, )).start()

    # --- PAIRED ---

    def target_on_paired(self, params):
        # when a platform node target receives a paired event means that the master received
        # the update event notifying that the two peers agreed in pairing each other; the
        # connections table in robotics data on the master is updated and the service logic
        # has been started
        # on local we have to launch the watchdog to monitor the other peer
        # connection
        self._logger.debug("Executing paired")
        self.wdm.add_watchdog(params)
        self.wdm.get_watchdog(params).start()
        reason = "Paired ack"
        result = RCMPMessage()
        result.create_ok_response(reason)
        if not self.pni[RCMPlatformNode.PNI_IS_MASTER]:
            # the paired event arrived so the RCMPlatformNode doesn't need to directly ping
            # the master to know if it is on the platform
            # TODO the write at this level means that maybe we have to synchronize pni
            self.pni[RCMPlatformNode.PNI_STATE] = RCMPlatformNode.PAIRED_STATE
        return result

    # --- ROLLBACK UPDATE ---

    def target_on_rollback_update(self, params):
        # the platform node target of an update is always the master
        threading.Thread(target=self.execute_rollback_update, args=(params, )).start()
        reason = "Rollback update ack"
        result = RCMPMessage()
        result.create_ok_response(reason)
        return result

    def execute_rollback_update(self, params):
        from rcmp_robotics_data import execute_int_robotics_data_query
        self._logger.debug("Executing rollback update with params: %s" % params)
        conns = execute_int_robotics_data_query(self.get_conn, params,
                                                "Unable to get the connections from '%s'" %
                                                params[self.PI_R_NAME_KEY] if self.PI_R_NAME_KEY in params
                                                else params[self.PI_S_NAME_KEY])
        self._logger.debug("-- TMP -- conns: %s" % conns)
        if conns:
            from rcmp_service_command import ServiceSpace, ServiceNode, ServiceLauncher
            s_space = ServiceSpace(self.pni, self.msm)
            s_node = ServiceNode(self.pni, self.msm)
            s_launcher = ServiceLauncher(self.pni, self.msm)
            if isinstance(conns, list):
                # starting from a server we have a list of tuples
                for conn in conns:
                    params[self.PI_R_NAME_KEY] = conn[0]
                    self.rollback_update_conn(params, s_space, s_node, s_launcher)
            else:
                # starting from a robot we have only one tuple
                params[self.PI_S_NAME_KEY] = conns[0]
                self.rollback_update_conn(params, s_space, s_node, s_launcher)

    def rollback_update_conn(self, params, s_space, s_node, s_launcher):
        from rcmp_robotics_data import execute_int_robotics_data_query
        self.stop_service_logic(params, s_space, s_node, s_launcher)
        self._logger.debug("-- TMP -- Service logic stopped")
        # in every case we have to cancel the connection and the peer that results unreachable
        execute_int_robotics_data_query(self.delete_conn, params,
                                        "Unable to delete the connection between %s and %s" %
                                        (params[self.PI_S_NAME_KEY], params[self.PI_R_NAME_KEY]))
        self._logger.debug("-- TMP -- Connection deleted")
        # the following must be done to clean up all there is relative to the platform node instance
        # that results disconnected (and is not strictly related with the service logic)
        self.clean_up(params, s_space)
        self._logger.debug("-- TMP -- After clean_up")

    def stop_service_logic(self, params, s_space, s_node, s_launcher):
        from rcmp_robotics_data import execute_int_robotics_data_query
        # TODO this operation must be atomic: in case of failure all has to come back as before
        ss_name = execute_int_robotics_data_query(self.get_ss_name_from_conn, params,
                                                  "Unable to get ss_name from the connection '%s' - '%s'" %
                                                  (params[self.PI_R_NAME_KEY], params[self.PI_S_NAME_KEY]))
        self._logger.debug("-- TMP -- Stopping service logic using params: %s and ss_name: %s" % (params, ss_name))
        if ss_name:
            sl_items = execute_int_robotics_data_query(self.get_sl_sli, params,
                                                       "Unable to get the service logic items from "
                                                       "service logic of '%s'" % params[self.PI_R_NAME_KEY])
            from rcmp_command import DELETE_SERVICE_SPACE, KILL_SERVICE_NODE, KILL_SERVICE_LAUNCHER, RCMPCommandHandler
            from rcmp_service_command import ServiceSpace, ServiceNode, ServiceLauncher
            if sl_items:
                for sli in sl_items:
                    if sli[3]:
                        # sli[0] is the name of the package for that item, sli[1] is the tag of the item
                        # representing the node type in case of node or the launcher name in case of launcher
                        # sli[2] is service_item type (node or launcher) and is used to know what type of
                        # service we have to kill
                        # sli[3] is service_logic_item si_pi_type_target (meaning where the item was launched
                        # and can be S or R)
                        cmd = {ServiceSpace.I_NAME_KEY: params[self.PI_S_NAME_KEY] if sli[3] == RCMPlatformNode.S_TYPE
                               else params[self.PI_R_NAME_KEY], ServiceSpace.SS_NAME_KEY: ss_name}
                        if sli[2] == ServiceSpace.SLI_N_TYPE:
                            # node
                            cmd[RCMPCommandHandler.COMMAND_KEY] = KILL_SERVICE_NODE
                            # sli[4] is a name for the service item
                            if sli[4]:
                                cmd[ServiceNode.SN_NAME_KEY] = sli[4]
                            else:
                                cmd[ServiceNode.SN_PACKAGE_KEY] = sli[0]
                                cmd[ServiceNode.SN_TYPE_KEY] = sli[1]
                            s_node.stop(cmd)
                        elif sli[2] == ServiceSpace.SLI_L_TYPE:
                            # launcher
                            cmd[RCMPCommandHandler.COMMAND_KEY] = KILL_SERVICE_LAUNCHER
                            # sli[4] is a name for the service item
                            if sli[4]:
                                cmd[ServiceLauncher.SL_NAME_KEY] = sli[4]
                            else:
                                cmd[ServiceLauncher.SL_PACKAGE_KEY] = sli[0]
                                cmd[ServiceLauncher.SL_F_LAUNCHER_KEY] = sli[1]
                            s_launcher.stop(cmd)
                        self._logger.debug("-- TMP -- %s stopped" % cmd)
            # once stopped the service logic items (nodes and launchers) we have to delete the service space
            ss_pi_type_target = execute_int_robotics_data_query(self.get_sl_ss_pi_type_target, params,
                                                                "Unable to get the service space target from "
                                                                "service logic of '%s'" % params[self.PI_R_NAME_KEY])
            if ss_pi_type_target:
                self._logger.debug("Rollback update has to delete the service space on %s" % ss_pi_type_target)
                result = self.delete_all_of_ss(ss_name, s_space)
                if not result.is_ok():
                    self._logger.debug("-- TMP -- Forcing service space deletion")
                    # if the cleaning in the right way failed we force the robotics data cleaning
                    # and aspect that the unreachable platform node would clean himself
                    execute_int_robotics_data_query(self.delete_ss, ss_name,
                                                    "Unable to delete the service space '%s'" % ss_name)

    def get_conn(self, rdm, params):
        if self.PI_R_NAME_KEY in params:
            row = rdm.get_connection_from_connection_pi_r(params[self.PI_R_NAME_KEY])
        else:
            # if self.PI_S_NAME_KEY in params
            row = rdm.get_connection_from_connection_pi_s(params[self.PI_S_NAME_KEY])
        # in case of PI_R_NAME_KEY row is a single tuple instead in all the other cases
        # (meaning when PI_S_NAME_KEY is available) row will be a list of tuple
        return row

    def get_ss_name_from_conn(self, rdm, params):
        row = rdm.get_ss_name_from_connection(params[self.PI_S_NAME_KEY], params[self.PI_R_NAME_KEY])
        return row[0] if row and row[0] else None

    def get_sl_pi_target(self, rdm, params):
        row = rdm.get_pi_r_sl_from_connection_pi_r(params[self.PI_R_NAME_KEY])
        return row

    def delete_ss(self, rdm, ss_name):
        rdm.delete_service_space_from_name(ss_name)

    def delete_conn(self, rdm, params):
        rdm.delete_connection(params[self.PI_R_NAME_KEY])

    def delete_all_of_ss(self, ss_name, s_space):
        from rcmp_robotics_data import execute_int_robotics_data_query
        ss_info = execute_int_robotics_data_query(self.get_ss_info, ss_name,
                                                  "Unable to get the service spaces info of %s" % ss_name)
        if ss_info:
            from rcmp_command import DELETE_L_SERVICE_SPACE, DELETE_SERVICE_SPACE, RCMPCommandHandler
            from rcmp_service_command import ServiceSpace
            cmd = {RCMPCommandHandler.COMMAND_KEY: DELETE_L_SERVICE_SPACE,
                   PNodeInstance.I_ADDRESS_KEY: self.pni[RCMPlatformNode.PNI_ADDRESS],
                   ServiceSpace.SS_ADDRESS_KEY: ss_info[0],
                   ServiceSpace.SS_PORT_KEY: str(ss_info[1])}
            result = s_space.delete_local(cmd)
        else:
            result = RCMPMessage()
            result.create_error_response("Not enough info to delete services of '%s'" % ss_name)
        return result

    def roll_pi_back(self, pi_name):
        # for robots we roll back half while for servers we roll back fully
        from rcmp_command import ROLLBACK_PROVISIONING_PNODE_INSTANCE, RCMPCommandHandler
        from rcmp_platform_command import PNodeInstance as PNodeI
        cmd = {RCMPCommandHandler.COMMAND_KEY: ROLLBACK_PROVISIONING_PNODE_INSTANCE,
               PNodeI.I_ADDRESS_KEY: self.pni[RCMPlatformNode.PNI_ADDRESS],
               PNodeI.PI_NAME_KEY: pi_name}
        p_node = PNodeI(self.pni, self.wdm)
        p_node.rollback_provisioning(cmd)

    def clean_up(self, params, s_space):
        # we cannot reach params[self.PI_NAME_KEY] because is the platform node that results
        # unreachable so we cannot do the right delete and we have to locally delete the remaining
        # services associated with that node
        from rcmp_robotics_data import execute_int_robotics_data_query
        self._logger.debug("-- TMP -- Clean_up with params: %s" % params)
        rows = execute_int_robotics_data_query(self.get_ss_list, params,
                                               "Unable to get the service spaces of %s" %
                                               params[self.PI_S_NAME_KEY]
                                               if params[self.PI_REACHABLE_KEY] == RCMPlatformNode.R_TYPE
                                               else params[self.PI_R_NAME_KEY])
        if rows:
            self._logger.debug("-- TMP -- Service space missed in the rollback service logic")
            for row in rows:
                if row:
                    self.delete_all_of_ss(row[0], s_space)
        # at the end we do the rollback provisioning for the disconnected platform node instance
        self.roll_pi_back(params[self.PI_S_NAME_KEY]
                          if params[self.PI_REACHABLE_KEY] == RCMPlatformNode.R_TYPE
                          else params[self.PI_R_NAME_KEY])

    def get_ss_list(self, rdm, params):
        return rdm.get_service_space_list_from_pi(params[self.PI_S_NAME_KEY]
                                                  if params[self.PI_REACHABLE_KEY] == RCMPlatformNode.R_TYPE
                                                  else params[self.PI_R_NAME_KEY])

    def get_ss_info(self, rdm, ss_name):
        return rdm.get_ss_ip_address_port(ss_name)

    # --- ROLLBACK UPDATE ALL ---

    def target_on_rollback_update_all(self, params):
        # the platform node target of a rollback update all is always the master
        # the all version means that we have to rollback all the update between
        # we are rolling back all the update of the master platform node, so we take the
        # name of the current platform node and use it to have the connections associated
        # with it
        params[self.PI_S_NAME_KEY] = self.pni[RCMPlatformNode.PNI_NAME]
        params[self.PI_REACHABLE_KEY] = self.pni[RCMPlatformNode.PNI_TYPE]
        threading.Thread(target=self.execute_rollback_update_all, args=(params, )).start()
        reason = "Rollback update all ack"
        result = RCMPMessage()
        result.create_ok_response(reason)
        return result

    def execute_rollback_update_all(self, params):
        from rcmp_service_command import ServiceSpace
        self.execute_rollback_update(params)
        s_space = ServiceSpace(self.pni, self.msm)
        self.clean_up_all(params, s_space)
        self._logger.debug("-- TMP -- After clean_up")

    def clean_up_all(self, params, s_space):
        # this is to clean up all there is on the master that is not in the connection context
        from rcmp_robotics_data import execute_int_robotics_data_query
        self._logger.debug("-- TMP -- Clean_up_all with params: %s" % params)
        # we take all the service spaces
        rows = execute_int_robotics_data_query(self.get_ss_list_all,
                                               err_reason="Unable to get the remaining service spaces")
        if rows:
            for row in rows:
                if row:
                    self.delete_all_of_ss(row[0], s_space)
        # at the end we do the rollback provisioning for all the platform node instances except the master
        rows = execute_int_robotics_data_query(self.get_pi_list_all,
                                               err_reason="Unable to get the remaining platform instances")
        if rows:
            for row in rows:
                if row:
                    # self.roll_pi_back(row[0], row[1])
                    # for robots we roll back half while for servers we roll back fully
                    self.roll_pi_back(row[0])

    def get_ss_list_all(self, rdm):
        return rdm.get_all_service_space_names()

    def get_pi_list_all(self, rdm):
        return rdm.get_platform_instance_list(full=True, exclude_master=True)

    # --- STOP WD ---

    def target_on_stop_wd(self, params):
        # the platform node target of a clean up is always the master
        threading.Thread(target=self.wdm.delete_watchdog, args=(params, )).start()
        reason = "Stop wd ack"
        result = RCMPMessage()
        result.create_ok_response(reason)
        return result

    # --- STOP WD ---

    def target_on_stop_all_wd(self, params):
        # the platform node target of a clean up is always the master
        threading.Thread(target=self.wdm.delete_all_watchdogs).start()
        reason = "Stop all wd ack"
        result = RCMPMessage()
        result.create_ok_response(reason)
        return result


class RCMPEventHandler:
    """The handler of rcm platform events."""

    EVENT_KEY = "event"

    def __init__(self, pni=None, wdm=None, msm=None):
        self.pni = pni
        self.wdm = wdm
        self.msm = msm

    def handle(self, params):
        """Delegate a sub component to handle the event."""
        result = RCMPMessage()
        event = params[self.EVENT_KEY]
        # events about instances
        p_node_instance = PNodeInstance(self.pni, self.wdm, self.msm)
        if event == PAIRING_EVENT:
            result = p_node_instance.pairing(params)
        elif event == UPDATE_EVENT:
            result = p_node_instance.update(params)
        elif event == PAIRED_EVENT:
            result = p_node_instance.paired(params)
        elif event == ROLLBACK_UPDATE_EVENT:
            result = p_node_instance.rollback_update(params)
        elif event == ROLLBACK_UPDATE_ALL_EVENT:
            result = p_node_instance.rollback_update_all(params)
        elif event == STOP_WD_EVENT:
            result = p_node_instance.stop_wd(params)
        elif event == STOP_ALL_WD_EVENT:
            result = p_node_instance.stop_all_wd(params)
        else:
            reason = "The event '%s' does not exist" % event
            result.create_error_response(reason)
        return result
