__author__ = "Fabio Giuseppe Di Benedetto"

from rcmp_node import logging, RCMP_LOGGER_NAME, RCMPlatformNode
from rcmp_inter_communication import RCMPInterCommunicationClient, RCMPMessage

START_SERVICE_NODE = 'SN'
KILL_SERVICE_NODE = 'KN'
KILL_ALL_L_SERVICE_NODE = 'KALN'
READ_SERVICE_NODE = 'RN'
CREATE_SERVICE_SPACE = 'CS'
DELETE_SERVICE_SPACE = 'DS'
DELETE_L_SERVICE_SPACE = 'DLS'
DELETE_ALL_L_SERVICE_SPACE = 'DALS'
READ_SERVICE_SPACE = 'RS'
READ_P_IP_SERVICE_SPACE = 'RPIPS'
START_SERVICE_LAUNCHER = 'SL'
KILL_SERVICE_LAUNCHER = 'KL'
KILL_ALL_L_SERVICE_LAUNCHER = 'KALL'
READ_SERVICE_LAUNCHER = 'RL'
MAN_PROVISIONING_PNODE_INSTANCE = 'MP'
AUTO_PROVISIONING_PNODE_INSTANCE = 'PROV'
ROLLBACK_PROVISIONING_PNODE_INSTANCE = 'RBP'
READ_PNODE_INSTANCE_LIST = 'RP'
READ_PNODE_INSTANCE = 'RI'
PING_PNODE_INSTANCE = 'PING'
DISCOVERY_PNODE_INSTANCE = 'DISC'
MAN_PROVISIONING_SLOGIC_INSTANCE = 'MPSLG'
ROLLBACK_PROVISIONING_SLOGIC_INSTANCE = 'RBPSLG'
READ_SLOGIC_INSTANCE_LIST = 'RSLG'
READ_SLOGIC_INSTANCE = 'RSLGI'


class RCMPCommandAgent:
    """The generic agent that execute the command."""

    I_ADDRESS_KEY = "i_address"
    I_NAME_KEY = "i_name"

    def __init__(self, pni):
        self._logger = logging.getLogger(RCMP_LOGGER_NAME)
        self._who = ""
        self.pni = pni

    def delegate(self, params, t_out=None):
        """Delegate the command to the RCM platform node instance specified in the request. Return an RCMPMessage."""
        result = RCMPMessage()
        # the timeout provide the capability to decrease or increase the time available to perform
        # the task and can be useful let it change in base of the required operation
        if t_out:
            c = RCMPInterCommunicationClient(params[self.I_ADDRESS_KEY], t_out=t_out)
        else:
            c = RCMPInterCommunicationClient(params[self.I_ADDRESS_KEY])
        try:
            c.connect()
            # propagate the request as it is
            c.send(RCMPMessage(params).get_txt())
            # take the result of the operation
            res = c.receive_all_so_far()
            if res:
                result.set_content(res)
            else:
                reason = "No response received"
                result.create_error_response(reason)
        except Exception as e:
            reason = "Error delegating the request to the RCM platform node at %s: %s" % (params[self.I_ADDRESS_KEY], e)
            self._logger.error(reason)
            result.create_error_response(reason)
        finally:
            try:
                c.close()
            except Exception as e:
                # this exception doesn't change the result but we want to trace in the log
                self._logger.warning("Error closing the delegating client: %s" % e)
        return result

    def execute(self, m_operation=None, operation=None, params=None, t_out=None):
        """Execute the required operation with the parameters passed. Return an RCMPMessage."""
        result = RCMPMessage()
        if m_operation:
            if self.pni[RCMPlatformNode.PNI_IS_MASTER]:
                # perform the operation that can be done only by the master
                result = m_operation(params)
        if operation:
            if params and self.I_ADDRESS_KEY in params and params[self.I_ADDRESS_KEY] and \
               params[self.I_ADDRESS_KEY] != self.pni[RCMPlatformNode.PNI_ADDRESS]:
                # perform the delegation to another platform node because the target
                # is not the current one
                result = self.delegate(params, t_out)
            else:
                # perform the operation (the target is the current platform node)
                result = operation(params)
        return result


class RCMPCommandHandler:
    """The handler of rcm platform commands."""

    COMMAND_KEY = "command"

    def __init__(self, pni=None, msm=None, wdm=None):
        self.pni = pni
        self.msm = msm
        self.wdm = wdm

    def handle(self, params):
        """Delegate a sub component to execute the command. There are 2 sets of commands:
        commands addressed to services and others to the platform node itself."""
        from rcmp_service_command import ServiceSpace, ServiceLauncher, ServiceNode, SLogicInstance
        from rcmp_platform_command import PNodeInstance
        cmd = params[self.COMMAND_KEY]
        op = None
        result = RCMPMessage()
        # commands about services
        if cmd == START_SERVICE_NODE or cmd == KILL_SERVICE_NODE or \
                cmd == READ_SERVICE_NODE or cmd == KILL_ALL_L_SERVICE_NODE:
            s_node = ServiceNode(self.pni, self.msm)
            if cmd == START_SERVICE_NODE:
                op = s_node.start
            elif cmd == KILL_SERVICE_NODE:
                op = s_node.stop
            elif cmd == READ_SERVICE_NODE:
                op = s_node.read_list
            elif cmd == KILL_ALL_L_SERVICE_NODE:
                op = s_node.stop_all_local
        elif cmd == CREATE_SERVICE_SPACE or cmd == DELETE_SERVICE_SPACE or \
                cmd == READ_SERVICE_SPACE or cmd == READ_P_IP_SERVICE_SPACE or \
                cmd == DELETE_ALL_L_SERVICE_SPACE:
            s_space = ServiceSpace(self.pni, self.msm)
            if cmd == CREATE_SERVICE_SPACE:
                op = s_space.create
            elif cmd == DELETE_SERVICE_SPACE:
                op = s_space.delete
            elif cmd == READ_SERVICE_SPACE:
                op = s_space.read
            elif cmd == READ_P_IP_SERVICE_SPACE:
                op = s_space.read_public_ip
            elif cmd == DELETE_L_SERVICE_SPACE:
                op = s_space.delete_local
            elif cmd == DELETE_ALL_L_SERVICE_SPACE:
                op = s_space.delete_all_local
        elif cmd == START_SERVICE_LAUNCHER or cmd == KILL_SERVICE_LAUNCHER or \
                cmd == READ_SERVICE_LAUNCHER or cmd == KILL_ALL_L_SERVICE_LAUNCHER:
            s_launcher = ServiceLauncher(self.pni, self.msm)
            if cmd == START_SERVICE_LAUNCHER:
                op = s_launcher.start
            elif cmd == KILL_SERVICE_LAUNCHER:
                op = s_launcher.stop
            elif cmd == READ_SERVICE_LAUNCHER:
                op = s_launcher.read_list
            elif cmd == KILL_ALL_L_SERVICE_LAUNCHER:
                op = s_launcher.stop_all_local
        # commands about instances
        elif cmd == MAN_PROVISIONING_PNODE_INSTANCE or cmd == AUTO_PROVISIONING_PNODE_INSTANCE \
                or cmd == ROLLBACK_PROVISIONING_PNODE_INSTANCE or cmd == READ_PNODE_INSTANCE_LIST \
                or cmd == READ_PNODE_INSTANCE or cmd == PING_PNODE_INSTANCE \
                or cmd == DISCOVERY_PNODE_INSTANCE:
            p_node_instance = PNodeInstance(self.pni, self.wdm)
            if cmd == MAN_PROVISIONING_PNODE_INSTANCE:
                op = p_node_instance.manual_provisioning
            elif cmd == AUTO_PROVISIONING_PNODE_INSTANCE:
                op = p_node_instance.automatic_provisioning
            elif cmd == ROLLBACK_PROVISIONING_PNODE_INSTANCE:
                op = p_node_instance.rollback_provisioning
            elif cmd == READ_PNODE_INSTANCE_LIST:
                op = p_node_instance.read_list
            elif cmd == READ_PNODE_INSTANCE:
                op = p_node_instance.read
            elif cmd == PING_PNODE_INSTANCE:
                op = p_node_instance.ping
            elif cmd == DISCOVERY_PNODE_INSTANCE:
                op = p_node_instance.discovery
        elif cmd == MAN_PROVISIONING_SLOGIC_INSTANCE or cmd == ROLLBACK_PROVISIONING_SLOGIC_INSTANCE \
                or cmd == READ_SLOGIC_INSTANCE_LIST or cmd == READ_SLOGIC_INSTANCE:
            s_logic_instance = SLogicInstance(self.pni)
            if cmd == MAN_PROVISIONING_SLOGIC_INSTANCE:
                op = s_logic_instance.manual_provisioning
            elif cmd == ROLLBACK_PROVISIONING_SLOGIC_INSTANCE:
                op = s_logic_instance.rollback_provisioning
            elif cmd == READ_SLOGIC_INSTANCE_LIST:
                op = s_logic_instance.read_list
            elif cmd == READ_SLOGIC_INSTANCE:
                op = s_logic_instance.read
        else:
            reason = "The command '%s' does not exist" % cmd
            result.create_error_response(reason)
        if op is not None:
            result = op(params)
        return result
