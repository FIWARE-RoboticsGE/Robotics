__author__ = "Fabio Giuseppe Di Benedetto"

from twisted.internet import reactor
from twisted.web import http
from twisted.web.resource import Resource, NoResource
from twisted.web.server import Site
from rcmp_node import logging, RCMP_LOGGER_NAME
from rcmp_inter_communication import RCMPInterCommunicationClient, RCMPMessage
from rcmp_command import READ_SERVICE_SPACE, MAN_PROVISIONING_PNODE_INSTANCE, \
    ROLLBACK_PROVISIONING_PNODE_INSTANCE, READ_PNODE_INSTANCE, READ_PNODE_INSTANCE_LIST, \
    MAN_PROVISIONING_SLOGIC_INSTANCE, ROLLBACK_PROVISIONING_SLOGIC_INSTANCE, \
    READ_SLOGIC_INSTANCE_LIST, READ_SLOGIC_INSTANCE, RCMPCommandHandler
from rcmp_service_command import ServiceSpace, SLogicInstance
from rcmp_platform_command import PNodeInstance


SS_WEB_SERVICE = "service_space"
PI_WEB_SERVICE = "platform_instance"
SLG_WEB_SERVICE = "service_logic"
READ_WEB_SERVICE = "read"
PROVISIONING_WEB_SERVICE = "provisioning"
ROLLBACK_PROVISIONING_WEB_SERVICE = "r_provisioning"

I_NAME_PARAM = "i_name"
SS_NAME_PARAM = "ss_name"

PI_NAME_PARAM = "pi_name"
PI_SLG_PARAM = "pi_service_logic"

SLG_NAME_PARAM = "slg_name"
SLG_SN_LIST_PARAM = "sn_list"
SLG_SL_LIST_PARAM = "sl_list"


class CommandWS(Resource):
    """Root web service for commands."""

    def __init__(self):
        Resource.__init__(self)
        self.logger = logging.getLogger(RCMP_LOGGER_NAME)

    def render_OPTIONS(self, request):
        """Dispatch the post request to the internal communication server and return the result."""
        request.setHeader("Access-Control-Allow-Origin", "*")
        request.setHeader("Access-Control-Allow-Headers", "Content-Type")
        request.setHeader("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        return ""

    def generate_internal_msg(self, request):
        """Create an internal message to send to the internal communication server."""
        pass


class GCommandWS(CommandWS):
    """Root web service for commands using get method."""

    def render_GET(self, request):
        """Dispatch the get request to the internal communication server and return the result."""
        ret = RCMPMessage()
        try:
            # created using default parameters means calling to a local server
            icc = RCMPInterCommunicationClient()
            msg = None
            try:
                icc.connect()
                msg = self.generate_internal_msg(request)
                icc.send(msg.get_txt())
                res = icc.receive_all_so_far()
                if res:
                    ret.set_content(res)
                else:
                    reason = "No response received"
                    request.setResponseCode(http.INTERNAL_SERVER_ERROR)
                    ret.create_error_response(reason)
            except SyntaxError as se:
                reason = "Syntax error in the request: %s" % se
                self.logger.exception(reason)
                request.setResponseCode(http.BAD_REQUEST)
                ret.create_error_response(reason)
            except Exception as e:
                reason = "Unable to accomplish the request %s: %s" % (msg, e)
                self.logger.exception(reason)
                request.setResponseCode(http.INTERNAL_SERVER_ERROR)
                ret.create_error_response(reason)
            finally:
                icc.close()
        except Exception as e:
            reason = "Internal communication server error: %s" % e
            self.logger.exception(reason)
            request.setResponseCode(http.INTERNAL_SERVER_ERROR)
            ret.create_error_response(reason)
        request.setHeader("Content-Type", "application/json")
        request.setHeader("Access-Control-Allow-Origin", "*")
        request.setHeader("Access-Control-Allow-Headers", "Content-Type")
        request.setHeader("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        return ret.get_txt()


class PCommandWS(CommandWS):
    """Root web service for commands using post method."""

    def render_POST(self, request):
        """Dispatch the post request to the internal communication server and return the result."""
        ret = RCMPMessage()
        try:
            # created using default parameters means calling to a local server
            icc = RCMPInterCommunicationClient()
            msg = None
            try:
                icc.connect()
                ext_data = self.get_ext_data(request)
                msg = self.generate_internal_msg(ext_data)
                icc.send(msg.get_txt())
                res = icc.receive_all_so_far()
                if res:
                    ret.set_content(res)
                else:
                    reason = "No response received"
                    request.setResponseCode(http.INTERNAL_SERVER_ERROR)
                    ret.create_error_response(reason)
            except SyntaxError as se:
                reason = "Syntax error in the request: %s" % se
                self.logger.exception(reason)
                request.setResponseCode(http.BAD_REQUEST)
                ret.create_error_response(reason)
            except Exception as e:
                reason = "Unable to accomplish the request %s: %s" % (msg, e)
                self.logger.exception(reason)
                request.setResponseCode(http.INTERNAL_SERVER_ERROR)
                ret.create_error_response(reason)
            finally:
                icc.close()
        except Exception as e:
            reason = "Internal communication server error: %s" % e
            self.logger.exception(reason)
            request.setResponseCode(http.INTERNAL_SERVER_ERROR)
            ret.create_error_response(reason)
        request.setHeader("Content-Type", "application/json")
        request.setHeader("Access-Control-Allow-Origin", "*")
        request.setHeader("Access-Control-Allow-Headers", "Content-Type")
        request.setHeader("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        return ret.get_txt()

    def get_ext_data(self, request):
        """Get the data in the content of the request and return the post parameters
        in a dictionary form."""
        content = request.content.read()
        if not content:
            raise SyntaxError("No parameters in the content.")
        import json
        json_data = json.loads(content)
        self.logger.debug("json_data: %s" % json_data)
        return json_data


class SSReadWS(GCommandWS):
    """Web service to read a service space."""

    def generate_internal_msg(self, request):
        msg_dict = {RCMPCommandHandler.COMMAND_KEY: READ_SERVICE_SPACE}
        if I_NAME_PARAM in request.args and request.args[I_NAME_PARAM] and request.args[I_NAME_PARAM][0]:
            msg_dict[ServiceSpace.I_NAME_KEY] = request.args[I_NAME_PARAM][0]
        if SS_NAME_PARAM in request.args and request.args[SS_NAME_PARAM] and request.args[SS_NAME_PARAM][0]:
            # this is a mandatory parameter
            msg_dict[ServiceSpace.SS_NAME_KEY] = request.args[SS_NAME_PARAM][0]
        else:
            raise SyntaxError("%s is missing in the http request" % SS_NAME_PARAM)
        return RCMPMessage(msg_dict)


class ServiceSpaceWS(Resource):
    """Parent web service for service spaces."""

    def getChild(self, name, request):
        if name == "":
            return self
        if name == READ_WEB_SERVICE:
            return SSReadWS()
        else:
            return NoResource()


class PIProvisionWS(PCommandWS):
    """Web service to provision a platform node instance."""

    def generate_internal_msg(self, params):
        # in the case of post, this method receives params as parameter
        # instead of request: this is because in this case we don't pass
        # the http request usually (in the get case) do
        msg_dict = {RCMPCommandHandler.COMMAND_KEY: MAN_PROVISIONING_PNODE_INSTANCE}
        if PI_NAME_PARAM in params and params[PI_NAME_PARAM]:
            # this is a mandatory parameter
            msg_dict[PNodeInstance.PI_NAME_KEY] = params[PI_NAME_PARAM]
        else:
            raise SyntaxError("%s is missing in the http request" % PI_NAME_PARAM)
        if PI_SLG_PARAM in params and params[PI_SLG_PARAM]:
            msg_dict[PNodeInstance.PI_SLG_KEY] = params[PI_SLG_PARAM]
        return RCMPMessage(msg_dict)


class PIRollbackProvisionWS(PCommandWS):
    """Web service to rollback a provisioned platform node instance."""

    def generate_internal_msg(self, params):
        # in the case of post, this method receives params as parameter
        # instead of request: this is because in this case we don't pass
        # the http request usually (in the get case) do
        msg_dict = {RCMPCommandHandler.COMMAND_KEY: ROLLBACK_PROVISIONING_PNODE_INSTANCE}
        if PI_NAME_PARAM in params and params[PI_NAME_PARAM]:
            # this is a mandatory parameter
            msg_dict[PNodeInstance.PI_NAME_KEY] = params[PI_NAME_PARAM]
        else:
            raise SyntaxError("%s is missing in the http request" % PI_NAME_PARAM)
        msg_dict[PNodeInstance.PI_EXTENDED_KEY] = True
        return RCMPMessage(msg_dict)


class PIReadWS(GCommandWS):
    """Web service to read platform node instances."""

    def generate_internal_msg(self, request):
        # otherwise we suppose that we want the list of all available platform instances
        # and so READ_PNODE_INSTANCE_LIST
        msg_dict = {RCMPCommandHandler.COMMAND_KEY: READ_PNODE_INSTANCE_LIST}
        return RCMPMessage(msg_dict)


class PNodeInstanceWS(Resource):
    """Parent web service for platform node instances."""

    def getChild(self, name, request):
        if name == "":
            return self
        if name == PROVISIONING_WEB_SERVICE:
            return PIProvisionWS()
        elif name == ROLLBACK_PROVISIONING_WEB_SERVICE:
            return PIRollbackProvisionWS()
        elif name == READ_WEB_SERVICE:
            return PIReadWS()
        else:
            return NoResource()


class SLGProvisionWS(PCommandWS):
    """Web service to provision a service logic."""

    def generate_internal_msg(self, params):
        # in the case of post, this method receives params as parameter
        # instead of request: this is because in this case we don't pass
        # the http request usually (in the get case) do
        msg_dict = {RCMPCommandHandler.COMMAND_KEY: MAN_PROVISIONING_SLOGIC_INSTANCE}
        if SLG_NAME_PARAM in params and params[SLG_NAME_PARAM]:
            # this is a mandatory parameter
            msg_dict[SLogicInstance.SLG_NAME_KEY] = params[SLG_NAME_PARAM]
        else:
            raise SyntaxError("%s is missing in the http request" % SLG_NAME_PARAM)
        if SLG_SN_LIST_PARAM in params and params[SLG_SN_LIST_PARAM]:
            msg_dict[SLogicInstance.SLG_SN_LIST_KEY] = params[SLG_SN_LIST_PARAM]
        if SLG_SL_LIST_PARAM in params and params[SLG_SL_LIST_PARAM]:
            msg_dict[SLogicInstance.SLG_SL_LIST_KEY] = params[SLG_SL_LIST_PARAM]
        return RCMPMessage(msg_dict)


class SLGRollbackProvisionWS(PCommandWS):
    """Web service to rollback a provisioned service logic."""

    def generate_internal_msg(self, params):
        # in the case of post, this method receives params as parameter
        # instead of request: this is because in this case we don't pass
        # the http request usually (in the get case) do
        msg_dict = {RCMPCommandHandler.COMMAND_KEY: ROLLBACK_PROVISIONING_SLOGIC_INSTANCE}
        if SLG_NAME_PARAM in params and params[SLG_NAME_PARAM]:
            # this is a mandatory parameter
            msg_dict[SLogicInstance.SLG_NAME_KEY] = params[SLG_NAME_PARAM]
        else:
            raise SyntaxError("%s is missing in the http request" % SLG_NAME_PARAM)
        return RCMPMessage(msg_dict)


class SLGReadWS(GCommandWS):
    """Web service to read service logics."""

    def generate_internal_msg(self, request):
        if SLG_NAME_PARAM in request.args and request.args[SLG_NAME_PARAM]:
            if request.args[SLG_NAME_PARAM][0]:
                # we suppose that in case there is a parameter with service logic name
                # we want a READ_SLOGIC_INSTANCE
                msg_dict = {RCMPCommandHandler.COMMAND_KEY: READ_SLOGIC_INSTANCE,
                            SLogicInstance.SLG_NAME_KEY: request.args[SLG_NAME_PARAM][0]}
            else:
                raise SyntaxError("%s in the http request but empty" % SLG_NAME_PARAM)
        else:
            # otherwise we suppose that we want the list of all available service logics
            # and so READ_SLOGIC_INSTANCE_LIST
            msg_dict = {RCMPCommandHandler.COMMAND_KEY: READ_SLOGIC_INSTANCE_LIST}
        return RCMPMessage(msg_dict)


class ServiceLogicWS(Resource):
    """Parent web service for service logics."""

    def getChild(self, name, request):
        if name == "":
            return self
        if name == PROVISIONING_WEB_SERVICE:
            return SLGProvisionWS()
        elif name == ROLLBACK_PROVISIONING_WEB_SERVICE:
            return SLGRollbackProvisionWS()
        elif name == READ_WEB_SERVICE:
            return SLGReadWS()
        else:
            return NoResource()


class RCMPlatformNodeWSHome(Resource):
    """Web server home page."""

    def getChild(self, name, request):
        if name == "":
            return self
        if name == SS_WEB_SERVICE:
            return ServiceSpaceWS()
        elif name == PI_WEB_SERVICE:
            return PNodeInstanceWS()
        elif name == SLG_WEB_SERVICE:
            return ServiceLogicWS()
        else:
            return NoResource()

    def render_GET(self, request):
        return "<html><body>Welcome to the RCM platform node server!</body></html>"


class RCMPExtConnector:
    """The web server to communicate outside the rcm platform."""

    def __init__(self):
        self.reactor = reactor

    def start(self, ip_address=None, port=None):
        root = RCMPlatformNodeWSHome()
        factory = Site(root)
        if not ip_address:
            import netifaces
            from rcmp_node import RCMPlatformNode
            ip_address = netifaces.ifaddresses(RCMPlatformNode.
                                               NI_ETH)[netifaces.AF_INET][0][RCMPlatformNode.NI_ADDR_KEY]
        if not port:
            port = 80
        logging.getLogger(RCMP_LOGGER_NAME).info("Running RCMPExtConnector on %d" % port)
        self.reactor.listenTCP(port, factory, interface=ip_address)
        self.reactor.run()

    def stop(self):
        self.reactor.stop()
