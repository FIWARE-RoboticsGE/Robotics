__author__ = "Fabio Giuseppe Di Benedetto"

import socket
from SocketServer import ThreadingTCPServer, BaseRequestHandler
import json
import os
import errno
import time
from rcmp_node import logging, RCMP_LOGGER_NAME, threading, netifaces

DEFAULT_SERVER_ADDRESS = netifaces.ifaddresses("tun0")[netifaces.AF_INET][0]["addr"]
DEFAULT_SERVER_PORT = 9999


def check_port(address, port):
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        s.bind((address, port))
        address, port = s.getsockname()
    except socket.error as se:
        if se.errno == errno.EADDRINUSE:
            return None
    return address, port


class RCMPInterCommunicationClient:
    """The client for the internal communications between rcm platform node instances. In the
    master platform node instance is even used for the communication between the external connector
    and the internal command handler."""

    def __init__(self, address=DEFAULT_SERVER_ADDRESS, port=DEFAULT_SERVER_PORT, t_out=60.0):
        self._address = (address, port)
        self._conn = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._buf_size = 4096
        self._timeout = t_out
        # we add 2 seconds to the socket timer so that we can use it without impacting the use
        # of the select: in case of network unavailability when the connection is already taken
        # the timer of the select counts otherwise this network timer counts (this happens to be
        # useful when we try to connect but we don't have network)
        self._conn.settimeout(self._timeout + 2)
        self._logger = logging.getLogger(RCMP_LOGGER_NAME)

    def connect(self):
        """Connect to the server."""
        self._conn.connect(self._address)

    def send(self, msg):
        """Send a message."""
        self._conn.sendall(msg)

    def receive_all_so_far(self):
        # WARNING this only works on linux: is the same as read_all_so_far but using
        # the socket instead the streams (std out and std err)
        import select
        received = ""
        retry = 0
        all_data_taken = False
        inputs = [self._conn]
        outputs = []
        readable_list, _, exceptional_list = select.select(inputs, outputs, inputs, self._timeout)
        # in case of blocking servers when the client find some input available the channel
        # keep to be readable and the select at the end always return the same readable objects:
        # in these cases we check the data too, because in these cases data will be empty and
        # this means that we finished the reception: in case of much data they need some time
        # to be recovered, so we have to delay the exit to be sure we took all the data
        while readable_list and not exceptional_list and not all_data_taken:
            # some readable objects are available for the receive
            for readable in readable_list:
                if readable.fileno() == self._conn.fileno():
                    data = self._conn.recv(self._buf_size)
                    if received:
                        # some data already available and new received: append new data
                        received += data
                    else:
                        # no data previously available: add the new data
                        received = data
                    if received:
                        # we check if we received all the msg
                        try:
                            json.loads(received)
                        except Exception as e:
                            # exception means that the msg is not completed
                            if data:
                                # every time we receive data we reset the retry
                                retry = 0
                            else:
                                # every time we receive nothing means that the missing parts of
                                # the msg aren't arrived yet: we delay the reception but give 3
                                # retry before failing definitely
                                retry += 1
                                if retry < 3:
                                    time.sleep(1)
                        else:
                            # if the check went well we can exit
                            all_data_taken = True
            if not self._conn:
                # if the socket was delete we clean the readable_list so that we exit
                readable_list = None
            else:
                # in case of big packets they are split in n packet: not all packets come
                # in the same readable block, so in case we haven't finished we have to wait
                # until the next readable block
                if not all_data_taken:
                    readable_list, _, exceptional_list = select.select(inputs, outputs, inputs, self._timeout)
        if exceptional_list:
            raise IOError("Socket caused exception")
        return received

    def close(self):
        """Close the connection with the server."""
        self._conn.close()


class RCMPPacketHandler(BaseRequestHandler):
    """The server handler of the received requests."""

    def handle(self):
        """Take the content of the RCMP packet and use the dispatcher
        to handle the command."""
        logger = logging.getLogger(RCMP_LOGGER_NAME)
        # logger.debug("Handling the request from %s:%d" % self.client_address)
        response = RCMPMessage()
        try:
            # TODO see if we need a receive with more data (these are the requests)
            content = self.request.recv(1024)
            if content:
                msg = RCMPMessage()
                msg.set_content(content)
                response = self.server.dispatcher.dispatch(msg.get_dict())
        except Exception as e:
            reason = "Error executing the request: %s" % e
            logger.error(reason)
            response.create_error_response(reason)
        finally:
            try:
                self.request.sendall(response.get_txt())
            except Exception as e:
                logger.error("Error sending the response: %s" % e)


class RCMPInterCommunicationServer(ThreadingTCPServer):
    """The server for the internal communications between rcm platform node instances. In the
    master platform node instance is even used for the communication between the external connector
    and the internal command handler."""

    ROS_IP_ENV = "ROS_IP"
    # we allow the reuse of address so we don't have to wait when we restart
    # the server: this has to be set before the __init__ because TCPServer has
    # the bind() embedded in it
    allow_reuse_address = True

    def __init__(self, server_address=(DEFAULT_SERVER_ADDRESS, DEFAULT_SERVER_PORT),
                 RequestHandlerClass=RCMPPacketHandler, dispatcher=None):
        ThreadingTCPServer.__init__(self, server_address, RequestHandlerClass)
        # keep track of the handlers used to process the requests
        self.handlers = {}
        self.dispatcher = dispatcher
        # we set the ros_ip at this level because in case of more than one ip
        # address this is the one we use for the platform and we want to use
        # for ros too
        os.environ[self.ROS_IP_ENV] = server_address[0]

    def process_request(self, request, client_address):
        """Start a new thread to process the request."""
        # this is specific when an handler has to process a request
        t = threading.Thread(target=self.process_request_thread,
                             args=(request, client_address))
        t.daemon = self.daemon_threads
        self.handlers[request] = t
        t.start()

    def close_request(self, request):
        """Clean up an individual request."""
        # this is specific when an handler has finished his work
        request.close()
        del self.handlers[request]

    def shutdown(self):
        """Stop the server."""
        # this is the call for shutting down the server
        ThreadingTCPServer.shutdown(self)
        # wait until all the requests have finished
        for h in self.handlers.values():
            h.join()


class RCMPMessage:
    """The message of rcm platform."""

    RESPONSE_RESULT = "result"
    RESPONSE_REASON = "reason"
    RESULT_OK = "OK"
    RESULT_ERROR = "ERROR"

    def __init__(self, content=None):
        """Initialize the RCMPMessage using the specified dict argument."""
        self.content = content

    def set_content(self, content):
        """Set the content of the message starting from the specified string argument."""
        if content:
            self.content = json.loads(content)

    def create_ok_response(self, reason):
        """Create an ok response message."""
        self.content = {self.RESPONSE_RESULT: self.RESULT_OK, self.RESPONSE_REASON: reason}

    def create_error_response(self, reason):
        """Create an error response message."""
        self.content = {self.RESPONSE_RESULT: self.RESULT_ERROR, self.RESPONSE_REASON: reason}

    def add_response_warning(self, warning):
        """Add a warning into the response."""
        if self.content and self.RESPONSE_REASON in self.content:
            self.content[self.RESPONSE_REASON] += "(warning: %s)" % warning

    def is_empty(self):
        """Check if the message is empty."""
        return self.content is None

    def is_ok(self):
        """Check if the message is a response and the result of the response is ok."""
        if self.content and self.RESPONSE_RESULT in self.content:
            return self.content[self.RESPONSE_RESULT] == self.RESULT_OK
        return False

    def get_response_reason(self):
        """Get the reason into a response message."""
        return self.content[self.RESPONSE_REASON] if self.content and self.RESPONSE_REASON in self.content else None

    def get_txt(self):
        """Get the text version of the message."""
        return json.dumps(self.content) if self.content else None

    def get_dict(self):
        """Get the dictionary version of the message."""
        return self.content


class RCMPDispatcher:
    """The dispatcher of rcm platform messages."""

    def __init__(self, pni=None, msm=None, wdm=None):
        self.pni = pni
        self.msm = msm
        self.wdm = wdm

    def dispatch(self, params):
        """Dispatch the command to the components that have to handle it. Receive
        params in form of dict type and return the response as RCMPMessage."""
        from rcmp_command import RCMPCommandHandler
        from rcmp_event import RCMPEventHandler
        if RCMPCommandHandler.COMMAND_KEY in params:
            rh = RCMPCommandHandler(self.pni, self.msm, self.wdm)
        elif RCMPEventHandler.EVENT_KEY in params:
            rh = RCMPEventHandler(self.pni, self.wdm, self.msm)
        else:
            reason = "The RCM platform node instance manages only commands or events"
            response = RCMPMessage()
            response.create_error_response(reason)
            return response
        return rh.handle(params)
