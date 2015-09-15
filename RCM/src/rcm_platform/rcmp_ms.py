__author__ = "Fabio Giuseppe Di Benedetto"

from rcmp_inter_communication import threading
from rcmp_command import logging, RCMP_LOGGER_NAME, CREATE_SERVICE_SPACE, START_SERVICE_LAUNCHER, \
    START_SERVICE_NODE, DELETE_SERVICE_SPACE, KILL_SERVICE_LAUNCHER, KILL_SERVICE_NODE, \
    DELETE_ALL_L_SERVICE_SPACE, KILL_ALL_L_SERVICE_NODE, KILL_ALL_L_SERVICE_LAUNCHER, RCMPCommandHandler
from rcmp_service_command import ServiceNode, ServiceLauncher, ServiceSpace


class MServiceManager:
    postfix = "autogn"

    def __init__(self):
        self._logger = logging.getLogger(RCMP_LOGGER_NAME)
        # we keep track of the services managed by the platform node instance
        # (the running services on it)
        self.ms = {}
        # lock for synchronization around managed_services
        self.ms_lock = threading.RLock()

    # for all the method relative to managed service:
    # key_mask is a dict in which values are keys for access the
    # managed services (mask of indexes): we re-use the params we
    # received by the request to accomplish this task; to
    # traverse key_mask we use km_traversal_mask which is the list
    # of keys in key_mask to take the values we need; the order in
    # which we pick the elements of km_traversal_mask defines the
    # level in this sort of tree

    def add_managed_service(self, key_mask, ms_data):
        """Add a managed service."""
        try:
            self.ms_lock.acquire()
            is_new = False
            if ServiceSpace.SS_ADDRESS_KEY and ServiceSpace.SS_PORT_KEY in key_mask:
                # the params needed for have the service space are available
                if key_mask[ServiceSpace.SS_ADDRESS_KEY] not in self.ms:
                    # the specified service space address doesn't exist
                    self.ms[key_mask[ServiceSpace.SS_ADDRESS_KEY]] = {}
                    is_new = True
                if key_mask[ServiceSpace.SS_PORT_KEY] not in self.ms[key_mask[ServiceSpace.SS_ADDRESS_KEY]]:
                    # the specified service space port doesn't exist
                    self.ms[key_mask[ServiceSpace.SS_ADDRESS_KEY]][key_mask[ServiceSpace.SS_PORT_KEY]] = {}
                    is_new = True
            target = self.ms[key_mask[ServiceSpace.SS_ADDRESS_KEY]][key_mask[ServiceSpace.SS_PORT_KEY]]
            if is_new:
                if key_mask[RCMPCommandHandler.COMMAND_KEY] == CREATE_SERVICE_SPACE:
                    if ServiceSpace.SS_NAME_KEY in key_mask:
                        target[ServiceSpace.SS_NAME_KEY] = key_mask[ServiceSpace.SS_NAME_KEY]
                    target[ServiceSpace.SS_OWNED] = True
                    target.update(ms_data)
                target[ServiceSpace.SN_LIST] = []
                target[ServiceSpace.SL_LIST] = []
            if key_mask[RCMPCommandHandler.COMMAND_KEY] == START_SERVICE_NODE:
                sn = {}
                # adding a node can happen that the node already exist: the new running node
                # kills the old one, but we must clean the data structure and put the new info
                idx = 0
                for old_sn in target[ServiceSpace.SN_LIST]:
                    if old_sn[ServiceNode.SN_NAME_KEY] == key_mask[ServiceNode.SN_NAME_KEY]:
                        target[ServiceSpace.SN_LIST].pop(idx)
                    idx += 1
                if ServiceNode.SN_PACKAGE_KEY in key_mask:
                    sn[ServiceNode.SN_PACKAGE_KEY] = key_mask[ServiceNode.SN_PACKAGE_KEY]
                if ServiceNode.SN_TYPE_KEY in key_mask:
                    sn[ServiceNode.SN_TYPE_KEY] = key_mask[ServiceNode.SN_TYPE_KEY]
                if ServiceNode.SN_NAME_KEY in key_mask:
                    sn[ServiceNode.SN_NAME_KEY] = key_mask[ServiceNode.SN_NAME_KEY]
                if ServiceNode.SN_AUTO_GEN_NAME_KEY in key_mask:
                    sn[ServiceNode.SN_AUTO_GEN_NAME_KEY] = key_mask[ServiceNode.SN_AUTO_GEN_NAME_KEY]
                if ServiceNode.SN_PARAMS_KEY in key_mask:
                    sn[ServiceNode.SN_PARAMS_KEY] = key_mask[ServiceNode.SN_PARAMS_KEY]
                if ServiceSpace.SS_OWNED in target and target[ServiceSpace.SS_OWNED]:
                    sn[ServiceNode.SN_LOCAL_KEY] = True
                sn.update(ms_data)
                target[ServiceSpace.SN_LIST].append(sn)
            if key_mask[RCMPCommandHandler.COMMAND_KEY] == START_SERVICE_LAUNCHER:
                sl = {}
                if ServiceLauncher.SL_PACKAGE_KEY in key_mask:
                    sl[ServiceLauncher.SL_PACKAGE_KEY] = key_mask[ServiceLauncher.SL_PACKAGE_KEY]
                if ServiceLauncher.SL_F_LAUNCHER_KEY in key_mask:
                    sl[ServiceLauncher.SL_F_LAUNCHER_KEY] = key_mask[ServiceLauncher.SL_F_LAUNCHER_KEY]
                if ServiceLauncher.SL_NAME_KEY in key_mask:
                    sl[ServiceLauncher.SL_NAME_KEY] = key_mask[ServiceLauncher.SL_NAME_KEY]
                else:
                    idx = 0
                    for old_sl in target[ServiceSpace.SL_LIST]:
                        if ServiceLauncher.SL_PACKAGE_KEY in key_mask and \
                                old_sl[ServiceLauncher.SL_PACKAGE_KEY] == key_mask[ServiceLauncher.SL_PACKAGE_KEY]:
                            if ServiceLauncher.SL_F_LAUNCHER_KEY in key_mask and \
                                    old_sl[ServiceLauncher.SL_F_LAUNCHER_KEY] == \
                                    key_mask[ServiceLauncher.SL_F_LAUNCHER_KEY]:
                                if ServiceLauncher.SL_AUTO_GEN_NAME_KEY in old_sl and \
                                        old_sl[ServiceLauncher.SL_AUTO_GEN_NAME_KEY]:
                                    idx += 1
                    sl[ServiceLauncher.SL_NAME_KEY] = "%s_%s%d" % \
                                                      (key_mask[ServiceLauncher.SL_F_LAUNCHER_KEY], self.postfix, idx)
                    sl[ServiceLauncher.SL_AUTO_GEN_NAME_KEY] = True
                if ServiceLauncher.SL_PARAMS_KEY in key_mask:
                    sl[ServiceLauncher.SL_PARAMS_KEY] = key_mask[ServiceLauncher.SL_PARAMS_KEY]
                sl.update(ms_data)
                target[ServiceSpace.SL_LIST].append(sl)
            self._logger.debug("------ key_mask ------")
            self._logger.debug(key_mask)
            self._logger.debug("------ ms_data ------")
            self._logger.debug(ms_data)
            self._logger.debug("------ ms ------")
            self._logger.debug(self.ms)
            self._logger.debug("---------------")
        finally:
            self.ms_lock.release()

    def get_managed_service_data(self, key_mask):
        """Get the data for a managed service. Return a dictionary with the process object running that service
        or a list of dictionaries with the characteristics specified in key_mask. In case of list, the dictionaries
        have the name of the node relative to that dictionary; in case of launcher the dictionary has the process
        object and another field with the list of nodes managed by that launcher."""
        try:
            self.ms_lock.acquire()
            self._logger.debug("------ key_mask ------")
            self._logger.debug(key_mask)
            self._logger.debug("------ ms ------")
            self._logger.debug(self.ms)
            self._logger.debug("---------------")
            if key_mask[RCMPCommandHandler.COMMAND_KEY] == DELETE_ALL_L_SERVICE_SPACE:
                ss_l = []
                for ss_address in self.ms.keys():
                    for ss_port in self.ms[ss_address].keys():
                        target = {ServiceSpace.SS_ADDRESS_KEY: ss_address, ServiceSpace.SS_PORT_KEY: ss_port}
                        ss_l.append(target)
                if ss_l:
                    return ss_l[0] if len(ss_l) == 1 else ss_l
            else:
                if ServiceSpace.SS_ADDRESS_KEY and ServiceSpace.SS_PORT_KEY in key_mask:
                    # the params needed for have the service space are available
                    if key_mask[ServiceSpace.SS_ADDRESS_KEY] not in self.ms:
                        raise ValueError("%s not managed by this platform node instance" %
                                         key_mask[ServiceSpace.SS_ADDRESS_KEY])
                    if key_mask[ServiceSpace.SS_PORT_KEY] not in self.ms[key_mask[ServiceSpace.SS_ADDRESS_KEY]]:
                        raise ValueError("%s not managed by this platform node instance" %
                                         key_mask[ServiceSpace.SS_PORT_KEY])
                target = self.ms[key_mask[ServiceSpace.SS_ADDRESS_KEY]][key_mask[ServiceSpace.SS_PORT_KEY]]
                if key_mask[RCMPCommandHandler.COMMAND_KEY] == DELETE_SERVICE_SPACE:
                    return target
                if key_mask[RCMPCommandHandler.COMMAND_KEY] == KILL_SERVICE_NODE:
                    if ServiceNode.SN_NAME_KEY in key_mask and key_mask[ServiceNode.SN_NAME_KEY]:
                        for sn in target[ServiceSpace.SN_LIST]:
                            if sn[ServiceNode.SN_NAME_KEY] == key_mask[ServiceNode.SN_NAME_KEY]:
                                return sn
                    elif ServiceNode.SN_PACKAGE_KEY in key_mask and key_mask[ServiceNode.SN_PACKAGE_KEY] and \
                            ServiceNode.SN_TYPE_KEY in key_mask and key_mask[ServiceNode.SN_TYPE_KEY]:
                        # we can use package and type only if the user doesn't know the name (it is auto generated
                        # by ros platform): can be more than one in case of anonymous option (that adds a numeric
                        # postfix to keep names unique)
                        sn_l = []
                        for sn in target[ServiceSpace.SN_LIST]:
                            if sn[ServiceNode.SN_PACKAGE_KEY] == key_mask[ServiceNode.SN_PACKAGE_KEY] and \
                               sn[ServiceNode.SN_TYPE_KEY] == key_mask[ServiceNode.SN_TYPE_KEY] and \
                               ServiceNode.SN_AUTO_GEN_NAME_KEY in sn and sn[ServiceNode.SN_AUTO_GEN_NAME_KEY]:
                                sn_l.append(sn)
                        if sn_l:
                            return sn_l[0] if len(sn_l) == 1 else sn_l
                    # if we arrive here is because we haven't found a node with the specified name
                    if ServiceNode.SN_NAME_KEY in key_mask and key_mask[ServiceNode.SN_NAME_KEY]:
                        raise ValueError("Service node '%s' isn't managed by this platform node instance" %
                                         key_mask[ServiceNode.SN_NAME_KEY])
                    else:
                        raise ValueError("Service node without user defined name isn't managed "
                                         "by this platform node instance")
                if key_mask[RCMPCommandHandler.COMMAND_KEY] == KILL_ALL_L_SERVICE_NODE:
                    sn_l = []
                    for sn in target[ServiceSpace.SN_LIST]:
                        sn_l.append(sn)
                    if sn_l:
                        return sn_l[0] if len(sn_l) == 1 else sn_l
                if key_mask[RCMPCommandHandler.COMMAND_KEY] == KILL_SERVICE_LAUNCHER:
                    if ServiceLauncher.SL_NAME_KEY in key_mask and key_mask[ServiceLauncher.SL_NAME_KEY]:
                        for sl in target[ServiceSpace.SL_LIST]:
                            if sl[ServiceLauncher.SL_NAME_KEY] == key_mask[ServiceLauncher.SL_NAME_KEY]:
                                return sl
                    elif ServiceLauncher.SL_PACKAGE_KEY in key_mask and \
                            key_mask[ServiceLauncher.SL_PACKAGE_KEY] and \
                            ServiceLauncher.SL_F_LAUNCHER_KEY in key_mask and \
                            key_mask[ServiceLauncher.SL_F_LAUNCHER_KEY]:
                        # we can use launcher file name only if the user doesn't know the name (it is auto generated):
                        # can be more than one
                        sl_l = []
                        for sl in target[ServiceSpace.SL_LIST]:
                            if sl[ServiceLauncher.SL_PACKAGE_KEY] == key_mask[ServiceLauncher.SL_PACKAGE_KEY] and \
                                    sl[ServiceLauncher.SL_F_LAUNCHER_KEY] == \
                                    key_mask[ServiceLauncher.SL_F_LAUNCHER_KEY] and \
                                    ServiceLauncher.SL_AUTO_GEN_NAME_KEY in sl and \
                                    sl[ServiceLauncher.SL_AUTO_GEN_NAME_KEY]:
                                sl_l.append(sl)
                        if sl_l:
                            return sl_l[0] if len(sl_l) == 1 else sl_l
                    # if we arrive here is because we haven't found a node with the specified name
                    if ServiceLauncher.SL_NAME_KEY in key_mask and key_mask[ServiceLauncher.SL_NAME_KEY]:
                        raise ValueError("Service launcher '%s' isn't managed by this platform node instance" %
                                         key_mask[ServiceLauncher.SL_NAME_KEY])
                    else:
                        raise ValueError("Service launcher without user defined name isn't managed "
                                         "by this platform node instance")
                if key_mask[RCMPCommandHandler.COMMAND_KEY] == KILL_ALL_L_SERVICE_LAUNCHER:
                    sl_l = []
                    for sl in target[ServiceSpace.SL_LIST]:
                        sl_l.append(sl)
                    if sl_l:
                        return sl_l[0] if len(sl_l) == 1 else sl_l
        finally:
            self.ms_lock.release()

    def get_managed_service(self, key_mask):
        """Get the branch of managed services"""
        try:
            self.ms_lock.acquire()
            branch = None
            if ServiceSpace.SS_ADDRESS_KEY and ServiceSpace.SS_PORT_KEY in key_mask:
                # the params needed for have the service space are available
                # if key_mask[ServiceSpace.SS_ADDRESS_KEY] not in self.ms:
                #     raise ValueError("%s not managed by this platform node instance" %
                #                      key_mask[ServiceSpace.SS_ADDRESS_KEY])
                # if key_mask[ServiceSpace.SS_PORT_KEY] not in self.ms[key_mask[ServiceSpace.SS_ADDRESS_KEY]]:
                #     raise ValueError("%s not managed by this platform node instance" %
                #                      key_mask[ServiceSpace.SS_PORT_KEY])
                if key_mask[ServiceSpace.SS_ADDRESS_KEY] not in self.ms or \
                   key_mask[ServiceSpace.SS_PORT_KEY] not in self.ms[key_mask[ServiceSpace.SS_ADDRESS_KEY]]:
                    self._logger.error("Trying to access unavailable services using service space %s:%s" %
                                       (key_mask[ServiceSpace.SS_ADDRESS_KEY], key_mask[ServiceSpace.SS_PORT_KEY]))
                    self._logger.error("---- ms dump - begin ----")
                    self._logger.error(self.ms)
                    self._logger.error("---- ms dump - end ----")
                else:
                    branch = self.ms[key_mask[ServiceSpace.SS_ADDRESS_KEY]][key_mask[ServiceSpace.SS_PORT_KEY]]
            return branch
        finally:
            self.ms_lock.release()

    def delete_managed_service(self, key_mask, ms_data):
        """Delete a managed service."""
        try:
            self.ms_lock.acquire()
            self._logger.debug("------ key_mask ------")
            self._logger.debug(key_mask)
            self._logger.debug("------ ms_data ------")
            self._logger.debug(ms_data)
            self._logger.debug("------ ms before delete------")
            self._logger.debug(self.ms)
            self._logger.debug("---------------")
            if ServiceSpace.SS_ADDRESS_KEY and ServiceSpace.SS_PORT_KEY in key_mask:
                # the params needed for have the service space are available
                if key_mask[ServiceSpace.SS_ADDRESS_KEY] not in self.ms:
                    raise ValueError("%s not managed by this platform node instance" %
                                     key_mask[ServiceSpace.SS_ADDRESS_KEY])
                if key_mask[ServiceSpace.SS_PORT_KEY] not in self.ms[key_mask[ServiceSpace.SS_ADDRESS_KEY]]:
                    raise ValueError("%s not managed by this platform node instance" %
                                     key_mask[ServiceSpace.SS_PORT_KEY])
            target = self.ms[key_mask[ServiceSpace.SS_ADDRESS_KEY]][key_mask[ServiceSpace.SS_PORT_KEY]]
            if key_mask[RCMPCommandHandler.COMMAND_KEY] == DELETE_SERVICE_SPACE:
                if len(self.ms[key_mask[ServiceSpace.SS_ADDRESS_KEY]].keys()) > 1:
                    # there are other service spaces in this address and must be preserved
                    return self.ms[key_mask[ServiceSpace.SS_ADDRESS_KEY]].pop(key_mask[ServiceSpace.SS_PORT_KEY])
                else:
                    return self.ms.pop(key_mask[ServiceSpace.SS_ADDRESS_KEY])
            if key_mask[RCMPCommandHandler.COMMAND_KEY] == KILL_SERVICE_NODE or \
               key_mask[RCMPCommandHandler.COMMAND_KEY] == KILL_ALL_L_SERVICE_NODE:
                if (ServiceNode.SN_NAME_KEY in key_mask and key_mask[ServiceNode.SN_NAME_KEY]) or \
                        (ServiceNode.SN_NAME_KEY in ms_data and ms_data[ServiceNode.SN_NAME_KEY]):
                    if len(target[ServiceSpace.SN_LIST]) == 1 and len(target[ServiceSpace.SL_LIST]) == 0 \
                            and not(ServiceSpace.SS_OWNED in target and target[ServiceSpace.SS_OWNED]):
                        # only sn_list has one element (presumably the one we want delete) and this is a service
                        # space that the platform node instance doesn't own
                        if ServiceNode.SN_NAME_KEY in key_mask and key_mask[ServiceNode.SN_NAME_KEY]:
                            if target[ServiceSpace.SN_LIST][0][ServiceNode.SN_NAME_KEY] != \
                                    key_mask[ServiceNode.SN_NAME_KEY]:
                                raise ValueError("Service node '%s' isn't managed by this platform node instance" %
                                                 key_mask[ServiceNode.SN_NAME_KEY])
                        elif ServiceNode.SN_NAME_KEY in ms_data and ms_data[ServiceNode.SN_NAME_KEY]:
                            if target[ServiceSpace.SN_LIST][0][ServiceNode.SN_NAME_KEY] != \
                                    ms_data[ServiceNode.SN_NAME_KEY]:
                                raise ValueError("Service node '%s' isn't managed by this platform node instance" %
                                                 ms_data[ServiceNode.SN_NAME_KEY])
                        if len(self.ms[key_mask[ServiceSpace.SS_ADDRESS_KEY]].keys()) > 1:
                            # there are other service spaces in this address and must be preserved
                            return self.ms[key_mask[ServiceSpace.SS_ADDRESS_KEY]]\
                                .pop(key_mask[ServiceSpace.SS_PORT_KEY])
                        else:
                            return self.ms.pop(key_mask[ServiceSpace.SS_ADDRESS_KEY])
                    # in all other cases we must delete only the element
                    idx = 0
                    for sn in target[ServiceSpace.SN_LIST]:
                        if ServiceNode.SN_NAME_KEY in key_mask and key_mask[ServiceNode.SN_NAME_KEY]:
                            if sn[ServiceNode.SN_NAME_KEY] == key_mask[ServiceNode.SN_NAME_KEY]:
                                return target[ServiceSpace.SN_LIST].pop(idx)
                        elif ServiceNode.SN_NAME_KEY in ms_data and ms_data[ServiceNode.SN_NAME_KEY]:
                            if sn[ServiceNode.SN_NAME_KEY] == ms_data[ServiceNode.SN_NAME_KEY]:
                                return target[ServiceSpace.SN_LIST].pop(idx)
                        idx += 1
                # if we arrive here is because we haven't found a node with the specified name
                raise ValueError("Service node '%s' isn't managed by this platform node instance" %
                                 key_mask[ServiceNode.SN_NAME_KEY])
            if key_mask[RCMPCommandHandler.COMMAND_KEY] == KILL_SERVICE_LAUNCHER or \
               key_mask[RCMPCommandHandler.COMMAND_KEY] == KILL_ALL_L_SERVICE_LAUNCHER:
                if (ServiceLauncher.SL_NAME_KEY in key_mask and key_mask[ServiceLauncher.SL_NAME_KEY]) or \
                        (ServiceLauncher.SL_NAME_KEY in ms_data and ms_data[ServiceLauncher.SL_NAME_KEY]):
                    if len(target[ServiceSpace.SL_LIST]) == 1 and len(target[ServiceSpace.SN_LIST]) == 0 \
                            and not(ServiceSpace.SS_OWNED in target and target[ServiceSpace.SS_OWNED]):
                        # only sl_list has one element (presumably the one we want delete) and this is a service
                        # space that the platform node instance doesn't own
                        if ServiceLauncher.SL_NAME_KEY in key_mask and key_mask[ServiceLauncher.SL_NAME_KEY]:
                            if target[ServiceSpace.SL_LIST][0][ServiceLauncher.SL_NAME_KEY] != \
                                    key_mask[ServiceLauncher.SL_NAME_KEY]:
                                raise ValueError("Service launcher '%s' isn't managed by this platform node instance" %
                                                 key_mask[ServiceLauncher.SL_NAME_KEY])
                        elif ServiceLauncher.SL_NAME_KEY in ms_data and ms_data[ServiceLauncher.SL_NAME_KEY]:
                            if target[ServiceSpace.SL_LIST][0][ServiceLauncher.SL_NAME_KEY] != \
                                    ms_data[ServiceLauncher.SL_NAME_KEY]:
                                raise ValueError("Service launcher '%s' isn't managed by this platform node instance" %
                                                 ms_data[ServiceLauncher.SL_NAME_KEY])
                        if len(self.ms[key_mask[ServiceSpace.SS_ADDRESS_KEY]].keys()) > 1:
                            # there are other service spaces in this address and must be preserved
                            return self.ms[key_mask[ServiceSpace.SS_ADDRESS_KEY]]\
                                .pop(key_mask[ServiceSpace.SS_PORT_KEY])
                        else:
                            return self.ms.pop(key_mask[ServiceSpace.SS_ADDRESS_KEY])
                    # in all other cases we must delete only the element
                    idx = 0
                    for sl in target[ServiceSpace.SL_LIST]:
                        if ServiceLauncher.SL_NAME_KEY in key_mask and key_mask[ServiceLauncher.SL_NAME_KEY]:
                            if sl[ServiceLauncher.SL_NAME_KEY] == key_mask[ServiceLauncher.SL_NAME_KEY]:
                                return target[ServiceSpace.SL_LIST].pop(idx)
                        elif ServiceLauncher.SL_NAME_KEY in ms_data and ms_data[ServiceLauncher.SL_NAME_KEY]:
                            if sl[ServiceLauncher.SL_NAME_KEY] == ms_data[ServiceLauncher.SL_NAME_KEY]:
                                return target[ServiceSpace.SL_LIST].pop(idx)
                        idx += 1
                # if we arrive here is because we haven't found a launcher with the specified name
                raise ValueError("Service launcher '%s' isn't managed by this platform node instance" %
                                 key_mask[ServiceLauncher.SL_NAME_KEY])
        finally:
            self.ms_lock.release()
