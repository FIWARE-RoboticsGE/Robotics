__author__ = "Fabio Giuseppe Di Benedetto"

import sqlite3
import os.path
from rcmp_node import logging, RCMP_LOGGER_NAME, RCMPlatformNode


class RCMPRoboticsDataManager:
    """The database manager for rcm platform robotics data."""
    DEFAULT_DB_DIR = "/opt/rcm-platform/"
    DEFAULT_DB_NAME = "robotics_data.db"
    DEFAULT_DB_PATH = os.path.join(DEFAULT_DB_DIR, DEFAULT_DB_NAME)
    # TODO keep here until service logic has its manager of some type
    DEFAULT_SERVICE_LOGIC = "empty"

    def __init__(self, db_path=DEFAULT_DB_PATH):
        self.db_path = db_path
        self.db = None
        self.crs = None
        # ip_address can be null for enabling 2-step provisioning
        self.__ct_platform_instances_sql = "create table platform_instances(id INTEGER PRIMARY KEY, " \
                                           "name VARCHAR(20) UNIQUE NOT NULL, ip_address VARCHAR(15), " \
                                           "type VARCHAR(1), load INTEGER, " \
                                           "s_logic INTEGER REFERENCES service_logics(id), " \
                                           "free_inbound_ports VARCHAR(20))"
        self.__dt_platform_instances_sql = "drop table platform_instances"
        self.__iit_platform_instances_sql = "insert into platform_instances(name, ip_address, type, load) " \
                                            "values (?, ?, ?, ?)"
        self.__iit_platform_instances_with_ifp_sql = "insert into platform_instances(name, ip_address, type, load, " \
                                                     "free_inbound_ports) values (?, ?, ?, ?, ?)"
        self.__iit_platform_instances_with_sl_sql = "insert into platform_instances(name, ip_address, type, load, " \
                                                    "s_logic) values (?, ?, ?, ?, (select id from service_logics " \
                                                    "where name=?))"
        self.__iit_platform_instances_with_sl_and_ifp_sql = "insert into platform_instances(name, ip_address, type, " \
                                                            "load, s_logic, free_inbound_ports) values (?, ?, ?, ?, " \
                                                            "(select id from service_logics where name=?), ?)"
        self.__ut_platform_instances_ip_address_sql = "update platform_instances set ip_address=? where name=?"
        self.__ut_platform_instances_ifp_sql = "update platform_instances set free_inbound_ports=? " \
                                               "where name=? and type=?"
        self.__dft_platform_instances_sql = "delete from platform_instances where name=?"
        self.__select_platform_instances_full_sql = "select name, type, ip_address, load, s_logic " \
                                                    "from platform_instances"
        self.__select_platform_instances_full_em_sql = "select name, type, ip_address, load, s_logic " \
                                                       "from platform_instances where name<>?"
        self.__select_platform_instances_sql = "select name, ip_address, s_logic from platform_instances " \
                                               "where type=?"
        self.__select_platform_instances_load_ordered_sql = "select name, ip_address from platform_instances " \
                                                            "where type=? order by load asc"
        self.__select_pi_ip_address_sql = "select ip_address, type from platform_instances where name=?"
        self.__select_pi_ifp_sql = "select free_inbound_ports from platform_instances where name=?"
        self.__ct_service_spaces_sql = "create table service_spaces(id INTEGER PRIMARY KEY, " \
                                       "name VARCHAR(20) UNIQUE NOT NULL, " \
                                       "p_instance INTEGER REFERENCES platform_instances(id) NOT NULL, " \
                                       "port INTEGER NOT NULL, used_inbound_port INTEGER)"
        self.__dt_service_spaces_sql = "drop table service_spaces"
        self.__iit_service_spaces_sql = "insert into service_spaces(name, p_instance, port) values (?, " \
                                        "(select id from platform_instances where name=?), ?)"
        self.__iit_service_spaces_using_ip_sql = "insert into service_spaces(name, p_instance, port) values (?, " \
                                                 "(select id from platform_instances where ip_address=?), ?)"
        self.__ut_service_spaces_uip_sql = "update service_spaces set used_inbound_port=? where name=?"
        self.__dft_service_spaces_from_pi_sql = "delete from service_spaces where p_instance=" \
                                                "(select id from platform_instances where name=?)"
        self.__dft_service_spaces_sql = "delete from service_spaces where " \
                                        "p_instance=(select id from platform_instances where ip_address=?) and port=?"
        self.__dft_service_spaces_from_name_sql = "delete from service_spaces where name=?"
        self.__select_ss_pi_sql = "select ss.name, ss.port, used_inbound_port from service_spaces ss inner join platform_instances pi " \
                                  "on pi.id=ss.p_instance where pi.name=?"
        self.__select_ss_names_sql = "select name from service_spaces"
        self.__select_ss_ip_address_port_sql = "select pi.ip_address, ss.port from service_spaces ss " \
                                               "inner join platform_instances pi on pi.id=ss.p_instance " \
                                               "where ss.name=?"
        self.__select_last_ss_port_sql = "select max(ss.port) from service_spaces ss " \
                                         "inner join platform_instances pi on pi.id=ss.p_instance " \
                                         "where pi.ip_address=?"
        self.__select_ss_external_access_sql = "select used_inbound_port from service_spaces where name=?"
        self.__select_used_inbound_ports_from_pi_name_sql = "select used_inbound_port from service_spaces ss " \
                                                            "inner join platform_instances pi " \
                                                            "on pi.id=ss.p_instance where pi.name=?"
        self.__ct_connections_sql = "create table connections(id INTEGER PRIMARY KEY, " \
                                    "p_instance_s INTEGER REFERENCES platform_instances(id), " \
                                    "p_instance_r INTEGER REFERENCES platform_instances(id), " \
                                    "s_space INTEGER REFERENCES service_spaces(id))"
        self.__iit_connections_sql = "insert into connections(p_instance_S, p_instance_R) values (" \
                                     "(select id from platform_instances where name=?), " \
                                     "(select id from platform_instances where name=?))"
        self.__iit_connections_with_ss_sql = "insert into connections(p_instance_S, p_instance_R, s_space) values (" \
                                             "(select id from platform_instances where name=?), " \
                                             "(select id from platform_instances where name=?), " \
                                             "(select id from service_spaces where name=?))"
        # self.__select_cnn_from_pi_r_sql = "select pi_s.name, ss.name from connections cnn " \
        #                                   "inner join platform_instances pi_s on pi_s.id=cnn.p_instance_s " \
        #                                   "inner join service_spaces ss on cnn.s_space=ss.id " \
        #                                   "where cnn.p_instance_r=(select id from platform_instances where name=?)"
        self.__select_cnn_from_pi_r_sql = "select pi_s.name from connections cnn " \
                                          "inner join platform_instances pi_s on pi_s.id=cnn.p_instance_s " \
                                          "where cnn.p_instance_r=(select id from platform_instances where name=?)"
        # self.__select_cnn_from_pi_s_sql = "select pi_r.name, ss.name from connections cnn " \
        #                                   "inner join platform_instances pi_r on pi_r.id=cnn.p_instance_r " \
        #                                   "inner join service_spaces ss on cnn.s_space=ss.id " \
        #                                   "where cnn.p_instance_s=(select id from platform_instances where name=?)"
        self.__select_cnn_from_pi_s_sql = "select pi_r.name from connections cnn " \
                                          "inner join platform_instances pi_r on pi_r.id=cnn.p_instance_r " \
                                          "where cnn.p_instance_s=(select id from platform_instances where name=?)"
        self.__select_cnn_ss_name_sql = "select ss.name from service_spaces ss " \
                                        "where ss.id=(select s_space from connections " \
                                        "where p_instance_s=(select id from platform_instances where name=?) and " \
                                        "p_instance_r=(select id from platform_instances where name=?))"
        # TODO see if we could keep this way
        # even if it is considered one of the call for connection it isn't because the robot has only one server
        self.__select_cnn_pi_r_sl_sql = "select sl.name, sl.ss_pi_type_target, sl.ss_name from service_logics sl " \
                                        "inner join platform_instances pi_r on sl.id=pi_r.s_logic and pi_r.name=?"
        self.__dft_connections_sql = "delete from connections where p_instance_r=(select id from platform_instances " \
                                     "where name=?)"
        self.__dt_connections_sql = "drop table connections"
        self.__ct_service_logics_sql = "create table service_logics(id INTEGER PRIMARY KEY, " \
                                       "name VARCHAR(20), ss_pi_type_target VARCHAR(1), ss_name_suffix VARCHAR(20))"
        self.__iit_service_logics_sql = "insert into service_logics(name, ss_pi_type_target) values (?, ?)"
        self.__select_sl_name_sql = "select name from service_logics where id=?"
        self.__select_sl_id_sql = "select id from service_logics where name=?"
        self.__select_sl_ss_pi_type_target_sql = "select sl.ss_pi_type_target from service_logics sl " \
                                                 "inner join platform_instances pi_r on pi_r.s_logic=sl.id " \
                                                 "where pi_r.name=?"
        # the same as before but using the service logic name instead the platform instance
        self.__select_sl_ss_side_sql = "select ss_pi_type_target from service_logics where name=?"
        self.__select_service_logics_sql = "select name from service_logics"
        self.__select_sl_uses_count_sql = "select count(pi.id) from platform_instances pi " \
                                          "inner join service_logics sl on pi.s_logic=sl.id where sl.name=?"
        self.__dft_service_logics_sql = "delete from service_logics where name=?"
        self.__dt_service_logics_sql = "drop table service_logics"
        # TODO not used yet
        self.__ct_service_items_sql = "create table service_items(id INTEGER PRIMARY KEY, package VARCHAR(20), " \
                                      "tag VARCHAR(20), type VARCHAR(1))"
        self.__iit_service_items_sql = "insert into service_items(package, tag, type) values (?, ?, ?)"
        self.__select_service_item_sql = "select id from service_items where package=? and tag=?"
        self.__dft_service_items_sql = "delete from service_items where id=?"
        self.__dt_service_items_sql = "drop table service_items"
        self.__ct_service_logic_items_sql = "create table service_logic_items(id INTEGER PRIMARY KEY, " \
                                            "s_logic INTEGER REFERENCES service_logics(id), " \
                                            "s_item INTEGER REFERENCES service_items(id), " \
                                            "si_pi_type_target VARCHAR(1), si_name VARCHAR(20), si_params VARCHAR(20))"
        self.__iit_service_logic_items_sql = "insert into service_logic_items(s_logic, s_item, si_pi_type_target, " \
                                             "si_name, si_params) values ((select id from service_logics " \
                                             "where name=?), (select id from service_items " \
                                             "where package=? and tag=?), ?, ?, ?)"
        self.__select_sli_sql = "select si.package, si.tag, si.type, sli.si_pi_type_target, sli.si_name, " \
                                "sli.si_params from service_logic_items sli inner join service_items si " \
                                "on si.id=sli.s_item inner join platform_instances pi_r " \
                                "on pi_r.s_logic=sli.s_logic where pi_r.name=?"
        self.__select_sli_from_sl_name_sql = "select si.package, si.tag, si.type, sli.si_pi_type_target, " \
                                             "sli.si_name, sli.si_params from service_logic_items sli " \
                                             "inner join service_items si on si.id=sli.s_item " \
                                             "inner join service_logics sl on sl.id=sli.s_logic where sl.name=?"
        self.__select_si_id_from_sl_name_sql = "select sli.s_item from service_logic_items sli " \
                                               "inner join service_logics sl on sl.id=sli.s_logic where sl.name=?"
        self.__select_si_uses_from_id_sql = "select count(s_logic) from service_logic_items where s_item=?"
        self.__dft_service_logic_items_sql = "delete from service_logic_items " \
                                             "where s_logic=(select id from service_logics where name=?) and s_item=?"
        self.__dt_service_logic_items_sql = "drop table service_logic_items"
        self._logger = logging.getLogger(RCMP_LOGGER_NAME)

    def is_rd_available(self):
        """Check if the database is available. If available this platform node is the master."""
        return os.path.exists(self.db_path)

    def connect(self):
        """Connect to the database. Return False if the connection went wrong or the database
        is not available (not the master platform node)."""
        return self.__connect()

    def __connect(self, force=False):
        # the parameter 'force' is used to force the connection: the sqlite3.connect
        # create the file if it doesn't exist so forcing the connection means create
        # the database and this must be done only in the master platform node (when
        # it calls init_db)
        ret = False
        if force or self.is_rd_available():
            self.db = sqlite3.connect(self.db_path)
            self.crs = self.db.cursor()
            ret = True
        return ret

    def init_db(self, pni):
        """Initialize the database. Only the master can have robotics data so you can
        use RCMPRoboticsDataManager module in other platform nodes, but the only
        functionality actually available in those nodes is the check of the database
        availability.
        This is the method used to create a robotics data database when you are on
        a master platform node."""
        available_tables_sql = "select name from sqlite_master where type='table'"
        try:
            self.__connect(True)
            self.crs.execute(available_tables_sql)
            rows = self.crs.fetchall()
            tables_available = 0
            if rows:
                for c_row in rows:
                    if "platform_instances" or "service_spaces" or "connections" or \
                            "service_logics" or "service_logic_items" or "service_items" in c_row:
                        tables_available += 1
            if tables_available != 6:
                self._create_all_tables()
                # the initialization create only the platform instance (the master information
                # with the name of the platform node and the address): the type and load are
                # forced respectively to S and 0 when we initialized the RCMPlatformNode
                self.insert_platform_instance(pni[RCMPlatformNode.PNI_NAME], pni[RCMPlatformNode.PNI_ADDRESS],
                                              pni[RCMPlatformNode.PNI_TYPE], pni[RCMPlatformNode.PNI_LOAD],
                                              ifp_list=pni[RCMPlatformNode.PNI_INBOUND_OPEN_PORTS])
                self.__populate_custom_service_logics()
            else:
                # this is the case in which the master re-runs: in this case is
                # better update the db with the address in case it is changed
                self.delete_service_spaces_from_pi(pni[RCMPlatformNode.PNI_NAME])
                self.update_ip_address_into_platform_instance(pni[RCMPlatformNode.PNI_ADDRESS],
                                                              pni[RCMPlatformNode.PNI_NAME])
        finally:
            self.disconnect()

    def __populate_custom_service_logics(self):
        from rcmp_service_command import ServiceRCMPCommandAgent
        # no service space created
        self.insert_service_logic(self.DEFAULT_SERVICE_LOGIC, None)
        # it creates a service space and an heavy node on server side and
        # a lite node on robot side
        self.insert_service_logic("base", RCMPlatformNode.S_TYPE)
        # it creates a service space server side and no nodes or launchers
        self.insert_service_logic("fiware_demo1", RCMPlatformNode.S_TYPE)
        # it creates a service space and fiware nodes (rcmdriver, firos) on
        # server side and nothing on robot side
        self.insert_service_logic("fiware_base", RCMPlatformNode.S_TYPE)
        # it creates a service space and fiware nodes (with rcmdriver) on
        # server side and turtlebot launchers on robot side
        self.insert_service_logic("fiware_demo2", RCMPlatformNode.S_TYPE)
        # it creates a service space and fiware nodes on server side and
        # turtlebot launchers and kurento client on robot side
        self.insert_service_logic("fiware_livedemo", RCMPlatformNode.S_TYPE)
        #
        self.insert_service_item("turtlebot_bringup", "minimal.launch", ServiceRCMPCommandAgent.SLI_L_TYPE)
        self.insert_service_item("turtlebot_bringup", "3dsensor.launch", ServiceRCMPCommandAgent.SLI_L_TYPE)
        self.insert_service_item("firos", "core.py", ServiceRCMPCommandAgent.SLI_N_TYPE)
        self.insert_service_item("rcm", "rcmdriver.py", ServiceRCMPCommandAgent.SLI_N_TYPE)
        self.insert_service_item("webrtc", "webRTC_master.py", ServiceRCMPCommandAgent.SLI_N_TYPE)
        #
        self.insert_service_logic_item("fiware_base", "rcm", "rcmdriver.py", RCMPlatformNode.S_TYPE, "", "")
        self.insert_service_logic_item("fiware_base", "firos", "core.py", RCMPlatformNode.S_TYPE, "", "")
        #
        self.insert_service_logic_item("fiware_demo2", "turtlebot_bringup", "minimal.launch",
                                       RCMPlatformNode.R_TYPE, "", "")
        # self.insert_service_logic_item("fiware_demo2", "turtlebot_bringup", "3dsensor.launch",
        #                                RCMPlatformNode.R_TYPE, "", "")
        self.insert_service_logic_item("fiware_demo2", "rcm", "rcmdriver.py", RCMPlatformNode.S_TYPE, "", "")
        self.insert_service_logic_item("fiware_demo2", "firos", "core.py", RCMPlatformNode.S_TYPE, "", "")
        #
        self.insert_service_logic_item("fiware_livedemo", "turtlebot_bringup", "minimal.launch",
                                       RCMPlatformNode.R_TYPE, "", "")
        # self.insert_service_logic_item("fiware_livedemo", "turtlebot_bringup", "3dsensor.launch",
        #                                RCMPlatformNode.R_TYPE, "", "")
        self.insert_service_logic_item("fiware_livedemo", "webrtc", "webRTC_master.py",
                                       RCMPlatformNode.R_TYPE, "", "address=188.15.109.226:8080")
        self.insert_service_logic_item("fiware_livedemo", "rcm", "rcmdriver.py", RCMPlatformNode.S_TYPE, "", "")
        self.insert_service_logic_item("fiware_livedemo", "firos", "core.py",
                                       RCMPlatformNode.S_TYPE, "", "")

    def _create_all_tables(self):
        # create all the tables in the db
        self.__create_platform_instances()
        self.__create_service_spaces()
        self.__create_connections()
        self.__create_service_logics()
        self.__create_service_items()
        self.__create_service_logic_items()

    # about platform instances

    def __create_platform_instances(self):
        # create platform_instances(id, name, ip_address)
        self.crs.execute(self.__ct_platform_instances_sql)
        self.db.commit()

    def insert_platform_instance(self, name, ip_address, i_type, load=None, s_logic=None, ifp_list=None):
        """Insert a new platform instance in the database."""
        if ifp_list is None:
            if i_type == RCMPlatformNode.S_TYPE:
                self.crs.execute(self.__iit_platform_instances_sql, (name, ip_address, i_type, load))
            else:
                if s_logic is None:
                    s_logic = self.DEFAULT_SERVICE_LOGIC
                else:
                    pass
                self.crs.execute(self.__iit_platform_instances_with_sl_sql, (name, ip_address, i_type, load, s_logic))
        else:
            if i_type == RCMPlatformNode.S_TYPE:
                self.crs.execute(self.__iit_platform_instances_with_ifp_sql, (name, ip_address, i_type,
                                                                              load, ifp_list))
            else:
                if s_logic is None:
                    s_logic = self.DEFAULT_SERVICE_LOGIC
                else:
                    pass
                self.crs.execute(self.__iit_platform_instances_with_sl_and_ifp_sql, (name, ip_address, i_type,
                                                                                     load, s_logic, ifp_list))
        self.db.commit()

    def update_ip_address_into_platform_instance(self, ip_address, name):
        """Update the ip address of a platform instance in the database."""
        self.crs.execute(self.__ut_platform_instances_ip_address_sql, (ip_address, name))
        self.db.commit()

    # TODO not used yet
    def update_ifp_list_into_platform_instance(self, free_inbound_ports, name, pi_type):
        """Update the inbound free port list of a platform instance in the database."""
        self.crs.execute(self.__ut_platform_instances_ifp_sql, (free_inbound_ports, name, pi_type))
        self.db.commit()

    def get_platform_instance_list(self, full=False, exclude_master=False):
        """Get all the platform instances."""
        if full:
            if exclude_master:
                self.crs.execute(self.__select_platform_instances_full_em_sql,
                                 (RCMPlatformNode.DEFAULT_MASTER_NAME, ))
            else:
                self.crs.execute(self.__select_platform_instances_full_sql)
        else:
            self.crs.execute(self.__select_platform_instances_sql, (RCMPlatformNode.R_TYPE, ))
        return self.crs.fetchall()

    def get_unloaded_s_pi(self):
        """Get the platform instances of type S sorted ascending from the less loaded."""
        self.crs.execute(self.__select_platform_instances_load_ordered_sql, (RCMPlatformNode.S_TYPE, ))
        return self.crs.fetchall()

    def get_pi_ip_address_from_name(self, name):
        """Get the ip address corresponding to a platform instance name."""
        self.crs.execute(self.__select_pi_ip_address_sql, (name, ))
        return self.crs.fetchone()

    def get_inbound_free_port(self, name, ss_name):
        ifp = None
        uip = self.get_ss_external_access_used(ss_name)
        if uip and uip[0]:
            raise ValueError("The only inbound port spot available for the service space '%s' "
                             "is already used by another service on %d" % (ss_name, uip[0]))
        self.crs.execute(self.__select_used_inbound_ports_from_pi_name_sql, (name, ))
        used_inbound_ports = self.crs.fetchall()
        self.crs.execute(self.__select_pi_ifp_sql, (name, ))
        row = self.crs.fetchone()
        if row and row[0]:
            ifp_range = row[0].split(",")
            used = False
            for ifp_item in ifp_range:
                ifp = ifp_item.strip()
                if used_inbound_ports:
                    for used_inbound_port in used_inbound_ports:
                        if used_inbound_port[0]:
                            if ifp == str(used_inbound_port[0]):
                                # the ifp in the range is already used we have to go to the next in the range
                                used = True
                                break
                if used:
                    # reset the variables for the next port check
                    used = False
                    ifp = None
                else:
                    # the inbound free port is not used by other monitored process
                    break
        return ifp

    def delete_platform_instance(self, name, full=False):
        """Delete a platform instance. Parameter 'full' is used to delete
        the whole entry and not only the ip address associated with the
        platform instance."""
        if full:
            # we delete the whole entry
            self.crs.execute(self.__dft_platform_instances_sql, (name, ))
        else:
            # we want delete only the ip address
            self.crs.execute(self.__ut_platform_instances_ip_address_sql, (None, name))
        self.db.commit()

    # about service spaces

    def __create_service_spaces(self):
        # create service_spaces(id, name, p_instance, port)
        self.crs.execute(self.__ct_service_spaces_sql)
        self.db.commit()

    def insert_service_space(self, name, p_name, port):
        """Insert a new service space in the database."""
        self.crs.execute(self.__iit_service_spaces_sql, (name, p_name, port))
        self.db.commit()

    def insert_service_space_with_next_port(self, name, ip_address):
        """Insert a new service space in the database. The new service space will be with the next
        port available for the machine at the specified ip_address."""
        # take the last used port for the machine at the specified ip_address
        self.crs.execute(self.__select_last_ss_port_sql, (ip_address, ))
        lp_row = self.crs.fetchone()
        if lp_row and lp_row[0]:
            # first port not used yet
            port = lp_row[0] + 1
        else:
            # default starting port number
            port = 11311
        self.crs.execute(self.__iit_service_spaces_using_ip_sql, (name, ip_address, port))
        self.db.commit()
        return port

    def get_service_space_list_from_pi(self, pi_name):
        """Get all the service spaces of the platform instance with the passed name."""
        self.crs.execute(self.__select_ss_pi_sql, (pi_name, ))
        return self.crs.fetchall()

    def get_all_service_space_names(self):
        """Get all the names of service spaces in robotics data."""
        self.crs.execute(self.__select_ss_names_sql)
        return self.crs.fetchall()

    def get_ss_ip_address_port(self, name):
        """Get the ip address and port corresponding to a service space name."""
        self.crs.execute(self.__select_ss_ip_address_port_sql, (name, ))
        return self.crs.fetchone()

    def get_ss_external_access_used(self, name):
        """Get ip address and port for external access in the service space corresponding to the name."""
        self.crs.execute(self.__select_ss_external_access_sql, (name, ))
        return self.crs.fetchone()

    def delete_service_spaces_from_pi(self, pi_name):
        """Delete all service spaces associated with a no more available platform
        instance."""
        self.crs.execute(self.__dft_service_spaces_from_pi_sql, (pi_name, ))
        self.db.commit()

    def delete_service_space(self, ip_address, port):
        """Delete a service space connected to the specified ip address of
        the instance platform and with the specified port."""
        self.crs.execute(self.__dft_service_spaces_sql, (ip_address, port))
        self.db.commit()

    def delete_service_space_from_name(self, ss_name):
        """Delete the service space with the specified name."""
        self.crs.execute(self.__dft_service_spaces_from_name_sql, (ss_name, ))
        self.db.commit()

    def update_used_inbound_port(self, ss_name, port):
        self.crs.execute(self.__ut_service_spaces_uip_sql, (int(port), ss_name))
        self.db.commit()

    # about connections

    def __create_connections(self):
        # create connections(id, p_instance_S, p_instance_R, s_space)
        self.crs.execute(self.__ct_connections_sql)
        self.db.commit()

    def insert_connection(self, pi_s_name, pi_r_name, ss_name):
        """Insert a new connection between S instance and R instance in the database."""
        if ss_name is None:
            self.crs.execute(self.__iit_connections_sql, (pi_s_name, pi_r_name))
        else:
            self.crs.execute(self.__iit_connections_with_ss_sql, (pi_s_name, pi_r_name, ss_name))
        self.db.commit()

    # def get_pi_s_name_from_connection_pi_r(self, pi_r_name):
    #     """Get the name of the platform node of type S paired with the platform node
    #      of type R with the passed name."""
    #     # TODO this is wrong if the robot can be paired with more than one server
    #     self.crs.execute(self.__select_cnn_pi_s_name_sql, (pi_r_name, ))
    #     return self.crs.fetchone()

    def get_connection_from_connection_pi_r(self, pi_r_name):
        """Get the name of the platform node of type S paired with the platform node
         of type R with the passed name and the service space used by this connection."""
        self.crs.execute(self.__select_cnn_from_pi_r_sql, (pi_r_name, ))
        # TODO this is wrong if the robot can be paired with more than one server
        return self.crs.fetchone()

    def get_connection_from_connection_pi_s(self, pi_s_name):
        """Get the name of the platform nodes of type R paired with the platform node
         of type S with the passed name and the service spaces used by each connection."""
        self.crs.execute(self.__select_cnn_from_pi_s_sql, (pi_s_name, ))
        return self.crs.fetchall()

    def get_pi_r_sl_from_connection_pi_r(self, pi_r_name):
        """Get the service logic of the platform node of type R."""
        # TODO actually is useless to pass through the connection table
        # but this depends if the robot can be paired only with one server
        # or if it doesn't;
        self.crs.execute(self.__select_cnn_pi_r_sl_sql, (pi_r_name, ))
        return self.crs.fetchone()

    def get_ss_name_from_connection(self, pi_s_name, pi_r_name):
        """Get the service space name associated with the connection between a platform
        node of type S and the platform node of type R with the specified name."""
        self.crs.execute(self.__select_cnn_ss_name_sql, (pi_s_name, pi_r_name))
        return self.crs.fetchone()

    def delete_connection(self, pi_r_name):
        """Delete a connection between a platform node of type R specified through the name
        and a platform node of type S."""
        self.crs.execute(self.__dft_connections_sql, (pi_r_name, ))
        self.db.commit()

    # about service logic

    def __create_service_logics(self):
        # create service_logics(id, name, ss_pi_type_target)
        self.crs.execute(self.__ct_service_logics_sql)
        self.db.commit()

    def insert_service_logic(self, name, ss_pi_type):
        """Insert a new service logic in the database."""
        self.crs.execute(self.__iit_service_logics_sql, (name, ss_pi_type))
        self.db.commit()

    def get_sl_id_from_name(self, name):
        """Get the service logic id starting from the service logic name."""
        self.crs.execute(self.__select_sl_id_sql, (name, ))
        return self.crs.fetchone()

    def get_sl_name_from_id(self, sl_id):
        """Get the service logic name starting from the service logic id."""
        self.crs.execute(self.__select_sl_name_sql, (sl_id, ))
        return self.crs.fetchone()

    def get_sl_ss_pi_type_target_from_name(self, name):
        """Get the platform instance type where the service space will be started for
        the service logic. It uses the name of the platform instance using the
        service logic."""
        self.crs.execute(self.__select_sl_ss_pi_type_target_sql, (name, ))
        return self.crs.fetchone()

    def get_sl_ss_pi_type_target_from_sl_name(self, name):
        """Get the platform instance type where the service space will be started for
        the service logic. It uses the service logic name."""
        self.crs.execute(self.__select_sl_ss_side_sql, (name, ))
        return self.crs.fetchone()

    def get_service_logic_list(self):
        """Get all the service logics."""
        self.crs.execute(self.__select_service_logics_sql)
        return self.crs.fetchall()

    def get_sl_uses(self, name):
        self.crs.execute(self.__select_sl_uses_count_sql, (name, ))
        return self.crs.fetchone()

    def delete_service_logic(self, name):
        """Delete the service logic with the name specified."""
        self.crs.execute(self.__dft_service_logics_sql, (name, ))
        self.db.commit()

    # about service items

    def __create_service_items(self):
        # create service_items(id, package, tag, type)
        self.crs.execute(self.__ct_service_items_sql)
        self.db.commit()

    def insert_service_item(self, package, tag, si_type):
        """Insert a new service item (launcher, node) for the service logic in the database."""
        self.crs.execute(self.__iit_service_items_sql, (package, tag, si_type))
        self.db.commit()

    def get_service_item(self, package, tag):
        """Get the id of the service item (launcher, node)."""
        self.crs.execute(self.__select_service_item_sql, (package, tag))
        return self.crs.fetchone()

    def get_service_items_from_sl_name(self, name):
        """Get the list of ids of service items associated with the specified service logic name."""
        self.crs.execute(self.__select_si_id_from_sl_name_sql, (name, ))
        return self.crs.fetchall()

    def get_service_item_uses(self, si_id):
        """Get the number of service logics using the service item identified by the specified id."""
        self.crs.execute(self.__select_si_uses_from_id_sql, (si_id, ))
        return self.crs.fetchone()

    def delete_service_item(self, si_id):
        """Delete the service item with the id specified."""
        self.crs.execute(self.__dft_service_items_sql, (si_id, ))
        self.db.commit()

    # about service logic items

    def __create_service_logic_items(self):
        # create service_logic_items(id, s_logic, s_item, si_pi_type_target, si_name, si_params)
        self.crs.execute(self.__ct_service_logic_items_sql)
        self.db.commit()

    def insert_service_logic_item(self, s_logic_name, si_package, si_tag, si_pi_type, si_name, si_params):
        """Insert a new service logic item in the database."""
        self.crs.execute(self.__iit_service_logic_items_sql, (s_logic_name, si_package, si_tag,
                                                              si_pi_type, si_name, si_params))
        self.db.commit()

    def get_sli_from_connection_pi_r(self, pi_r_name):
        """Get the service logic items of the platform node of type R."""
        # but this depends if the robot can be paired only with one server
        # or if it doesn't;
        self.crs.execute(self.__select_sli_sql, (pi_r_name, ))
        return self.crs.fetchall()

    def get_sli_from_sl_name(self, name):
        """Get the service logic items associated with the service logic with the specified name."""
        self.crs.execute(self.__select_sli_from_sl_name_sql, (name, ))
        return self.crs.fetchall()

    def delete_service_logic_item(self, sl_name, si_id):
        """Delete the service item with the id specified."""
        self.crs.execute(self.__dft_service_logic_items_sql, (sl_name, si_id))
        self.db.commit()

    def _drop_all_tables(self):
        # drop all the tables in the db
        self.__drop_platform_instances()
        self.__drop_service_spaces()
        self.__drop_connections()
        self.__drop_service_logics()
        self.__drop_service_items()
        self.__drop_service_logic_items()

    def __drop_platform_instances(self):
        # drop platform_instances(id, name, ip_address)
        self.crs.execute(self.__dt_platform_instances_sql)
        self.db.commit()

    def __drop_service_spaces(self):
        # drop service_spaces(id, name, p_instance, port)
        self.crs.execute(self.__dt_service_spaces_sql)
        self.db.commit()

    def __drop_connections(self):
        # drop connections(id, p_instance_S, p_instance_R, s_space)
        self.crs.execute(self.__dt_connections_sql)
        self.db.commit()

    def __drop_service_logics(self):
        # drop service_logics(id, name, ip_address)
        self.crs.execute(self.__dt_service_logics_sql)
        self.db.commit()

    def __drop_service_items(self):
        # drop service_items(id, package, name, type)
        self.crs.execute(self.__dt_service_items_sql)
        self.db.commit()

    def __drop_service_logic_items(self):
        # drop service_logic_items(id, s_logic, s_item, si_pi_type_target, si_name, si_params)
        self.crs.execute(self.__dt_service_logic_items_sql)
        self.db.commit()

    def disconnect(self):
        """Disconnect from the database."""
        if self.crs:
            self.crs.close()
        if self.db:
            self.db.close()


# --- GENERIC FUNCTIONS FOR ACCESSING ROBOTICS DATA ---

def execute_robotics_data_query(robotics_data_related_f, params=None, err_reason="Error"):
    """Execute a robotics data query. Manage the connection to RCMPRoboticsDataManager
    and call the function robotics_data_related_f to know what to do with that. Return an RCMPMessage."""
    from rcmp_inter_communication import RCMPMessage
    rdm = RCMPRoboticsDataManager()
    try:
        rdm.connect()
        if params:
            result = robotics_data_related_f(rdm, params)
        else:
            result = robotics_data_related_f(rdm)
    except sqlite3.Error as se:
        reason = "%s: %s" % (err_reason, se)
        result = RCMPMessage()
        result.create_error_response(reason)
    finally:
        rdm.disconnect()
    return result


def execute_int_robotics_data_query(robotics_data_related_f, params=None, err_reason="Error"):
    """Execute a robotics data query. Manage the connection to RCMPRoboticsDataManager
    and call the function robotics_data_related_f to know what to do with that. Return an object
    different from RCMPMessage and is used for internal purposes."""
    rdm = RCMPRoboticsDataManager()
    try:
        rdm.connect()
        if params:
            res = robotics_data_related_f(rdm, params)
        else:
            res = robotics_data_related_f(rdm)
        return res
    except sqlite3.Error as se:
        reason = "%s: %s" % (err_reason, se)
        raise IOError(reason)
    finally:
        rdm.disconnect()
