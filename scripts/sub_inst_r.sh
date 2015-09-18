#! /bin/sh

# path used
rcmp_folder=/opt/rcm-platform
log_folder=$rcmp_folder/log
conf_path=$rcmp_folder/init.cfg
master_conf_path=$rcmp_folder/.init.cfg
db_path=$rcmp_folder/robotics_data.db
dest_rcmpd_path=/etc/init.d/rcmpd

create_conf_file()
{
    echo "[main]" >> $conf_path
    default_instance_name=$(hostname)
    read -p "Enter the name of this RCM platform node [$default_instance_name]: " instance_name
    if [ -z "$instance_name" ]
    then
        instance_name=$default_instance_name
    fi
    echo "name=$instance_name" >> $conf_path
    read -p "Do you have ports opened for inbound communication that you want provide to this RCM platform node ([N]o/yes)? " iop_needed
    case ${iop_needed} in
        y|yes|Y|Yes )
            default_iop="10100,10101,10102,10103"
            read -p "Enter the list of ports opened for inbound communication separated by commas [$default_iop]: " iop
            if [ -z "$iop" ]
            then
                iop=$default_iop
            fi
            echo "inbound_open_ports=$iop" >> $conf_path
        ;;
        * )
        ;;
    esac
    echo "robot=yes" >> $conf_path
    # extract from tun0 the ip address
    default_ip_master=$(ifconfig tun0 | grep -o "inet addr:[0-9\.]*" | grep -o "[0-9\.]*")
    # substitute the last element of the address with 1 (the supposed vpn server and so the master)
    default_ip_master=$(echo $default_ip_master | grep -o "\([0-9]*.\)\{3\}")"1"
    read -p "Enter the ip address of the RCM platform node master [$default_ip_master]: " ip_master
    if [ -z "$ip_master" ]
    then
        ip_master=$default_ip_master
    fi
    echo "ip_master=$ip_master" >> $conf_path
}

create_non_master_conf()
{
    if [ -e "$db_path" ]
    then
        rm $db_path
    fi
    if [ -e "$master_conf_path" ]
    then
        rm $master_conf_path
    fi
    if [ -e "$conf_path" ]
    then
        read -p "An old version of the file configuration already exists: do you wanna replace it ([Y]es/no)? " replace_cf
        case ${replace_cf} in
            n|no|N|No )
            ;;
            * )
                rm $conf_path
                create_conf_file
            ;;
        esac
    else
        # no config file existing
        create_conf_file
    fi
}

echo "installing rcmplatform"
# rcmplatform must be executable (chmod +x rcmplatform) and in PATH to
# be visible from rcmpd
cp rcmp_n /usr/local/bin/.
# these files must be visible as python modules so must be in the
# python path
rm -r /usr/local/lib/python2.7/dist-packages/rcm_platform
cp -r rcm_platform /usr/local/lib/python2.7/dist-packages/.

if [ ! -d "$rcmp_folder" ]
then
	echo "creating rcmplatform folder"
	mkdir $rcmp_folder
fi

# a folder with the log configuration for ros (enable the stdout for rosout)
cp -r ros $rcmp_folder/.
cp LICENSE.txt $rcmp_folder/.

if [ ! -d "$log_folder" ]
then
	echo "creating rcmplatform log folder"
	mkdir $log_folder
fi

# this is not a master so if it was before, we have to delete the db
# pre installation of needed libraries
apt-get install python-netifaces -y
# apt-get install python-twisted -y
create_non_master_conf

read -p "RCM platform node works as root that cannot connect to X server and run nodes with graphical interface: enable working with GUI ([Y]es/no)? " is_gui_on
case ${is_gui_on} in
    n|no|N|No )
		sed "/<INSTALLER_XAUTHORITY>/d" <rcmpd >$dest_rcmpd_path
    ;;
    * )
        if [ -z "$XAUTHORITY" ]
		then
		    echo "XAUTHORITY not found"
			# when we use network tools like ssh this env variable is not set: in this
			# cases we use $HOME/.XAuthority
			installer_x_authority=$HOME/.Xauthority
        else
            installer_x_authority=$XAUTHORITY
		fi
		if [ -e "$installer_x_authority" ]
		then
		    echo "------- installer_x_authority exists -------"
		    sed "s|<INSTALLER_XAUTHORITY>|export XAUTHORITY=$installer_x_authority|" <rcmpd >$dest_rcmpd_path
		else
			sed "/<INSTALLER_XAUTHORITY>/d" <rcmpd >$dest_rcmpd_path
			echo "The .XAuthority file is not available: the RCM platform will be unable to run service nodes with graphical interfaces."
		fi
    ;;
esac

sed -i "s|<ROS_SETUP>|. $1/setup.sh|" $dest_rcmpd_path
sed -i "s|<RCMP_WS_SETUP>|. $HOME/rcmp_ws/devel/setup.sh|" $dest_rcmpd_path

# using sed to create the modified file means that it loses the permissions
chmod +x $dest_rcmpd_path
update-rc.d rcmpd defaults