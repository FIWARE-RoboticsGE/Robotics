#! /bin/sh

dest_rcmpd_path=/etc/init.d/rcmpd
ws_folder=$HOME/rcmp_ws
installer_folder=$(pwd)
ws_src_folder=$ws_folder/src
ros_folder=/opt/ros

install_rcm_component()
{
    rcm_component_path=$installer_folder/$1
    if [ -e "$rcm_component_path" ]
    then
        cp $rcm_component_path $1
        unzip $1
        rm $1
    else
        echo "The package $1 is not available between the components in $installer_folder"
        read -p "download the component and put it in the installer folder or provide the path: [$installer_folder]: " comp_folder
        if [ -z "$comp_folder" ]
        then
            comp_folder=$installer_folder
        else
            if [ -d "comp_folder" ]
            then
                rcm_component_path=$comp_folder/$1
                if [ -e "$rcm_component_path" ]
                then
                    cp $rcm_component_path $1
                    unzip $1
                    rm $1
                else
                    echo "The package $1 is not found in $comp_folder and must be installed manually"
                fi
            fi
        fi
    fi
}

if [ -e "$dest_rcmpd_path" ]
then
    echo "The old RCM daemon already exists: we trying to stop it"
    sudo service rcmpd stop
fi

is_tun0=$(ifconfig | grep tun0)
if [ ! -z "$is_tun0" ]
then
	# tun0 exists
	# creating the ros workspace for the rcm platform node
	if [ -d "$ws_folder" ]
	then
	    read -p "The workspace rcmp_ws already exists: do you wanna replace it ([Y]es/no)? " replace_ws
        case ${replace_ws} in
            n|no|N|No )
            ;;
            * )
                rm -r $ws_folder
                mkdir $ws_folder
            ;;
        esac
    else
        mkdir $ws_folder
	fi
	if [ ! -d "$ws_src_folder" ]
	then
		echo "creating source folder in rcmp workspace"
		mkdir $ws_src_folder
	fi
	# setting the ros environment variables
	if [ ! -d "$ros_folder" ]
	then
		echo "$ros_folder does not exist"
	fi
	ros_dist=$(ls $ros_folder)
    ros_folder=$ros_folder/$ros_dist
    echo "using ROS distribution $ros_dist"

	sudo ./sub_inst.sh $ros_folder

	# finishing the installation with operation relative to the workspace
	. $ros_folder/setup.sh

    cd $ws_src_folder
    echo "initializing rcmp workspace"
    catkin_init_workspace

    cd $ws_folder
    echo "compiling rcmp workspace"
    catkin_make
    . devel/setup.sh

    echo "configuration completed"

    read -p "Do you wanna start the RCM platform node now ([Y]es/no)? " start_rcm
    case ${start_rcm} in
        n|no|N|No )
        ;;
        * )
            sudo service rcmpd start
        ;;
    esac
else
	echo "The RCM platform works on VPN: no VPN detected; please configure correctly."
	exit 3
fi