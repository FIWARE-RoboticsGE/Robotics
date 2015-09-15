#! /bin/sh

dest_rcmpd_path=/etc/init.d/rcmpd
if [ -e "$dest_rcmpd_path" ]
then
    echo "The old RCM daemon already exists: we trying to stop it"
    sudo service rcmpd stop
fi

rcmp_workspace=~/rcmp_ws
if [ -d "$rcmp_workspace" ]
then
	echo "deleting rcmp workspace folder"
	rm -r $rcmp_workspace
fi

# doing this we overwrite the previous setting with the deleted workspace
sudo ./sub_uninst.sh