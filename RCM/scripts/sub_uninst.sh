#! /bin/sh

echo "uninstalling rcmplatform"
# daemon uninstallation
update-rc.d -f rcmpd remove
rm /etc/init.d/rcmpd
rm /usr/local/bin/rcmp_n

rcmp_modules_folder=/usr/local/lib/python2.7/dist-packages/rcm_platform
if [ -d "$rcmp_modules_folder" ]
then
	echo "deleting rcmplatform modules"
	rm -r $rcmp_modules_folder
fi

rcmp_folder=/opt/rcm-platform
if [ -d "$rcmp_folder" ]
then
	echo "deleting rcmplatform folder"
	rm -r $rcmp_folder
fi