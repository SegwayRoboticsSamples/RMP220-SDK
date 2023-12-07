#!/bin/bash

echo -e "\nSegway_RMP_Init.sh File Version v1.0.0, 16 Nov 2023 edited.\n"
echo -e "Segway-Ninebot notice: If you are using RMP platform,Segway_RMP_Init.sh must be executed with administrator privileges first! The purpose is to select and set up the communication interface.\n"

read -p "Input your communication type, <UART> or <CAN> : " commu_type

if [ $commu_type == "UART" -o $commu_type == "CAN" ]
then
    echo -e "Communication type is $commu_type.\n"
else
    echo -e "Your input words is error,please try again!!!"
    exit 0
fi



cmd_uart(){

	read -p "Is software stty installed? <yes/no> : " install_result_1
	if [ $install_result_1 == "no" ]
	then
		echo -e "Install software stty first,please!\n"
		exit 0
	elif [ $install_result_1 == "yes" ]
	then
		echo -e
	else
		echo -e "Your input words is error,please try again!!!\n"
		exit 0
	fi


	echo -e "\nSegway-Ninebot notice: UART is set to the USB ID{10c4:ea60},you can also costomize changes.\n"

	echo -e "\nCreate rules files...\n"

	rpserialport="rpserialport.rules"
	cd /etc/udev/rules.d/

	if [ ! -f "$rpserialport" ]
	then    
	    touch $rpserialport
	    echo 'KERNEL=="ttyUSB[0-9]*",ATTRS{idVendor}=="10c4",ATTRS{idProduct}=="ea60",MODE:="0777",SYMLINK+="rpserialport"' > $rpserialport
	    echo "" >> $rpserialport
	    stty -F /dev/rpserialport 921600
	    echo "$rpserialport created"
	else
	    stty -F /dev/rpserialport 921600
	    echo "$rpserialport existed"
	fi

	sudo service udev reload
	sudo service udev restart
	
	echo -e "\nUART setting finish!!!\n"
}

cmd_can(){

	read -p "Are software busybox and modprobe installed? <yes/no> : " install_result_2
	if [ $install_result_2 == "no" ]
	then
		echo -e "Install software busybox and modprobe first,please!\n"
		exit 0
	elif [ $install_result_2 == "yes" ]
	then
		echo -e
	else
		echo -e "Your input words is error,please try again!!!\n"
		exit 0
	fi
	
    echo -e "\nSegway-Ninebot notice: CAN settings for the Nvidia Jetson platform (AGX Orin\Xavier NX series\AGX Xavier series),you can also costomize changes.More infomation refer to file detail please!\n"
	
	#busybox update can0 and can1 Pinmux value
	#CAN0 setting support Nvidia Jetson (AGX Orin\ Xavier NX series\ AGX Xavier series)
	##########################################################################################
	#sudo busybox devmem 0x0c303000 32 0x0000C400 #can1_dout
	#sudo busybox devmem 0x0c303008 32 0x0000C458 #can1_din
	sudo busybox devmem 0x0c303010 32 0x0000C400 #can0_dout
	sudo busybox devmem 0x0c303018 32 0x0000C458 #can0_din
	
	#load can module
	sudo modprobe can
	sudo modprobe can_raw
	sudo modprobe mttcan
	
	#change can setting
	sudo ip link set down can0
	sudo ip link set can0 type can bitrate 1000000  #baud rate=1MHz
	sudo ip link set up can0
	#sudo ip link set down can1
	#sudo ip link set can1 type can bitrate 1000000  #baud rate=1MHz
	#sudo ip link set up can1
	
	echo -e "\nCAN setting finish!!!\n"

}


if [ $commu_type == "UART" ]
then
   cmd_uart
elif [ $commu_type == "CAN" ]
then
   cmd_can
fi

