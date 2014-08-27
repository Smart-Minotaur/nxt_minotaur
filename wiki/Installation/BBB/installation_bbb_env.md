Environment {#installation_bbb_env}
===========

* everything is only tested on '''Ubuntu 12.04 (Precise armhf)'''

* install '''build-essential'''


    sudo apt-get install build-essential


* install '''git'''


    sudo apt-get install git


* install '''screen'''


    sudo apt-get install screen


* set time (for compilation)


    sudo apt-get install ntp
    sudo su
    echo "sudo ntpdate -b -s -u ntp.ubuntu.com pool.ntp.org" > /etc/cron.daily/ntpdate
    echo "server pool.ntp.org" >> /etc/ntp.conf
    rm /etc/localtime
    ln -s /usr/share/zoneinfo/Europe/Berlin /etc/localtime
    reboot


* install ros hydro


    sudo update-locale LANG=C LANGUAGE=C LC_ALL=C LC_MESSAGES=POSIX
    sudo sh -c 'echo "deb http://packages.namniart.com/repos/ros precise main" > /etc/apt/sources.list.d/ros-latest.list'
    wget http://packages.namniart.com/repos/namniart.key -O - | sudo apt-key add -
    sudo apt-get update
    sudo apt-get install ros-hydro-ros-base
    sudo apt-get install python-rosdep
    sudo rosdep init
    rosdep update
    echo "source /opt/ros/hydro/setup.bash" >> ~/.bashrc
    source ~/.bashrc
    sudo apt-get install ros-hydro-move-base


* install '''libusb''' (used Version 1.0)


    sudo apt-get install libusb-1.0-0-dev


* create lego group


    sudo groupadd lego
    sudo usermod -a -G lego <username>
    sudo touch /etc/udev/rules.d/70-lego.rules
    sudo echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="0694", GROUP="lego", MODE="0660"' > /etc/udev/rules.d/70-lego.rules


* if last step says "permission denied", try


    sudo su
    sudo echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="0694", GROUP="lego", MODE="0660"' > /etc/udev/rules.d/70-lego.rules


* restart your Beaglebone Black


