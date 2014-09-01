Installation Beagle Bone Black {#installation-bbb}
===

\tableofcontents

\section Beschreibung Beschreibung

Auf dem Beagle Bone Black (BBB) wird ein Ubuntu in der Version 12.04 für
ARM-Prozessoren aufgesetzt. ROS wird in der der version __hydro__
verwendet. Weiterhin wird zur Kommunikation über den USB-Bus die
Bibliothek __libusb 1.0__ benötigt.

\section Installation Installation
\subsection Ubuntu Ubuntu 12.04

Um Ubuntu 12.04 auf dem BBB zu installieren, muss das Image auf die
Micro-SD Karte entpackt werden.

Die Micro-SD Karte muss mit dem PC verbunden werden. Danach müssen die
nachfolgenden Kommandos ausgeführt werden. Für __/dev/sdX__ im letzten
Schritt muss das entsprechende Gerät für die Micro-SD Karte eingesetzt
werden.

~~~
wget -O ubuntu-precise-12.04.3-armhf-3.8.13-bone30.img.xz http://s3.armhf.com/debian/precise/bone/ubuntu-precise-12.04.3-armhf-3.8.13-bone30.img.xz
sudo su
xz -cd ubuntu-precise-12.04.3-armhf-3.8.13-bone30.img.xz > /dev/sdX
~~~

Standard Benutzer und Passwort lauten wie folgt:
* Benutzer: __ubuntu__
* Passwort: __ubuntu__

\subsection Programme Grundlegende Programme

Die folgenden Programme müssen auf dem BBB installiert sein, um das
Projekt zu kompilieren.

~~~
sudo apt-get install git subversion build-essential libusb-1.0-0-dev vim ntp
~~~

\subsection ROS ROS

Um ROS auf dem BBB zu installieren, müssen die folgenden Kommandos
ausgeführt werden.

~~~
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
~~~

\section Konfiguration Konfiguration
\subsection Lego Lego NXT

Damit der Lego NXT Brick als USB-Gerät erkannt wird und somit auch die
Kommunikation zwischen Brick und BBB stattfinden kann, muss eine neue
Benutzergruppe erzeugt werden. Die Benutzer, die den Brick nutzen
möchten müssen Teil dieser Gruppe sein.

~~~
sudo su
groupadd lego
usermod -a -G lego <username>
touch /etc/udev/rules.d/70-lego.rules
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="0694", GROUP="lego", MODE="0660"' > /etc/udev/rules.d/70-lego.rules
~~~

Nun muss das BBB neugestartet werden.

\subsection WLAN WLAN

Folgendes muss in die Datei `/etc/network/interfaces` eingefügt werden.
Wenn dort bereits ein Adapter namens wlan0 existiert, sollte dieser
überschrieben werden.

~~~
auto wlan0
iface wlan0 inet static
wpa-ssid "yourSSID"
wpa-psk "yourWPA-PSK-Key"
address 192.168.11.143
netmask 255.255.255.0
gateway 192.168.11.1
~~~

\subsection SPI SPI Device Tree

This page explains how to setup SPI0 and SPI1 on BBB with the device tree.

The script installs the overlay files for '''SPI0 (1 chip select)''' and '''SPI1 (2 chips selects)'''.

'''Only for angstrom linux'''
Warning! On the newest angstrom image there is no dtc in opkg available.
A simple solution is to create the compiled dtbo file on a desktop ubuntu machine
and copy it to the BBB.

* Clone the bbb_setup repository
* Install dtc (device tree compiler)

~~~
sudo ./dtc.sh
~~~

* Run the addOverlayToDeviceTree script (compiles and installs SPI0 and SPI1 overlay file)

~~~
sudo ./addOverlayToDeviceTree
~~~

* To enable SPI1 on startup (and disable HDMI which uses SPI1 pins)
* Mount boot partition

~~~
mkdir /mnt/boot
mount /dev/mmcblk0p1 /mnt/boot
~~~

* Edit uEnv.txt file on boot partition

~~~
optargs=quiet drm.debug=7 capemgr.disable_partno=BB-BONELT-HDMI,BB-BONELT-HDMIN capemgr.enable_partno=MINOTAUR-SPI1
~~~

* Reboot bbb

~~~
sudo reboot
~~~
