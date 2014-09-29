Installation Beagle Bone Black {#installation-bbb}
===

\tableofcontents

\section beschreibung-installation-bbb Beschreibung

Auf dem Beagle Bone Black (BBB) wird ein Ubuntu in der Version 12.04 für
ARM-Prozessoren aufgesetzt. ROS wird in der der Version __hydro__
verwendet. Weiterhin wird zur Kommunikation über den USB-Bus die
Bibliothek __libusb 1.0__ benötigt.

---

\section installation-installation-bbb Installation
\subsection ubuntu-installation-bbb Ubuntu 12.04

Um Ubuntu 12.04 auf dem BBB zu installieren, muss das Image auf die
Micro-SD Karte entpackt werden.

Die Micro-SD Karte muss mit dem PC verbunden werden. Danach müssen die
nachfolgenden Kommandos ausgeführt werden. Für \b /dev/sdX im letzten
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

\subsection programme-installation-bbb Grundlegende Programme

Die folgenden Programme müssen auf dem BBB installiert sein, um das
Projekt zu kompilieren.

~~~
sudo apt-get install git subversion build-essential libusb-1.0-0-dev vim ntp
~~~

\subsection ros-installation-bbb ROS

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

---

\section konfiguration-installation-bbb Konfiguration
\subsection lego-installation-bbb Lego NXT

Damit der Lego NXT Brick als USB-Gerät erkannt wird und somit auch die
Kommunikation zwischen Brick und BBB stattfinden kann, muss eine neue
Benutzergruppe erzeugt werden. Die Benutzer, die den Brick nutzen
möchten, müssen Teil dieser Gruppe sein.

~~~
sudo su
groupadd lego
usermod -a -G lego ubuntu
touch /etc/udev/rules.d/70-lego.rules
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="0694", GROUP="lego", MODE="0660"' > /etc/udev/rules.d/70-lego.rules
~~~

\b <username> muss dabei durch den Benutzer ersetzt werden, der Teil der
Gruppe werden soll. Nun muss das BBB neugestartet werden.

\subsection wlan-installation-bbb WLAN

Folgendes muss in die Datei \b /etc/network/interfaces eingefügt und angepasst werden.
Falls dort bereits ein Adapter namens wlan0 existiert, sollte dieser
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

\subsection spi-installation-bbb SPI Device Tree

Um den SPI-Bus auf dem BBB zu verwenden, müssen die jeweiligen SPI-Ports über den Device-Tree aktiviert werden. In diesem Abschnitt wird die Installation von SPI0 und SPI1 auf dem Beagle Bone Black erläutert.

Das Script addOverlayToDeviceTree installiert die Overlay-Dateien für '''SPI0 (1 chip select)''' und '''SPI1 (2 chips selects)'''.

> __Nur für Angstrom Linux__
>
> Achtung! Für das neuste Angstrom image ist kein opkg dtc Paket verfügbar. Eine einfach Lösung dafür ist, die kompilierte dtbo Datei auf einem Ubuntu Desktop PC zu erzeugen und dann aufs BBB zu kopieren.

* Klonen des bbb_setup repository
* Installation des dtc (device tree compiler)

~~~
sudo ./dtc.sh
~~~

* Das addOverlayToDeviceTree Skript kompiliert und installiert das SPI0 und SPI1 Overlay-File.

~~~
sudo ./addOverlayToDeviceTree
~~~

* Aktivieren von SPI1 beim Start (HDMI wird deaktiviert, da HDMI auch Pins, die von SPI1 verwendet werden benutzt und das zu fehlerhafter Kommunikation führt)
* Boot Partition mounten.

~~~
mkdir /mnt/boot
mount /dev/mmcblk0p1 /mnt/boot
~~~

* Textdatei uEnv.txt auf der Boot Partition bearbeiten und folgendes einfügen.

~~~
optargs=quiet drm.debug=7 capemgr.disable_partno=BB-BONELT-HDMI,BB-BONELT-HDMIN capemgr.enable_partno=MINOTAUR-SPI1
~~~

* Das Beagle Bone Black neustarten.

~~~
sudo reboot
~~~

---

\section kompilierung-installation-bbb Kompilierung

In dem Repository des Projekts existiert die Datei __compile__. Diese 
beinhaltet __verschiedene Kommandos um das Projekt zu kompilieren__. Um 
das Projekt auf dem Beagle Bone Black zu kompileren, muss __in das 
Repositoryverzeichnis gewechselt__ und folgendes Kommando ausgeführt 
werden.

~~~
./compile beagle
~~~

Um einen kompletten Rebuild durchzuführen, muss __vor dem oben genannten 
Befehl__ folgender Befehl ausgeführt werden.

~~~
./compile clean
~~~
