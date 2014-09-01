Installation PC {#installation-pc}
===

\tableofcontents

\section Beschreibung Beschreibung

Auf dem Entwicklungsrechner wird ebenso wie auf dem Beagle Bone Black
Ubuntu in der Version 12.04 verwendet. Vorzugsweise sollte die 32bit
Version benutzt werden, da ansonsten die benötigten 32bit Bibliotheken
nachinstalliert werden müssen. Die Installation des Betriebssystem wird
hier nicht weiter erklärt, da sich genügend Anleitungen und Tools für
diesen Zweck im Internet finden.

---

\section Installation Installation
\subsection Programme Grundlegende Programme

Die folgenden Programme müssen installiert werden.

~~~
sudo apt-get install libusb-1.0-0-dev
~~~

\subsection GCC GCC 4.8
Der GCC 4.8 implementiert mehr Features des C++11 Standards als der
standardmäßige 4.6 Compiler unter Ubuntu 12.04. Um diesen zu
installieren und als Standardcompiler festzulegen, müssen die folgenden
Kommandos ausgeführt werden.

~~~
sudo add-apt-repository ppa:ubuntu-toolchain-r/test
sudo apt-get update
sudo apt-get install gcc-4.8 g++-4.8
sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-4.8 50
sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-4.8 50
~~~

\subsection ROS ROS

Die verwendete ROS Version ist __hydro__. Um ROS hydro auf dem Computer
zu installieren, müssen folgende Befehle ausgeführt werden.

~~~
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu precise main" > /etc/apt/sources.list.d/ros-latest.list'
wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
sudo apt-get update
sudo apt-get install ros-hydro-desktop-full
sudo rosdep init
rosdep update
echo "source /opt/ros/hydro/setup.bash" >> ~/.bashrc
source ~/.bashrc
~~~

\subsection QT4 QT4

Für die grafischen Programme des Projekts werden QT4 und QWT
verwendet. Diese werden aus den Paketquellen installiert.

~~~
sudo apt-get install ros-hydro-qt* libqwt-dev
~~~

---

\section Konfiguration Konfiguration
\subsection Lego Lego NXT

Damit der Lego NXT Brick als USB-Gerät erkannt wird und somit auch die
Kommunikation zwischen Brick und BBB stattfinden kann, muss eine neue
Benutzergruppe erzeugt werden. Die Benutzer, die den Brick nutzen
möchten müssen Teil dieser Gruppe sein. Dieser Schritt ist nicht
zwingend notwendig. Er wird nur benötigt, wenn der Brick zu Testzwecken
direkt vom Computer aus angesteuert werden soll.

~~~
sudo su
groupadd lego
usermod -a -G lego <username>
touch /etc/udev/rules.d/70-lego.rules
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="0694", GROUP="lego", MODE="0660"' > /etc/udev/rules.d/70-lego.rules
~~~

__<username>__ muss dabei durch den Benutzer ersetzt werden, der Teil
Gruppe werden soll. Um die Einstellungen zu übernehmen, muss der
Computer neugestartet werden.
