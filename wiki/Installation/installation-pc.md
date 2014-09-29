Installation PC {#installation-pc}
===

\tableofcontents

\section beschreibung-installation-pc Beschreibung

Auf dem Entwicklungsrechner wird, ebenso wie auf dem Beagle Bone Black,
__Ubuntu in der Version 12.04__ verwendet. Vorzugsweise sollte die 32bit
Version benutzt werden, da ansonsten die benötigten 64bit Bibliotheken
nachinstalliert werden müssen. Die Installation des Betriebssystem wird
hier nicht weiter erklärt, da sich genügend Anleitungen und Tools für
diesen Zweck im Internet finden.

---

\section installation-installation-pc Installation
\subsection Programme Grundlegende Programme

Die folgenden Programme müssen installiert werden.

~~~
sudo apt-get install libusb-1.0-0-dev
~~~

\subsection gcc-installation-pc GCC 4.8
Der GCC 4.8 implementiert mehr Features des __C++11 Standards__ als der
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

\subsection cross-gcc-installation-pc GCC-ARM 4.8
Zum cross-compilieren der pln_minotaur Bibliothek, welche zum Ansteuern der Maussensoren verwendet wird, wird eine Cross-Compile Toolchain verwendet. Dabei wird eine bereits kompilierte Toolchain für Linux von Linaro verwendet.

\subsection ros-installation-pc ROS

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
sudo apt-get install ros-hydro-joy
~~~

\subsection qt4-installation-pc QT4

Für die grafischen Programme des Projekts werden __QT4 und QWT__
verwendet. Diese werden aus den Paketquellen installiert.

~~~
sudo apt-get install ros-hydro-qt* libqwt-dev
~~~

---

\section konfiguration-installation-pc Konfiguration
\subsection lego-installation-pc Lego NXT

Damit der __Lego NXT Brick als USB-Gerät erkannt wird__ und somit auch die
Kommunikation zwischen Brick und PC stattfinden kann, muss eine neue
Benutzergruppe erzeugt werden. Die Benutzer, die den Brick nutzen
möchten, müssen Teil dieser Gruppe sein. Dieser Schritt ist nicht
zwingend notwendig. Er wird nur benötigt, wenn der Brick zu Testzwecken
direkt vom Computer aus angesteuert werden soll.

~~~
sudo su
groupadd lego
usermod -a -G lego <username>
touch /etc/udev/rules.d/70-lego.rules
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="0694", GROUP="lego", MODE="0660"' > /etc/udev/rules.d/70-lego.rules
~~~

<username> muss dabei durch den Benutzer ersetzt werden, der Teil der
Gruppe werden soll. Um diese Einstellungen zu übernehmen, muss der
Computer neugestartet werden.

---

\section kompilierung-installation-pc Kompilierung

In dem Repository des Projekts existiert die Datei __compile__. Diese 
beinhaltet __verschiedene Kommandos um das Projekt zu kompilieren__. Um 
das Projekt auf dem PC zu kompilieren, muss __in das 
Repositoryverzeichnis gewechselt__ und folgendes Kommando ausgeführt 
werden:

~~~
./compile pc
~~~

Um einen kompletten Rebuild durchzuführen, muss __vor dem oben genannten 
Befehl__ folgender Befehl ausgeführt werden:

~~~
./compile clean
~~~
