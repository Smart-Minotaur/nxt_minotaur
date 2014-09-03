NXTControl {#nxt-control}
===

\tableofcontents

\section Beschreibung Beschreibung

Die NXTControl Bibliothek bietet eine C++ Schnittstelle, um mit einem
Lego NXT Brick zu kommunizieren. Hierbei wird das Direct Command
Interface des Bricks verwendet. Dieses Interface definiert ein
Kommunikationsprotokoll, mit dem Befehle "remote" an den Brick übergeben
werden können. So können Sensoren und Motoren direkt angesteuert werden
oder es kann über das Mailboxsystem der Brickfirmware mit einem
laufenden Programm auf dem Brick kommuniziert werden.

Die Code Dokumentation findet sich im namespace *nxt*.

NXTControl bietet folgende Features:
* USB-Kommunikation
* Implementierung __aller__ Direct Commands
* einfache objektorientierte Nutzung
* schnelle Laufzeit durch Implementierung in C++
* Threadsafe implementiert

Folgende Erweiterungsmöglichkeiten wären für die Bibliothek denkbar:
* Bluetooth-Kommunikation
* Mehr Klassen für verschiedene Sensoren (momentan nur
[Motor](@ref nxt::Motor) und [Ultraschallsensor](@ref nxt::UltrasonicSensor))
* Platformunabhängigkeit

\section Kompilieren Kompilieren

Um NXTControl zu kompilieren muss ein Compiler verwendet werden, der 
zumindest den __C++0x Standard__ implementiert hat. Außerdem muss 
__liusb-1.0 dev__ installiert sein. Nun muss das Verzeichnis 
__NXTControl__ ausgecheckt werden. In diesem Verzeichnis müssen nun 
die folgenden Befehle ausgeführt werden:

~~~
mkdir build
cd build
cmake ..
make
~~~

Nun steht die Bibiliothek im Verzeichnis __build__ zur Verfügung.

\section Entwicklungshergang Entwicklungshergang

Im Rahmen dieses Projekts war irgendeine Form der __Kommunikation mit dem
Lego NXT Brick__ unverzichtbar. In ersten Ansätzen wurde eine
__Opensource-Pythonbibliothek__ verwendet
([nxt-python](https://code.google.com/p/nxt-python/)). Bei ersten
Versuchen auf dem BBB wurde jedoch schnell klar, dass eine
Interpretersprache wie Python für ein kleines Board wie das Beagle Bone
ungeeignet ist. Die __Auslastung war deutlich zu hoch__ und Abtastintervalle
konnten nicht eingehalten werden. Ein Blick auf die Systemauslastung
mit dem Befehl *top* hat diesen Verdacht auch gleich bestätigt. Der
Pythoninterpreter hat das System zu 100% ausgelastet.

Damit wurde die Bibliothek unbrauchbar für dieses Projekt. Es gibt auch
__C++ Bibliotheken__, die die Kommunikation mit dem Brick ermöglichen
([nxt++](https://github.com/cmwslw/nxt-plus-plus)). Doch waren diese
nicht so komfortabel und schön objektorientiert zu nutzen wie die
Pythonbibliothek. Daher wurde eine eigene kleine Bibliothek
implementiert, die in ihrer Struktur der Pythonbibliothek sehr ähnelt.
Durch eigene Implementierung in C++ war nun auch die CPU-Auslastung des
Systems auf __unter 30%__ gesunken, wodurch alle Abtastintervall eingehalten
werden konnten und noch genügend Resourcen für größere Aufgaben
vorhanden waren.
