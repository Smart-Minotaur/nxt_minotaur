pln_minotaur {#pln-minotaur}
===

\section description Beschreibung

pln_minotaur ist eine C++ Bibliothek zum Ansteuern der pln2033 Maussensoren.
Dadurch wird vom Benutzer keine genaue Kenntnis über das Linux SPI Interface
sowie den pln2033 Sensor benötigt. Das pln_minotaur::IPLNTrackingDevice
Interface stellt Funktionen zum Abfragen der Entfernungs-Werte sowie
Geschwindigkeits-Werte zu Verfügung. Zusätzlich können die Sensor Einstellungen
abgefragt sowie die Auflösung der Sensoren gesetzt werden. Durch die hohe
Flexibilität und Skalierbarkeit der Bibliothek können eine beliebige Anzahl von
Sensoren angeschlossen werden. Der Benutzer benötigt lediglich den Namen des
spidev device files.

pln_2033 bietet folgende Features:
* Viele verschiedene Funktionen zum Abfragen der Sensor Daten für verschiedene
Anwendungsfälle
* Abfrage von Displacement sowie Speed Werte (interner Zeitstempel) des Sensors
* Sehr abstrahiert vom Linux SPI Interface
* Einfache objektorientierte Benutzung
* Keine externen Bibliotheken werden benötigt (benutzt Linux spidev Treiber)
* Funktioniert für ARM sowie x86

Die API sowie Details über die Implementierung/Vererbungshierarchie sind auf den
Doxygen Seiten der pln_2033 Bibliothek verfügbar. Die Erklärung der
Konfiguration des SPI Interfaces befindet sich auf der Maussensoren Seite.

\section compile Kompilieren

Als Buildtool wird CMake verwendet. Die pln_minotaur Bibliothek wird für das BBB
cross-compiliert. Zum kompilieren wird ein C++11 fähiger Compiler benötigt. Der
bei der Erstellung der Bibliothek verwendete Compiler ist der GCC 4.8. Die
Bibliothek kann für ARM sowie x86 Architekturen kompiliert werden. Im
Projektverzeichnis ist jeweils eine statisch kompilierte ARM sowie x86 Version
der Bibliothek verfügbar. Um pln_2033 selbst zu kompilieren müssen für die
Architektur passende toolchain files verwendet werden. Folgende Befehle werden
ausgeführt um die Bibliotheken sowie ein Beispiel zu erstellen. Die erzeugten
Dateien befinden sich im Ordner build.

\subsection arm ARM

~~~
mkdir build
cd build
cmake -DCMAKE_TOOLCHAIN_FILE=../toolchain-BeagleBoneBlack.cmake ..
make
~~~

\subsection x86 x86

~~~
mkdir build
cd build
cmake -DCMAKE_TOOLCHAIN_FILE=../toolchain-x86-BeagleBoneBlack.cmake ..
make
~~~

\section api-implementation API sowie Implementierung

\subsection classes Klassen

Um pln_minotaur zu verwenden wird nur das Interface IPLNTrackingDevice benötigt.
Methoden zum Auslesen von
	 * Displacement in CPI (counts per inch)
	 * Displacement in cm
	 * Speed in cm/second
sind vorhanden.

Bei jedem Lesen der Displacement werte, wird zusätzlich ein Zeitstempel mit
abgespeichert. Dadurch kann die Geschwindigkeit des Sensors berechnet werden.
Über das Interface kann zusätzlich die aktuellen Sensor Einstellungen ausgelesen
werden. Auch ist das Setzen der X- und Y-Auflösung möglich.

Hauptbestandteil sind die Klassen PLN2033 sowie SPIDevice. Im folgenden werden
die Funktionen der verschiedenen klassen zusammengedfasst:

\image html pln2033_interface.png "pln_2033 interfaces"

\subsubsection spidevice SPIDevice.h

Wrapper klasse für den Linux SPI trieber:
* Senden und empfangen von daten über SPI
* Settzen verschiedner SPI configtatuinenn
* Pseudo-multiplex transfers

\subsubsection pln2033 PLN2033.h

Implementiert das SPI Bus Protokoll des pln2033 Sensors.
* Sensior register schreiben und lesen
* RAM schreiben und lesen
* RAM code laden
* RAM code überprüfen
* Snesor initalsieren (inital configuration procedur durchführen)
* power on
* poer down
* soft reset

\subsection config Optimale Enstellungen

SPI:
~~~
const int SPIDevice::DEFAULT_SPI_MODE = 1;
const int SPIDevice::DEFAULT_SPI_SB = 0;
const int SPIDevice::DEFAULT_SPI_BITS = 8;
const int SPIDevice::DEFAULT_SPI_SPEED = 3000000;
~~~

Sample Rate zum Abfragen der displacement Register: 1kHz (1ms interval).

\subsection API API

Nach dem Aufruf einer read* Funktion wird das Read Register Kommando über SPI an
den Sensor gesendet. Jedes read displacement Kommando wird intern mit einem
Zeitstempel versehen. Nach jedem Lesen werden die displacement Register
(x und y) auf 0 zurückgesetzt. Um die Sensor Einstellungen abzufragen wird
pln_minotaur::IPLNTrackingDevice::readPLNSettings() verwendet. Diese Funktion
liefert ein Objekt der Klasse PLNSettings zurück und beschreibt die gesamte
Sensor Konfiguration. Zum Abfragen des Statuus Registers steht die Gunktion
pln_minotaur::IPLNTrackingDevice::readPLNStatusRegister() zu verfügung. Achtung:
Beim Auslesen des Status registers werden ebenfalls die displacement register
zurück gesetzt.

ACHTUNG: Wichtig zu wissen ist, wann die internen Zähler/Register des sensors
zurückgesetzt werden. Werdern displacement werde oder das status register
gleesne, werden die dispalcement register auf 0 zurückgesetzt. Auch werden diese
zurückgestezt, wenn das Lift-Bit aktiviert wird (also wenn der sensor zu weit
von der oberfläche entfenrt wis).

\subsection array-converter Array-Converter
Um den Philips pln2033 zu benutzen muss zuerst ein Patch code ins RAM des DSP
geladen werden. Das kleine utility programm arrayConverter convertiert den
vorhandenen Pacth code in eine korrektes Format (Hex darstellung) um diesen
komfortabel beim start der bibliothek ins RAM laden zu können. Der Patch code
wird im uint8 array format und in korrekte endian darstellung in einer datei
sgespeichert.

\section example Beispiele
Beispiele zur Benutzung der pln_minotaur Bibliotheke befinden sich auf
folgender Seite: TODO

\section improve Verbesserungen

pln_2033 unterstützt nicht alle Funktionen des PLN2033. Eine Verbesserung der
Bibiothek wäre das Hinzufügen von Möglichkeiten zum konfigurieren verschiedener
Sleep-states des Sensors. Auch kann eine Interruptgesteuuerte Funktion
implementiert werden.

