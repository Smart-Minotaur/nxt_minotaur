pln_minotaur {#pln_minotaur}
===

\tableofcontents

\section description Beschreibung

pln_minotaur ist eine C++ Bibliothek zum Ansteuern der PLN2033 Maussensoren.
Durch das Benutzen dieser Bibliothek wird vom Benutzer keine genaue Kenntnis
über das Linux SPI Interface sowie den PLN2033 Sensor benötigt. Das
pln_minotaur::IPLNTrackingDevice Interface stellt Funktionen zum Abfragen der
Entfernungs-Werte sowie Geschwindigkeits-Werte zur Verfügung. Zusätzlich können
die Sensoreinstellungen abgefragt sowie die Auflösung des Sensors gesetzt
werden. Durch die hohe Flexibilität und Skalierbarkeit der Bibliothek kann
eine beliebige Anzahl von Sensoren angeschlossen werden. Der Benutzer benötigt
lediglich den Namen des spidev device files.

pln_2033 bietet folgende Features:
* Viele verschiedene Funktionen zum Abfragen der Sensor Daten für verschiedene
Anwendungsfälle
* Abfrage von Displacement sowie Speed Werten des Sensors
* Abfragen und Setzen verschiedener SPI Einstellungen
* Abfragen von Sensor Einstellungen und Sensor Status (Alle Sensor Register)
* Sehr abstrahiert vom Linux SPI Interface
* Setzen von X- und Y-Auflösung eines Sensors
* Einfache objektorientierte Benutzung
* Keine externen Bibliotheken werden benötigt (benutzt Linux spidev Treiber)
* Verfügbar für ARM sowie x86 Architekturen

Die API sowie Details über die Implementierung/Vererbungshierarchie sind auf den
Doxygen Seiten der pln_minotaur Bibliothek verfügbar. Die Erklärung der
Konfiguration des SPI Interfaces befindet sich auf der Maussensoren Seite.

\section compile Kompilieren

Als Buildtool wird CMake verwendet. Die pln_minotaur Bibliothek wird für das BBB
cross-compiliert. Zum Kompilieren wird ein C++11 fähiger Compiler benötigt. Der
bei der Erstellung der Bibliothek verwendete Compiler ist der GCC 4.8. Die
Bibliothek kann für ARM sowie x86 Architekturen kompiliert werden. Im
Projektverzeichnis ist jeweils eine statisch kompilierte ARM sowie x86 Version
der Bibliothek verfügbar. Um pln_minotaur selbst zu kompilieren müssen für die
Architektur passende toolchain files verwendet werden. In den toolchain files
müssen die verwendeten ARM oder x86 Compiler eingetragen sein. Folgende Befehle
werden ausgeführt um die Bibliotheken sowie ein Beispiel zu erstellen. Die
erzeugten Dateien befinden sich im Ordner build.

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

\section api-implementation API und Implementierung

\subsection classes Klassen

Um pln_minotaur zu verwenden wird nur das Interface
pln_minotaur::IPLNTrackingDevice benötigt. Methoden zum Auslesen von
	 * Displacement in CPI (counts per inch)
	 * Displacement in cm
	 * Displacement (in CPI oder cm) mit Zeitstempel
	 * Speed in cm/second
sind vorhanden.

Bei jedem Lesen der Displacement-Werte, wird zusätzlich ein Zeitstempel mit
abgespeichert. Dadurch kann die Geschwindigkeit des Sensors berechnet werden.
Über das Interface können zusätzlich die aktuellen Sensor-Einstellungen
ausgelesen werden. Auch ist das Setzen der X- und Y-Auflösung möglich.

Hauptbestandteil sind die Klassen pln_minotaur::PLN2033 sowie
pln_minotaur:: SPIDevice. Im folgenden werden die Funktionen der verschiedenen
Klassen zusammengefasst:

\image html pln2033_interface.png "PLN2033 Interfaces"

\subsubsection spidevice SPIDevice.h

Wrapper-Klasse für den Linux SPI Treiber. Bietet komfortablere Funktionen für den
Zugriff auf den SPI-Bus.
* Senden und Empfangen von Daten über SPI
* Setzen verschiedener SPI Konfigurationen

\subsubsection PLN2033 PLN2033.h

Implementiert das SPI Bus Protokoll des PLN2033 Sensors.
* Sensorregister schreiben und lesen
* RAM schreiben und lesen
* RAM Code laden
* RAM Code überprüfen
* Sensor initialisieren (Initial Configuration Procedur durchführen)
* Power on
* Power down
* Soft reset

\subsection config Optimale Einstellungen

SPI:
~~~
const int SPIDevice::DEFAULT_SPI_MODE = 1;
const int SPIDevice::DEFAULT_SPI_SB = 0;
const int SPIDevice::DEFAULT_SPI_BITS = 8;
const int SPIDevice::DEFAULT_SPI_SPEED = 3000000;
~~~

Optimale sample Rate zum Abfragen der Displacement Register: 1kHz (1ms Interval).

(Siehe Datenblatt des PLN2033 Sensors für mögliche sample rates)

\subsection API API

Nach dem Aufruf einer read* Funktion wird das Read Register Kommando über SPI an
den Sensor gesendet. Jedes read Displacement Kommando wird intern mit einem
Zeitstempel versehen. Nach jedem Lesen werden die Displacement Register
(X und Y) auf 0 zurückgesetzt. Um die Sensor Einstellungen abzufragen wird
pln_minotaur::IPLNTrackingDevice::readPLNSettings() verwendet. Diese Funktion
liefert ein Objekt der Klasse PLNSettings zurück und beschreibt die gesamte
Sensorkonfiguration. Diese besteht aus allen internen Registerwerten. Zum Abfragen
des Status-Registers steht die Funktion
pln_minotaur::IPLNTrackingDevice::readPLNStatusRegister() zu Verfügung. Beim
Auslesen des Statusregisters werden ebenfalls die Displacement-Register zurück
gesetzt.

ACHTUNG: Wichtig zu wissen ist, wann die internen Zähler/Register des Sensors
zurückgesetzt werden. Werden Displacement-Werte oder das Status-Register
gelesen, werden die Displacement-Register auf 0 zurückgesetzt. Auch werden diese
zurückgesetzt, wenn das Lift-Bit aktiviert wird (also wenn der Sensor zu weit
von der Oberfläche entfernt ist).

\subsection array-converter Array-Converter

Um den Philips PLN2033 zu benutzen, muss zuerst ein Patch Code ins RAM des DSP
geladen werden. Das kleine Utility-Programm arrayConverter konvertiert den
vorhandenen Patch Code in eine korrektes Format (Hex-Darstellung) um diesen
komfortabel beim Start der Bibliothek ins RAM laden zu können. Der Patch Code
wird im uint8 Array Format und in korrekter Endian-Darstellung in eine Datei
abgespeichert.

\section example Beispiele

Beispiele zur Benutzung der pln_minotaur Bibliothek befinden sich in der Doxygen
Dokumentation der pln_minotaur Bibliothek.

\section improvements Verbesserungen

pln_2033 unterstützt nicht alle Funktionen des PLN2033. Eine Verbesserung der
Bibliothek wäre das Hinzufügen von Möglichkeiten zum konfigurieren verschiedener
Sleep-States des Sensors. Zusätzlich könnte die Möglichkeiten zum Ansteuern des
Sensors über Interrupts implementiert werden.
