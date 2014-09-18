Maussensoren {#maussensoren}
===

Bisher wird für die Odometrie des Roboters lediglich die Motoren verwendet.
Diese bieten jedoch eine sehr geringe Genauigkeit und Zuverlässigkeit.
Zusätzlich kann sich dies je nach Fahrfläche verschlimmern (Räder drehen durch).
Um die Odometrie zu verbessern werden im Smart-Minotaur zusätzlich Maussensoren
von herkömmlichen Gaming-Computermäusen verwendet. Die in diesem Projekt
verwendete Sensoren sind die Philips pln2033 twin-eye laser sensoren.
Diese verfügen über eine sehr hohe Präzision (bis zu 6400 CPI - Counts Per Inch)
beim Messen der zurückgelegten X- und Y-Distanzwerte.

\section pln2033 PLN2033 Sensor
In diesem Abschnitt werden die wichtigsren Funktionen despln2033 Sensors erläutert.
Weitere Informationen befinden sich im Datenblatt: TODO.

\image html mouse_sensor.jpg "Maussensor von unten"

Der pln2033 besitzt unterschiedliche 16-Bit Register mit welchen dieser
gesteuert werden kann. Es werden zwei Laser verwendet, um die X und Y positions
änderungen zu messen. Intern verarbeitet ein DSP die gemessen Daten.
Die Auflösung kann von 100 bis 6400 CPI eingestellt werden. Die Ansteuerung
des Sensors erflgt über SPI und kann mit bis zu 8Mhz betrieben werden.
Der pln2033 benötigt 3.3V.

\section hardware-setup Hardware Setup

Das Beagle Bone Black besitzt zwei SPI Ports. SPI1 wird verwendet, da zwei
pln2033 Sensoren angeschlossen werden und dieser im gegensatz zu SPI0
zwei Chip-Selects besitzt.

Gelötete Platine Bild

TODO: Fritzing Bild mit SPI

\section spi-device-tree Linux SPI treiber und Device Tree
Zum Ansteuern des pln2033 wird der SPI-Bus verwendet. Für das Smart-Minotaur Projekt
wurden zwei Sensoren an den Roboter montiert
(Nur einer wird benötigt, um Position und Ausrichtung zu bestimmen).
Da zum Ansteuern von zwei Slaves auf dem SPI Bus zwei Chip sleects benötigt
werden wird der SPI1 Port des Beagle Bones verwendet. Um daten zu senden und
Emopfangen wird der linux spidev treiber verwendet, welcher rudimentöre Funktionen anbietet.
Um SPI1  Port benutzzen zu können muss dieser erst im Device Tree registreirt werden.
Dafür wird eine device teree source file benötigt, welche die korrekten Pins, register
und funktionaklittäten des SPUI Gerätes spezifiert. Diese Datei wird kompiliert und
im Linux kernel aktiviert. Wichtig ist, HDMI des BBB auszuschalten, da dieses auch
SPI1 Ports verwendet und die kommukiation stören wütrde. Nach erfolgreicher aktivierung
kann auf die slaves über gerätedateien zugegriffen werden:

/dev/spidev1.0 CS0
/dev/spidev1.1 CS1

\section pln_minotaur software Mouse Monitor Software
Die Mouse Monitor software wird zum testen, debuggen und überprüfen der
Maussensoren verwendet. Sie besteht aus zwei teilen: Ein Teil läuft auf dem Beagle Bonde Blasck,
der andere auf einem PC. Zur kommunuikation wird das ROS message system verwendet.

TODO: Bild mit packages und bei de Teile

\subsection comp Kompilierung
Die x86 Verson der pln_2033 Bibliothek muss beim kompilieren des mm_p package
dazu gelinkt werden, da daraus Datenstrukturen verwedet werden.
Zum erstellen beider Teile kann das mitgelieferte compile script verwendet werden.

\subsection mouse-monitor Mouse Monitor (mouse_monitor_pc package)
Mouse Monitor wurde ursprünglich zum testen der Philips pln2033 Maussensoren entwickelt.
Die Anwendung verfügt üpber eine Vielzahl von Funktionen und ist in zwei teile untergliedert.
Zur kommunikation über WLAN wird das ROS message systrem verwendet.

TODO BIld

\subsection bbbnode Beagle Bone Node
Diese Anwendung ist für das Auslesen der Maussdnesopr displacement und speed daten
zustäöndig und verwendet dafür die pln_minotaur bibliothek. Daten werden mit einer 1kHz freqwuenz
ausgelesne und aufsummiert. Auf anfrage der PC Node sendet die Aplikation die
geasmmeltebn daten zur PC Node. Zusätlich bietet diese funktionen um von extern die 
auflösung (CPI) der sensoren zu setzen sowie zum abgragen des sattus der sensoren.

PC Node
Die PC Node empfäng die Snesore daten, verarbeitet diese und dstellt sie grafisch dar.
Über diese Anwendung kann die CPI auflösung der sneoren eingestellt werden.
zusätzlich kann eine variable sampel rate der GUI konfiguuirtioert werden (meist 15-100Hz).
Im folgenden werden einige Features Der Anwendugn erklärt:

TODO

TODO: Bild

\section kalibrierung

\section localization Berechnung der Position und Ausrichtung des Roboters
Zum bestimmen der Position und der Ausrichtung des Roboters wird nur 1 Sensor benötigt. Die Berechnung der Roboter Porsition sowie Ausrichtung wird in deisem Abschnit beschrieben.

\section problems Probleme
Während der Durchführung des Projektes traten einige Probleme bezüglich der pln2033 Sensore auf. Diese werden in diesem Abschnitt erläutert.

* Schwierige Fehlersuche sowie Validierung der Funktionalität aufgrund schlechter Testumgebung und schlechtem hardware-Aufbau.

Der pln2033 ist ein hochprezisieser lasersensor. Um korrkete Aussagen über seine Funktionsweise zu machen, ist ein professioneller Prüfstand erforderlich welcher in diesem Projetkl nicht vorhanden war. Die tests mussten per hand (Lego-Wagen per Hand schieben, drehen ...) durchgeführt werrden, was genaues meessen unmöglich machte. Eine exakte geradweausfahrt, dreghung, .. konnte damit nicht erreicht werden.

Zum testen sowie bei der endmmontierung auf dem Roboter wird Lego benutzt, was wiederrum einige Probleme mit sich fühert:
* Wackelnde montage: Bei Bewegung des Roboters wackeln die sensoren.
* Sensoren sins sehr empfindlich. Mit Lego können diese nicht in einer exakten höhe montiert erden (was das Lift-Bit auslöst, dazu später mehr).
* Eine geraqde Montierung der sensoren ist nicht möglich.
* Damit die Snesoren immer einen gleichen Abstand zum Boden haben setzen die Sensoren auf dem Boden auf, dadruch können diese sich verkanten.

Unabhängig

* Lift-Bit problem
-> Testart: start - move. getdata. -> Sehr kleine Werte
TODO: Sollte noch auf das Prüfen
* Median Filter problerm
* WLAN Latenz

\section fazit Fazit

Mit einer hohen sample rate ist der Lift-Bit fehler sehr gering. Bei 1kHz kann dieser vernachlässigt werden.
Der Sensort arbeitet auf 1-2cm auf 100cm genau. Unabhängig davon driften die gemessenen Werte ab ca. 1m etwas ab.
Dies ist aufgrund der Lego monitierung welche sich immer wieder verschiebt. Trotz kalibrieren sind die messwerte nicht optimal.

TODO: BIld mit drift

\section improvement Verbesserungen



\image html sensor_test_car.jpg "Testfahrzeug mit 2 PLN2033"





















\section mouse_monitor_pc-gui-tools mouse_monitor_pc

\subsection mouse_monitor_pc-gui-tools_calibration-wizard Sensor Calibration Wizard

Mit diesem Tool kann die fehlerhafte Rotation bei der Montage des Sensors bestimmt werden.

\image html MM_CalibrationWizard.png

Dazu muss in der GUI der "Start"-Button betätigt werden. Anschließend muss man den Roboter 
möglichst geradeaus schieben, dass der Winkel berechnet werden kann. Während der Fahrt werden die Displacements der Sensoren angezeigt.
Danach kann der Vorgang mit dem "Stop"-Button beendet werden. Im nächsten Fenster kann dann der Verschiebungswinkel abgelesen werden.

TODO Image...

Dieser Winkel muss in der Datei __models.yaml__ bei Paramter "error_angle:" im Package __minotaur_common/param__ eingetragen werden,
sodass dieser beim Start des Roboters zur Verfügung steht.

Der Roboter wendet diesen Winkel auf die Rotationsmatrix an und multipliziert diesen mit dem aktuellen Positionsvektor.
Dadurch kann der neue Positionsvektor' bestimmt und die Rotation der Sensoren korrigiert werden.

\image html rotationmatrix.png



