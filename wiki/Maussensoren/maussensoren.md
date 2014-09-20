Maussensoren {#maussensoren}
===

Der im Projekt verwendete Roboter stellt für die Odometrie Motoren zu Verfügung.
Diese bieten jedoch eine sehr geringe Genauigkeit und Zuverlässigkeit. Zusätzlich
kann sich dies je nach Fahrfläche verschlimmern (Räder drehen durch). Um die
Odometrie zu verbessern werden im Smart-Minotaur zusätzlich Maussensoren von
herkömmlichen Gaming-Computermäusen verwendet. Diese Messen die zurückgelegte X-
und Y-Distaz indem die mithilfe von Lasern erzeugten Bilder der Oberfläche
verarbeitet werden. Die in diesem Projektve verwendete Sensoren sind die Philips
PLN2033 twin-eye laser sensoren. Diese verfügen über eine sehr hohe Präzision
(bis zu 6400 CPI - Counts Per Inch) beim Messen der zurückgelegten X- und
Y-Distanzwerte.

\section pln2033 PLN2033 Sensor
In diesem Abschnitt werden die wichtigsten Funktionen des pln2033 Sensors
erläutert. Weitere Informationen befinden sich im Datenblatt: TODO.

\image html mouse_sensor.jpg "Maussensor von unten"

Der pln2033 besitzt verschiedene 16-Bit Register mit welchen dieser konfiguriert
und gesteuert werden kann. Es werden zwei Laser verwendet, um die X und Y
Positionsänderungen zu messen. Mithilfe dieser Laser werden Bilder der Oberfläche
erstellt. Intern verarbeitet ein DSP diese Bilder und bestimmt aus diesen den
zurückgelegten Weg. Die Auflösung kann von 100 bis 6400 CPI eingestellt werden.
Die Ansteuerung des Sensors erflgt über SPI und kann mit bis zu 8Mhz (Linux SPI
Treiber Clock) betrieben werden. Der pln2033 benötigt zum Arbeiten 3.3V.

\section hardware-setup Hardware Setup

Das Beagle Bone Black besitzt zwei SPI Ports. SPI1 wird verwendet, da zwei
pln2033 Sensoren angeschlossen werden und dieser im gegensatz zu SPI0 zwei
Chip-Selects besitzt.

Gelötete Platine Bild

TODO: Fritzing Bild mit SPI

\section spi-device-tree Linux SPI treiber und Device Tree

Zum Ansteuern des pln2033 wird der SPI-Bus verwendet. Für das Smart-Minotaur
Projekt wurden zwei Sensoren an den Roboter montiert (Nur einer wird benötigt, um
Position und Ausrichtung zu bestimmen). Da zum Ansteuern von zwei Slaves auf dem
SPI Bus zwei Chip-Selects benötigt werden wird der SPI1 Port des Beagle Bones
verwendet. Um daten zu Senden und Empfangen wird der Linux spidev Treiber
verwendet, welcher rudimentäre Funktionen anbietet. Um den SPI1 Port benutzen zu
können muss dieser erst im Device Tree registriert werden. Dafür wird ein device
Tree source file benötigt (dts), welche die korrekten Pins, Register und
Funktionalitäten des SPI Gerätes spezifiziert. Diese Datei wird kompiliert und
im Linux Kernel aktiviert. Wichtig ist das HDMI des BBB auszuschalten, da dieses
auch SPI1 Ports verwendet und die Kommunikation stören würde. Nach erfolgreicher
Aktivierung kann auf die Slaves über Gerätedateien zugegriffen werden:

~~~
/dev/spidev1.0 CS0
/dev/spidev1.1 CS1
~~~

\section pln_minotaur software Mouse Monitor Software

Die Mouse Monitor software wird zum Testen, Debuggen und Überprüfen der
Maussensoren verwendet. Die Anwendung verfügt über eine Vielzahl von Funktionen.
Sie besteht aus zwei Teilen:
* Mouse Monitor beagle (mm_b)
* Mouse Monitor PC (mm_p)

Zur Kommunikation wird das ROS message system verwendet. Daher sind beide Teile
als ROS Packages realisiert. Entwickelt wurden beide Teile in C++. Zur Darstellung
der Daten auf dem PC wird Qt verwendet. Die pln_minotaur Bibliothek wird zum
Ansteuern der Sensoren benutzt.

TODO: Bild mit packages und beide Teile (Datenfluss)

\subsection comp Kompilierung

Die x86 Version der pln_minotaur Bibliothek muss beim Kompilieren des mm_p Package
dazu gelinkt werden, da daraus Datenstrukturen verwedet werden. Zum erstellen
beider Teile kann das mitgelieferte compile script verwendet werden.

~~~
# Mouse Monitor PC
./compile mm_p

# Mouse Monitor Beagle
./compile mm_b
~~~

\subsection mouse-monitor Mouse Monitor PC

Die PC Node empfäng die Snesore daten, verarbeitet diese und dstellt sie grafisch dar.
Über diese Anwendung kann die CPI auflösung der sneoren eingestellt werden.
zusätzlich kann eine variable sampel rate der GUI konfiguuirtioert werden (meist 15-100Hz).
Im folgenden werden einige Features Der Anwendugn erklärt.

TODO BIld

Features:
* Position, Ausrichtung und zurückgelegter Weg des Roboters sowie der Sensoren
wird auf einer Karte dargestellt.
* Kalibrierungsfunktion für die Sensoren
* darstellung der Sensor konfigurationen
* Einstellen der Auflösung beider Sensoren
* Konfigurierbare sample rate des PC Teils
* Darrstellung aller sensor Register
* Verschiiedene Darstellungen der Displacement Daten
** Direction Widget
** Parth Widget
** Graphen
** Plain Data List
* Debugging feature zum manuellen Abfragen der Sensor Daten
* Optionaler Median Filter mit dynamisch Änderbaren Parametern kann jeweils
f+r Sensor 1 oder Sensor 2 aktiviert werden

TODO: Bilder der verschiedenen Funktionen

\subsection bbbNode Mouse Monitor Beagle

Diese Anwendung ist für das Auslesen der Maussensor displacement und speed daten
zustäöndig und verwendet dafür die pln_minotaur bibliothek. Daten werden mit einer 1kHz freqwuenz
ausgelesne und aufsummiert. Auf anfrage der PC Node sendet die Aplikation die
geasmmeltebn daten zur PC Node. Zusätlich bietet diese funktionen um von extern die
auflösung (CPI) der sensoren zu setzen sowie zum abgragen des sattus der sensoren.

\section kalibrierung Kalibierung

Bei der berechnung der Position und der Ausrichtung des Roboters wird eine
korrekte Montierung der Maussensoren vorausgesetzt. Die Sensoren müssen richtig in
Fahrtrichtung ausgerichtet sein, um exakte berechnungen machen zu können. Das
bedeutet, bei einer Vorwärtsbewegung soll sich anur die Y-Werte Ändern. Da eine
korrekte Montierung sehr schwierig ist (und das durch das Lego ohnehin unmöglich ist)
müssen die Sensoren kalibriert werden um den Fehler der schräegen Montierung auszugleichen.

TODO: Bild mit schrägen Sensoren und Achsen

Über die GUI kann ein kalibrierungswizard gestartet werden. Beim Kalibrieren
wird die gesamtdistanz einer gefahrenen geradeausfahrt gemessen und der Winkel gegenüber
der idealen geradeaus fahrt bestimmt. Alle folgenden Messdaten (Vektoren) werden
mithilfe einer Rotationsmatrix um diesne winkel gedreht.

~~~
if (s1YDisplacement > 0)
			atanRealDistance = M_PI/2;
		else if (s1YDisplacement < 0)
			atanRealDistance = -M_PI/2;

s1AngleOffset = atanRealDistance - std::atan2(s1YDisplacement, s1XDisplacement);
~~~

~~
double angle = calibrationData.getS1AngleOffset();

data.x_disp = (std::cos(angle) * data.x_disp) + (-std::sin(angle) * data.y_disp);
data.y_disp = (std::sin(angle) * data.x_disp) + (std::cos(angle) * data.y_disp);
~~~

section comm Kommunikation

Für die WLAN Kommunikation zwischen Beagle Bone und PC existiert jeweils eine
ROS Node. Dabei werden ROS Services verwendet. Mouse Monitor PC kann in
konfigurierbaren Intervallen die Sensor Daten abgragen.

TODO: Bild

\section localization Berechnung der Position und Ausrichtung des Roboters

Zum bestimmen der Position und der Ausrichtung des Roboters wird nur 1 Sensor
benötigt. Die Berechnung der Roboter Porsition sowie Ausrichtung wird in deisem
Abschnit beschrieben.

Ist der Sensor in einer rechtwinkligen position zum kreistangente montiert vereinfacht sich
die Berechnung sehr. In diesem Abschnitt wird die Lösung bei einer variablen montierung beschrieben,
da dies allgemeiner ist. Obwohl am Roboter 2 Sensoren montiert sind, wird nur einer zum besimmen der position
verwendet.

subsection ausgang ausgangssituation:

Mit der Annahme, dass der Sensor kalibirtiert wurde kann von folgender situatin ausgegangen werden.

Todo: Bild

Bei einer geradeausfahrt ändert sich nur die y koordiante.

Zusätzlich sind folgende parameter des Roboers Bekannt.
*(settingss struct)
*
*

subsection ber Berechnung

Da die Sensoren nicht im rechtee Winkel mit der Kreistangente montiert sind, muss das verhältnis von x und y_disp
änderungen aus den Empangenen Sensorwärten herausgerechnet werden. Die zurückgelegte strecke für geradeausfahrt
sowei bei iener kurve muss ausgerechnet werden. Das verhältnis von x und y werten
bei einer drehung ist bekannt. Da die Maße des roboters bekannt sind kann dies bestimmt werden:

Dabei muss lediglich der winkel der y achse des sensors egenüber der kreistangente bestimt werden.
Mit diesem wissen kann der y anteil bei iener drehung bestimmt werden. Da die zurückgelegte disatz
bei einer drehung bekannt ist, kann mithilfe der kreisbogeformel der drehweinkel bestimmt werden.
Die geamse zurückgeegte distanz minus die kurvendistanz ergbt die zurüclkgekegte strecke bei geradeasu oder
rückwertsfahrt.

subsection impl Implementierung

Die Berechung wird bisher zu Debugging zwecken auf einem PC ausgeführt.
Die wichtigsten Klassen befinden ich inder datei Robot.h.

Roboter und Sneoren werden als Objekte der Superklasse Object erzeugt

Robot:
* X Position im globalen Koord
* Y Position im globalen Koord
* Uasrichtung im globalen Koords

Sensor
* X im  Roboter Koods
* Y im Roboter Kords
* Ausrichtung im robot Koords

Zus#tzlich wird bei allen der Zurükgelegte Pfad abgespeichert.

TODO: Vererbung object und sensor / robot

Die Klsse Object besitt die mehoen forward und rotate() um den roboter/Sensoren zu bewegen.
Die Methode move() wird verwendet um den Roboter aufgrund der X und Y Displacementwerte zu bewegene.

Dabei wird die oben erklärten berechngen ausgeführt, welche als ergebnis die zurückgelegte distanzen bei
gerausfahrt und drehung ergeben. Je nach richtung werden die forward() und rotate() methoden ausgeführt.

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

SPI device tree 2 Chip-Selects für PI1 und Ubuntu

\section fazit Fazit und Erkenntnisse

Mit einer hohen sample rate ist der Lift-Bit fehler sehr gering. Bei 1kHz kann dieser vernachlässigt werden.
Der Sensort arbeitet auf 1-2cm auf 100cm genau. Unabhängig davon driften die gemessenen Werte ab ca. 1m etwas ab.
Dies ist aufgrund der Lego monitierung welche sich immer wieder verschiebt. Trotz kalibrieren sind die messwerte nicht optimal.

TODO: BIld mit drift

Nur 1Sensor wird zum bestimmen der Position und Ausrichtung benötigt.

\section improvement Verbesserungen

* Filter beim Lift bit einführen
* Mehr teile vom PC zum BBB verlagern
* Bessere Hardware zum Montieren verwenden
** die nicht wackelt
** Gleicher abstand der Sensoren zum Biden ohne schleifen

\image html sensor_test_car.jpg "Testfahrzeug mit 2 PLN2033"

















Der Roboter wendet diesen Winkel auf die Rotationsmatrix an und multipliziert diesen mit dem aktuellen Positionsvektor.
Dadurch kann der neue Positionsvektor' bestimmt und die Rotation der Sensoren korrigiert werden.

\image html rotationmatrix.png
