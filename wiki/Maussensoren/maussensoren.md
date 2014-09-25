Maussensoren {#maussensoren}
===

Der im Projekt verwendete Roboter stellt für die Odometrie Motoren zu Verfügung.
Diese bieten jedoch eine sehr geringe Genauigkeit und Zuverlässigkeit. Zusätzlich
kann sich dies je nach Fahrfläche verschlimmern (Räder drehen durch). Um die
Odometrie zu verbessern werden im Smart-Minotaur zusätzlich Maussensoren von
herkömmlichen Gaming-Computermäusen verwendet. Diese messen die zurückgelegte X-
und Y-Distaz indem die mithilfe von Lasern erzeugten Bilder der Oberfläche
verarbeitet werden. Die in diesem Projekt verwendete Sensoren sind die Philips
PLN2033 twin-eye laser sensoren. Diese verfügen über eine sehr hohe Präzision
(bis zu 6400 CPI - Counts Per Inch) beim Messen der zurückgelegten X- und
Y-Distanzwerte.

\section pln2033 PLN2033 Sensor
In diesem Abschnitt werden die wichtigsten Funktionen des PLN2033 Sensors
erläutert. Weitere Informationen befinden sich im Datenblatt: TODO.

\image html mouse_sensor.jpg "Maussensor von unten"

Der PLN2033 besitzt verschiedene 16-Bit Register mit welchen dieser konfiguriert
und gesteuert werden kann. Es werden zwei Laser verwendet, um die X und Y
Positionsänderungen zu messen. Mithilfe dieser Laser werden Bilder der Oberfläche
erstellt. Intern verarbeitet ein DSP (digital signal processor) diese Bilder und bestimmt aus diesen den
zurückgelegten Weg. Die Auflösung kann von 100 bis 6400 CPI eingestellt werden.
Die Ansteuerung des Sensors erfolgt über SPI und kann mit bis zu 8Mhz (Linux SPI
Treiber Clock) betrieben werden. Der PLN2033 benötigt zum Arbeiten 3.3V.

\section hardware-setup Hardware Setup

Das Beagle Bone Black besitzt zwei SPI Ports. SPI1 wird verwendet, da zwei
PLN2033 Sensoren angeschlossen werden und dieser im Gegensatz zu SPI0 zwei
Chip-Selects besitzt.

TODO: Gelötete Platine Bild

TODO: Fritzing Bild mit SPI

\section spi-device-tree Linux SPI Treiber und Device Tree

Zum Ansteuern des PLN2033 wird der SPI-Bus verwendet. Für das Smart-Minotaur
Projekt wurden zwei Sensoren an den Roboter montiert (Nur einer wird benötigt, um
Position und Ausrichtung zu bestimmen). Da zum Ansteuern von zwei Slaves auf dem
SPI Bus zwei Chip-Selects benötigt werden wird der SPI1 Port des Beagle Bones
verwendet. Um daten zu Senden und Empfangen wird der Linux spidev Treiber
verwendet, welcher rudimentäre Funktionen anbietet. Um den SPI1 Port benutzen zu
können muss dieser erst im Device Tree registriert werden. Dafür wird ein Device
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
* Mouse Monitor Beagle (mm_b)
* Mouse Monitor PC (mm_p)

Zur Kommunikation wird das ROS message system verwendet. Daher sind beide Teile
als ROS Packages realisiert. Entwickelt wurden beide Teile in C++. Zur Darstellung
der Daten auf dem PC wird Qt verwendet. Die pln_minotaur Bibliothek wird zum
Ansteuern der Sensoren benutzt.

TODO: Bild mit packages und beide Teile (Datenfluss)

\subsection comp Kompilierung

Die x86 Version der pln_minotaur Bibliothek muss beim Kompilieren des mm_p Package
dazu gelinkt werden, da daraus Datenstrukturen verwendet werden. Zum erstellen
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

\image html rotationmatrix.png

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
konfigurierbaren Intervallen die Sensor Daten abfragen.

Beim Abfragen der Sensor Daten schickt die Beagle Bonde Node folgende ROS
Message:

Zusätzlich existieren Messages/Services zum setzen der Sensor Auflösung sowie
zum Abfragen der Sensor koniguration.

TODO: Bild

\section localization Berechnung der Position und Ausrichtung des Roboters

Zum bestimmen der Position und der Ausrichtung des Roboters wird nur 1 Sensor
benötigt. Die Berechnung der Roboter Porsition sowie Ausrichtung wird in deisem
Abschnit beschrieben.

Ist der Sensor in einer rechtwinkligen position zur kreistangente des Kreises
den der Sensor um die Achse desRoboter bildet montiert vereinfacht sich
die Berechnung sehr. In diesem Abschnitt wird die Lösung bei einer variablen
montierung beschrieben, da dies allgemeiner ist. Obwohl am Roboter 2 Sensoren
montiert sind, wird nur einer zum besimmen der position verwendet.

subsection ausgang ausgangssituation:

Mit der Annahme, dass der Sensor kalibirtiert wurde kann von folgender
situatin ausgegangen werden.

Todo: Bild

Bei einer geradeausfahrt ändert sich nur die y koordiante.

Zusätzlich sind folgende Parameter des Roboters bekannt:
~~~
double axisLength; // cm

double distanceToSensor_x; // cm
double distanceToSensor_y; // cm

double distanceToSensor_radius; // cm
double distanceToWheel; // cm
double sensorAngle; // Radiants
double m;
~~~

subsection ber Berechnung

Da die Sensoren nicht im rechten Winkel mit der Kreistangente montiert sind
ändern sich bei einer Drehung des Roboters X- sowie Y-Koordinaten des Sensors.
Bei einer geradeaus Fahrt ändern sich nur die Y-Koordinaten. Werden die
empfangenen Sensor Daten direkt dargestellt ergibt sich aufgrund der rotierten
Montierung des Sensors eine gerade mit fester Steigung:

TODO: Bild

Um anhand der Sensor-Displacement Daten die Position des Roboters zu bestimmen
muss aus diesen Daten die jeweilige zurückgelegte Distanz bei einer Drehung
sowie bei einer geradeaus Fahrt bestimmt werden. Die Werte müssen in Drehung und
geradeaus Fahrt unterteilt werden. Dafür wird das Verhältnis der Änderung der X-
und Y-Koordinaten bei einer Drehung aus den empfangenen Sensorwerten heraus
gerechnet werden. Das Verhältnis von X- und Y-Werten bei einer Drehung ist
bekannt. Das Verhältnis entspricht genau dem im folgenden Bild dargestellten
Winkel.

TODO: Bild mit Winkel

Da die Maße des Roboters bekannt sind kann dieser Winkel sowie die Steigung
bestimmt werden:

~~~
sensorAngle = (M_PI/2.0) - atan(distanceToSensor_y/distanceToSensor_x);
m = tan(sensorAngle);
~~~

Dabei muss lediglich der Winkel der Y-Achse des Sensors gegenüber dem Radius
bestimmt werden. Mit diesem Wissen kann der Y-Anteil bei einer Drehung bestimmt
werden. Mithilfe des Satz des Pythagoras kann die zurückgelegte Distanz bei
Drehung sowie geradeaus Fahrt bestimmt werden. Da die zurückgelegte Distanz bei
einer Drehung bekannt ist, kann mithilfe der Kreisbogenformel der Drehwinkel
bestimmt werden. Die gesamte zurückgelegte dDstanz minus die Kurvendistanz
ergibt die zurückgelegte Strecke bei geradeaus Fahrt (bzw. rückwärts Fahrt)..

subsection impl Implementierung

Die Berechnung wird bisher zu Debugging Zwecken auf einem PC ausgeführt. Die
wichtigsten Klassen befinden sich in der Datei Robot.h. Roboter und Sensoren
werden als Objekte der Superklasse Object erzeugt, welche als 
Hauptfunktion die Methoden forward() sowie rotate() zum bewegen der Objekte
bietet.

Robot:
* X Position im globalen Koord
* Y Position im globalen Koord
* Uasrichtung im globalen Koords

Sensor
* X im  Roboter Koods
* Y im Roboter Kords
* Ausrichtung im robot Koords

Zusätzlich wird bei allen Objekten der zurückgelegte Pfad (im globalen 
Koordinatensystem) abgespeichert.

TODO: Vererbung object und sensor / robot

Die Methode move() des Roboters wird verwendet um den Roboter aufgrund der X-
und Y-Displacement Werte eines Sensors zu bewegen. Dabei werden die oben
erklärten Berechnungen ausgeführt, welche als Ergebnis den Drehwinkel sowie die
zurückgelegte Distanz bei geradeaus Fahrt liefern. Je nach Richtung werden die
forward() und rotate() Methoden ausgeführt.

~~~
// V_r = Vector for rotation
// V_f = Vector for forward/backward
// V_rd = Vector for rotation and forward/backward

double Vx_r = dX;
double Vy_r = dX * attributes.m;

double Vdist_r = sqrt(pow(Vx_r, 2) + pow(Vy_r, 2));

double Vm_r = Vy_r/Vx_r;
double Vangle_r = atan(Vm_r);

// From circular arc (Vdist_r)
double rotateAngle = Vdist_r / attributes.distanceToSensor_radius;

// Rotation direction
if (dX >= 0)
	rotate(rotateAngle);
else
	rotate(rotateAngle * -1);

double Vdist_rf = sqrt(pow(dX, 2) + pow(dY, 2));
double Vdist_f = Vdist_rf - Vdist_r;

// Forward or backward
if (dY < 0)
	Vdist_f *= -1;

forward(Vdist_f);
~~~

\section problems Probleme
Während der Durchführung des Projektes traten einige Probleme bezüglich der
pln2033 Sensoren auf. Diese werden in diesem Abschnitt erläutert.

* Schwierige Fehlersuche

Aufgrund der schlechten Testumgebung und schlechtem Hardware-Aufbau war die
Fehlersuche sowie Validierung der Sensor-Funktionalität sehr schwierig. Der
pln2033 ist ein hoch präziser Lasersensor. Um korrekte Aussagen über seine
Funktionsweise zu machen, ist ein professioneller Prüfstand erforderlich welcher
in diesem Projekt nicht vorhanden war. Die Tests mussten per Hand (Lego-Wagen
per Hand schieben, drehen ...) durchgeführt werden, was genaues Messen
unmöglich machte. Eine exakte geradeaus Fahrt, Drehung, ... konnte damit nicht
erreicht werden.

* Zum Testen sowie bei der Endmontierung auf dem Roboter wird Lego benutzt, was
wiederum einige Probleme mit sich führt

** Wackelnde Montage: Bei Bewegung des Roboters wackeln die Sensoren. Trotz
Kalibrierung kann dadurch keine exakte Messung gemacht werden, da die Lage der
Sensoren sich bei einer Fahrt immer ändert.
** Die Sensoren sind sehr empfindlich. Mit Lego können diese nicht in einer
exakten Höhe montiert werden (was das Lift-Bit auslöst, dazu später mehr).
** Eine gerade Montierung der Sensoren ist nicht möglich.
** Damit die Sensoren immer einen gleichen Abstand zum Boden haben setzen die
Sensoren auf dem Boden auf, dadurch können diese sich verkanten, die Messungen
werden ungenau.

* Lift-Bit Problem

Zum Testen der Grundfunktionalität der Sensoren wurde folgender Test auf dem
Beagle Bone gemacht:

Teststart:
Reset der internen Register.

Der Roboter wird eine festgelegte Strecke (10cm) per Hand vorwärts bewegt.
Dabei zählen die internen X- und Y-Register hoch. Es wurde mit einer festgeleten
Auflösung sichergestellt, dass kein Überlauf bei der 10cm Strecke auftreten
kann.

Testende:
Die Register Werte des Sensors werden ausgelesen. Nun sollten die Y-Werte
(geradeaus) immer 10cm betragen.

Beobachtung:
Bei mehreren Test wurde festgestellt das bei einigen Testfahrten die Y-Werte
sehr kleine Werte annehmen. Dies passierte ungefähr in 10%-20% der Fälle.

Nach langem Debuggen wurde festgestellt, dass die Sensoren sehr empfindlich
auf anheben reagieren. Da bei Bewegungen von Hand die Sensoren sehr wackeln,
wurde beim anhalten des Roboters (nach 10cm) in einigen Fällen das Lift-Bit
des Sensors gesetzt. Dieses Lift-Bit wird gesetzt, wenn der Sensor zu weit von
seiner kalibrierten Höhe angehoben wird. Passiert dies, werden alle internen
Zähler zurückgesetzt, die Werte sind verloren.

Eine Lösung für dieses Problem ist es, die Sensor Werte mit einer sehr hohen
Abtastrate (1kHz) abzufragen, um den Fehler sehr gering zu halten. Zusätzlich
können bei Aktivierung des Lift-Bits interpolierte werte als Ersatz in die
Messdaten integriert werden.

TODO: Sollte noch auf das Prüfen

* Median Filter Problem und WLAN Latenz

Ursprünglich wurden die Messdaten mit einer sehr geringen Frequenz abgefragt.
Dadurch entstand der oben genannte Fehler. Um diese kleinen Messwerte
herauszufiltern wurde ein Median-Filter auf der PC-Seite eingesetzt.
Allerdings verschlimmerte dieser die Messergebnisse. Da die Kommunikation über
WLAN erfolgt entstehen Latenzen bei der Übertragung der Messdaten. Bei der
Abfrage der MEssdaten wird erst auf eine Antwort der aktuellen anfrage an das
BBB gewartet. Erst wenn diese eingetroffen ist wird nach dem nächsten Paket
gefragt. Die übertragenen Daten bleiben zwar korrekt, allerdings kann die
Übertragung von Paketen länger dauern. Dadurch hat der Sensor mehr Zeit
Messdaten zu sammeln. Langsamere Pakete enthalten dadurch höhere
Displacement-Werte. Da der Median diese als Ausreißer weg-filtert obwohl
diese korrekt sind, verschlimmert dieser die Messergebnisse.

Durch das Speichern der Messwerte mit einer hohen sample-rate auf dem BBB
entsteht dieser Fehler nicht mehr.

* SPI Device Tree
Das konfigurieren des Device Trees für SPI1 mit 2 Chip-Selects unter Ubuntu
brachte einige Probleme mit sich. Das Device Tree File musste mit den richtigen
GPIO Pins erweitert werden. Zusätzlich musste HDMI deaktiviert werden.

\section fazit Fazit und Erkenntnisse

Mit einer hohen sample rate ist der Lift-Bit Fehler sehr gering. Mit einem
geeignetem Filter könnte der Fehler noch weiter verringert werden. Bei 1kHz kann
dieser vernachlässigt werden. Zusätzlich könnten anstatt aufsummieren der Daten
auf dem BBB diese auch als Schritte abgespeichert werden. Allerdings reichen die
Abfrage-Intervalle der GUI vollkommen aus um schnelle Kurven zu erkennen. Auch
die WLAN Latenzen stören das Messergebnis nicht. Der Sensor arbeitet auf 1-2cm
auf 100cm genau. Unabhängig davon driften die gemessenen Werte ab ca. 1m etwas
ab. Dies ist aufgrund der Lego-Montierung welche sich immer wieder verschiebt.
Trotz kalibrieren sind die Messwerte nicht optimal.

TODO: Bild mit Drift

Nur ein Sensor wird zum Bestimmen der Position und Ausrichtung benötigt.

\section improvement Verbesserungen

* Filter beim Lift-Bit einführen
* Mehr teile vom PC zum BBB verlagern
* Einzelne Schritte abspeichern anstatt aufsummieren
* Bessere Hardware zum Montieren verwenden
** die nicht wackelt
** Gleicher Abstand der Sensoren zum Boden ohne schleifen
* ROS Topics anstatt Services verwenden (optional)

\image html sensor_test_car.jpg "Testfahrzeug mit 2 PLN2033"