Projekt {#projekt}
===

\tableofcontents

\section Beschreibung Beschreibung

Die Aufgabe des Roboters war es __ein Labyrinth zu karthographieren__. 
Als Roboter wurde ein __Lego NXT Brick__ verwendet. Der Roboter verfügt 
über einen __Differentialantrieb__ und kann seine Umgebung mithilfe von 
__Ultraschallsensoren__ warnehmen.

Zusätzlich wurde auf dem Roboter ein __Beagle Bone Black__ montiert. 
Dieses bietet __mehr Rechenleistung und eine flexiblere 
Programmierung__ als der Brick, auf welchem lediglich in der 
beschränkten Programmiersprache NXC gearbeitet werden kann. Außerdem 
wird das BBB benötigt, um zusätzliche __Mauslasersensoren__ über 
einen SPI-Bus anzusprechen. Diese Maussensoren werden verwendet um die 
Bewegung des Roboters zu messen und somit dessen __Odometrie zu 
verbessern__.

---

\section Struktur Struktur

Das Projekt ist in verschiedene __ROS-Packages__ unterteilt, sodass zu 
jeder Zeit immer nur __die wirklich benötigten Packages kompiliert 
werden können__. Dies ist vor allem in Hinblick auf die Lange 
kompilierdauer auf dem Beagle Bone nützlich. Die einzelnen Packages 
werden in den folgenden Abschnitten kurz beschrieben. Für weitere 
Informationen kann die __Code Dokumentation__ herangezogen werden.

\subsection minotaur_common minotaur_common

Dieses Package beinhaltet __grundlegende Strukturen und Funktionen__, die 
__von allen anderen Packages verwendet__ werden. Darunter sind 
verschiedene Funktionen zur Umrechnung von Maßeinheiten, Funtkionen 
für das Auslesen von Parametern vom ROS-Param-Server, Funktionen zur 
Kommunikation über ROS und Definitionen für ROS-Topics. Durch dieses 
Package wird vor allem __Codeduplikationen vorgebeugt__. Das Package 
muss __immer__ im Zusammenhang mit jedem anderen Package kompiliert 
werden.

\subsection minotaur_common_qt minotaur_common_qt

Auch hierbei handelt es sich um ein allgemeines Package, das 
praktische Funktionen und Strukturen anbietet. Dabei ist dieses 
Package speziell __für Aufgaben im Zusammenhang mit QT__ zuständig. Da 
solche Funktionen beispielsweise __auf dem Beagle Bone nicht benötigt__ 
werden, haben diese eine seperates Package erhalten. Die meisten QT 
nutzenden Packages verwenden auch dieses Package.

\subsection robot_control robot_control

---

\section Probleme Probleme

Im Verlauf des Projektes sind __mehrere Probleme__ aufgetreten, deren 
Lösung bis zum derzeiten Zeitpunkt noch ausstehen.

\subsection Ultraschallsensoren Ultraschallsensoren

Die Ultraschallsensoren des NXT registrieren in scheinbar zufälligen 
Abständen Fehlmessungen, die jenseits einer Fehlertoleranz liegen. 
Dabei handelt es sich um Fehlmessungen im Bereich von +-20cm. Diese 
könnten theoretisch durch einen Medianfilter ausgeglichen werden, was 
in diesem Projekt auch versucht wurde. Tritt jedoch so eine 
Fehlmessung auf, liefert der Sensor für die nächsten 4 bis 8 
Messungen denselben falschen Wert. Dadurch kann auch der Medianfilter 
diesen Fehler nicht mehr ausgleichen.

Es liegt der Verdacht nahe, dass der Sensor die folgenden Male den 
gleichen Sensorwert liefert, weil er keine weitere Messung unternommen 
hat. Jedoch wird bei der Messung kein Zeitstempel hinterlegt, wodurch 
sich nicht unterscheiden lässt, ob es sich um die gleiche Messung oder 
nur zufällig um den gleichen Wert bei einer weiteren Messung handelt. 
Die Ansteuerung des Ultraschallsensors geschieht durch das Direct 
Command Interface des Brick, wodurch kein dediziertes Programm auf dem 
Brick laufen muss.

Ein Vergrößern des Medianfilters oder eine Verlängern des 
Abtastintervalls führt zu einer Reduktion der Wirkung des Fehlers. Der 
größere Medianfilter filtert den Fehler entweder ganz weg oder dieser 
tritt nur einen kurzen Moment auf. Jedoch ist die Reaktionszeit des 
Roboter durch den großen Filter extremst verlängert und zumeist 
unzureichend. Der Roboter kann dann nicht auf Umgebungsänderungen wie 
das Auftauchen einer Wand reagieren. Daselbe gilt auch für ein 
verlängertes Abtastintervall. Es führt zu weniger Messungen mit 
derselben Fehlmessung. Damit wird auch bestätigt, dass immer wieder 
derselbe Messwert aus der gleichen Messungen geliefert wird. Die 
Reaktionszeit des Roboters sinkt jedoch auch in diesem Fall in einen
unzureichenden Bereich.

Durch die Abstraktion des Bricks in Hinsicht auf das Auslesen des 
Sensors, ist es auch nicht möglich direkt auf die Hardware des Sensors 
zuzugreifen. Daher konnte der Sensor auch nicht direkt, sondern nur 
über das Direct Command Interface ausgelesen werden.

Aufgrund des großen Messfehlers und der wiederauftretenden Messwerte 
ist eine Navigation im Labyrinth für den Roboter kaum möglich.

\subsection Maussensoren Maussensoren

Bla Bla Jens

---

\section Ansätze Ansätze

Im Rahmen dieses Projektes wurden mehrere Lösungsansätze verfolgt.

1. @subpage histogramm
2. @subpage graphen

Im ersten Ansatz wurde eine Histogrammkarte angefertigt und der Roboter 
sollte sich anhand dieser Karte und mithilfe des ROS Navigation Stacks 
durch das Labyrinth bewegen.

Im zweiten Ansatz wurde der Roboter direkt angesteuert. Das Labyrinth 
wurde als Graph betracht und in Zellen gleicher Größe unterteilt.

