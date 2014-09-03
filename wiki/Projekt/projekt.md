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
werden können__. Dies ist vor allem in Hinblick auf die lange 
Kompilierdauer auf dem Beagle Bone nützlich. Die einzelnen ROS-Packages 
werden im folgenden Abschnitt \ref Packages kurz beschrieben. Für weitere 
Informationen können die __Code Dokumentation__ und die genannten 
Abschnitte und Wikiseiten herangezogen werden.

In diesen ROS-Packages sind verschiedene ROS-Nodes enthalten, die alle 
miteinander über das ROS-Kommunikationsystem kommunizieren. Die Art 
und Richtung der Kommunikation wird in dem Abschnitt \ref Datenfluss 
erläutert.

\subsection Packages Packages

Das Package __minotaur_common__ beinhaltet __grundlegende Strukturen und Funktionen__, die 
__von allen anderen Packages verwendet__ werden. Darunter sind 
verschiedene Funktionen zur Umrechnung von Maßeinheiten, Funtkionen 
für das Auslesen von Parametern vom ROS-Param-Server, Funktionen zur 
Kommunikation über ROS und Definitionen für ROS-Topics. Durch dieses 
Package wird vor allem __Codeduplikationen__ vorgebeugt. Das Package 
muss __immer__ im Zusammenhang mit jedem anderen Package kompiliert 
werden.

Auch bei __minotaur_common_qt__ handelt es sich um ein allgemeines Package, das 
praktische Funktionen und Strukturen anbietet. Dabei ist dieses 
Package speziell __für Aufgaben im Zusammenhang mit QT__ zuständig. Da 
solche Funktionen beispielsweise __auf dem Beagle Bone nicht benötigt__ 
werden, haben diese eine seperates Package erhalten. Die meisten QT 
nutzenden Packages verwenden auch dieses Package.

Das Package __robot_control__ umfasst die Implementierung des __PID-Reglers__ für die 
Motoren, das Auslesen der Ultraschallsensoren und das Verarbeiten von 
ROS-Messages, um den Brick anzusteuern. Außerdem wird hier die 
__ROS-Node RobotControl__ bereitgestellt, welche die 
Kommunikatiosgrundlage aller anderen Packages darstellt. Mehr dazu in 
dem Abschnitt \ref RobotControl.

In dem Package __pid_monitor__ wird eine __GUI-Applikation__ realisiert, mit der sich 
das Verhalten des PID-Reglers in Form von Graphen betrachten lässt. 
Der Roboter kann außerdem ferngesteuert werden. So können dessen 
Einschwing- und Ausgleichverhalten beobachtet werden.
Neben den Motoren bietet die Applikation auch die Möglichkeit 
Hindernisse mit den Ultraschallsensoren wahrzunehmen. Mehr dazu im 
Abschnitt \ref GUI-Tools.

Das Package __minotaur_map__ realisiert den __ersten Ansatz__ dieses Projektes. Es 
beinhaltet Funktionen und Strukturen um __Histogrammkarten__ zu 
erzeugen und durch diese hindurch zu navigieren. Außerdem nutzt es 
die __ROS Navigation Stack Schnittstelle__, um den Roboter anzusteuern. 
Mehr Informationen dazu finden sich im Abschnitt \ref Ansätze.

Bei __map_monitor__ handelt es sich um ein Package, das eine __GUI-Applikation__ für 
die Histogrammkarte realisiert. Der Roboter kann angesteuert werden 
während __eine Karte live mitgezeichnet__ wird. Mit diesem Tool kann 
also die Qualität der Karte und auch das Verhalten des 
Erstellungsalgorithmus beobachtet und bewertet werden. Mehr 
Informationen finden sich im Abschnitt \ref GUI-Tools.

In dem Package __minotaur_maze__ wird der __zweite Ansatz__ dieses Projektes 
realisiert. Es enthält Strukturen und Funktionen, um den Roboter in 
einem __graphenähnlichen Labyrinth__ zu bewegen und eine Karte dieses 
Graphen zu erstellen. Mehr dazu im Abschnitt \ref Ansätze.

__mouse_monitor_beagle__ TODO Jens

__mouse_monitor_pc__ TODO Jens

\subsection Datenfluss Datenfluss

TODO Datenflussbild

---

\section Implementierung Implementierung

Das Projekt umfasst viele einzelne __ROS-Nodes__ (Applikationen). Dabei 
handelt es sich um __ineinandergreifende Anwendungen__, die über das 
ROS-Kommunikationssystem zusammenarbeiten. Einige davon dienen 
ausschließlich __Debuggingzwecken__ und sind zumeist graphischer Natur.
Ihre __Funktionsweise__ und das korrekte __Starten dieser Anwendungen__ 
(beispielsweise zur remote Steuerung des Roboters) wird 
in den folgenden Abschnitten näher beleuchtet.

\subsection RobotControl RobotControl

__RobotControl__ ist die zentrale ROS-Node des Projektes. Sie wird in 
dem Package \ref robot_control realisiert.

Die Node übernimmt das __Umrechnen von Geschwindigkeit und 
Winkelgeschwindigkeit__ auf Geschwindigkeiten der einzelnen Motoren. 
Außerdem regelt sie über einen __PID-Regler__ die Geschwindigkeiten der 
Motoren getrennt. Zur Regelung werden außerdem __die Geschwindigkeiten 
der Motoren ausgelesen__ und wiederrum in Geschwindigkeit und 
Winkelgeschwindigkeit umgerechnet. Die angeschlossenen 
__Ultraschallsensoren__ werden __in regelmäßigen Intervallen 
ausgelesen__.

Weiterhin nimmt RobotControl __ROS-Messages__ entgegen, mit denen die 
Geschwindigkeiten des Roboters gesetzt werden können. Die von den 
Motoren __gemessenen Geschwindigkeiten__ werden wiederum __in regelmäßigen 
Abständen an eine ROS-Topic gesendet__. Dabei werden für die 
Nachrichten die __Standard-ROS-Topics für den ROS Navigation Stack__ 
verwendet, sodass der Roboter __vollständig kompatibel zu dem ROS 
Navigation Stack__ ist. Dies bedeutet, dass Odometriedaten an die Topic 
\b /odom geschickt und Geschwindigkeitsbefehle von der Topic 
\b /cmd_vel entegegen genommen werden.

Die Ultraschallsensoren können über __eigene Topics und Messages__
angesteuert werden. So kann mit einer Message an die Topic 
\b /add_ultrasonic der Node mitgeteilt werden, dass sich an dem 
angegebenen Port ein Ultraschallsensor befindet und dieser nun auch 
ausgelesen werden soll. Dabei verwendet RobotControl ein __internes 
Mapping von Sensor-IDs auf Sensorports__, sodass bei den Sensormessungen 
immer klar ist welche Messung von welchem Sensor kommt. Die ID wird 
beim Hinzufügen des Sensors über eine ROS-Message zurückgegeben. 
Mehr Topics und deren Funktion können der Datei MinotaurTopics.hpp und 
der __Code Dokumentation__ entnommen werden.

Eine weitere komfortable Funktion von RobotControl ist, dass __sämtliche 
Einstellungen__ des PID-Reglers, Dimensionen des Roboters und 
angeschlossene Sensoren __über den ROS-Param-Server geladen werden 
können__. Diese Daten müssten ansonsten über ROS-Messages eingestellt 
werden. Dabei werden die Einstellungen auf dem ROS-Param-Server unter 
dem Namespace __minotaur__ hinterlegt. Danach folgt der __Name der 
Einstellungen__ und darunter die einzelnen Parameter. Die ganze Struktur 
kann aus der Datei models.yaml entnommen werden. Beim Start der 
RobotControl Node schaut diese nach dem Parameter __CurrentModel__. Ist 
dieser vorhanden wird dessen Inhalt als Name der initialen Einstellungen 
behandelt und die Node lädt diese Einstellungen.

Damit übernimmt diese ROS-Node die __komplette Ansteuerung des Brick__ und 
abstrahiert diese auf ROS-Messages. Damit muss lediglich diese Node auf 
dem Beagle Bone Black ausgeführt werden. Alle anderen Nodes können 
ebenfalls auf dem BBB oder aber auch auf einem anderen Computer im 
selben Netzwerk ausgeführt werden.

\subsection GUI-Tools GUI Tools

Während dieses Projektes wurde es notwendig __grafische Anwendungen__ zu 
entwickeln, um bestimmte __Systemteile zu debuggen__ und deren Verhalten 
zu beobachten. Alle Anwendungen wurden mit QT4 und der Erweiterung QWT 
geschrieben. QWT bietet grafische Komponenten wie Graphen, wodurch das 
Analysieren von Daten vereinfacht wird. Alle in diesem Projekt 
entwickelten grafischen Anwendungen werden auf der folgenden Seite 
näher beschrieben:

* @subpage gui-tools

\subsection Maussensoren1 Maussensoren

Da die Odometrie der Lego NXT Motoren nicht sehr gut ist und vor allem 
bei Drehungen des Roboter oft Fehler verursachen, __da die Reifen 
durchdrehen__, sollten __Maussensoren__ genutzt werden, um die Odometrie zu 
verbessern. Genauere Informationen zur Implementierung, dem 
Entwicklungshergang und gelösten wie ungelösten Problemen finden sich 
auf der folgenden Seite.

* @subpage maussensoren

\subsection Ansätze Ansätze

Im Rahmen dieses Projektes wurden mehrere Lösungsansätze verfolgt.

1. @subpage histogramm
2. @subpage graphen

Im __ersten Ansatz__ wurde eine __Histogrammkarte__ angefertigt und der Roboter 
sollte sich anhand dieser Karte und mithilfe des ROS Navigation Stacks 
durch das Labyrinth bewegen.

Im __zweiten Ansatz__ wurde der Roboter direkt angesteuert. Das Labyrinth 
wurde als __Graph__ betracht und in Zellen gleicher Größe unterteilt.

---

\section Probleme Bleibende Probleme

Im Verlauf des Projektes sind __mehrere Probleme__ aufgetreten, deren 
Lösung bis zum derzeiten Zeitpunkt noch ausstehen.

\subsection Ultraschallsensoren Ultraschallsensoren

Die Ultraschallsensoren des NXT registrieren __in scheinbar zufälligen 
Abständen Fehlmessungen__, die jenseits einer Fehlertoleranz liegen. 
Dabei handelt es sich um Fehlmessungen im Bereich von \b +-20cm. Diese 
könnten theoretisch __durch einen Medianfilter ausgeglichen werden__, was 
in diesem Projekt auch versucht wurde. Tritt jedoch so eine 
Fehlmessung auf, liefert der Sensor für __die nächsten 4 bis 8 
Messungen__ denselben falschen Wert. Dadurch kann auch der Medianfilter 
diesen Fehler nicht mehr ausgleichen.

Es liegt der Verdacht nahe, dass der Sensor die folgenden Male den 
__gleichen Sensorwert__ liefert, weil er keine weitere Messung unternommen 
hat. Jedoch wird bei der Messung __kein Zeitstempel__ hinterlegt, wodurch 
sich nicht unterscheiden lässt, ob es sich um die gleiche Messung oder 
nur zufällig um den gleichen Wert bei einer weiteren Messung handelt. 

Ein __Vergrößern des Medianfilters__ oder eine __Verlängern des 
Abtastintervalls__ führt zu einer __Reduktion der Wirkung des Fehlers__. Der 
größere Medianfilter filtert den Fehler entweder ganz weg oder der 
Fehler tritt nur einen kurzen Moment auf. Jedoch ist die __Reaktionszeit des 
Roboter__ durch den großen Filter __extremst verlängert und zumeist 
unzureichend__. Der Roboter kann dann nicht auf Umgebungsänderungen wie 
das Auftauchen einer Wand reagieren. __Daselbe gilt auch für ein 
verlängertes Abtastintervall__. Es führt zu weniger Messungen mit 
derselben Fehlmessung. Damit wird auch bestätigt, dass immer wieder 
derselbe Messwert aus der gleichen Messungen geliefert wird. Die 
Reaktionszeit des Roboters sinkt jedoch auch in diesem Fall in einen
unzureichenden Bereich.

Die __Ansteuerung der Ultraschallsensoren geschieht durch das Direct 
Command Interface des Brick__, wodurch kein dediziertes Programm auf dem 
Brick laufen muss. Durch __die Abstraktion des Bricks__ in Hinsicht auf das 
Auslesen des Sensors, ist es auch __nicht möglich direkt auf die 
Hardware des Sensors zuzugreifen__. Daher konnte der Sensor auch nicht 
direkt, sondern nur über das Direct Command Interface ausgelesen werden.

Aufgrund des großen Messfehlers und der wiederauftretenden Messwerte 
ist __eine Navigation in einem Labyrinth für den Roboter kaum möglich__.

\subsection Maussensoren2 Maussensoren

TODO Jens

