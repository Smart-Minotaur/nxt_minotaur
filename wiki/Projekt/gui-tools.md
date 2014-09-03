GUI Tools {#gui-tools}
===

\tableofcontents

\section pid_monitor pid_monitor

Im folgenden soll die Anwendung __pid_monitor__ näher beschrieben 
werden. Dabei wird auf ihren Nutzen und die Bedienung eingegangen.

\subsection BeschreibungPID Beschreibung

Die Anwendung __pid_monitor__, das sich im gleichnamigen Package befindet, 
realisiert eine grafische Anwendung um den __PID-Regler der RobotControl 
Node__ zu beobachten. Außerdem werden Hindernisse, die von den 
Ultraschallsensoren registriert werden, dargestellt. Die aktuellen 
Proportional-, Integral- und Differentialanteile des Roboters lassen 
sich genauso wie die Geschwindigkeit und Winkelgeschwindigkeit durch 
Schieberegler einstellen.

An den integrierten Graphen kann das __Einschwingverhalten des 
PID-Regler__ beobachtet werden. Weiterhin können 
durch das [Ziegler–Nichols Verfahren](http://de.wikipedia.org/wiki/Faustformelverfahren_%28Automatisierungstechnik%29#Methode_von_Ziegler_und_Nichols)
die optimalen PID-Parameter ermittelt werden.

\subsection BedienungPID Bedienung

Die Bedienung soll anhand des folgenden Screenshots erläutert werden.

\image html pid-monitor.png

Mit den Schiebereglern im rot umrahmten Bereich können die __Parameter 
für den PID-Regler__ eingestellt werden. __Kp__ repräsentiert hierbei den 
Proportional-, __Ki__ den Integral- und __Kp__ den Differentialanteil.

In dem hellblauen Bereich befinden sich die Schieberegler um die 
Geschwindigkeit und die Winkelgeschwindigkeit einzustellen. Mit __v__ 
kann die Geschwindigkeit und mit __w__ die Winkelgeschwindigkeit 
geregelt werden. Der Button __brake__ stoppt die Motoren des Roboters.

Der gelbe Bereich kennzeichnet die Geschwindigkeitsanzeigen. Auch gilt 
für __v__ und __w__ das gleich wie im roten Bereich. Unter der 
Überschrift __Target Velocity__ befinden sich die eingestellten 
Sollgeschwindigkeiten. Bei __Measured Velocity__ befinden sich die 
tatsächlich gemessenen Geschwindigkeiten, die über das 
ROS-Kommunikationssystem empfangen werden.

Im grünen Bereich befinden sich nun die Graphen, die die 
Geschwindigkeit und Winkelgeschwindigkeit über die Zeit auftragen. Die 
__Y-Achse__ stellt die gemessene Geschwindigkeit in m/s dar (bei 
Winkelgeschwindigkeit Rad/s). Die X-Achse 
zeigt die Nummer der Messung an. 3 bedeutet hierbei die 3. Messung. 
Die Zeit zwischen 2 Messungen ist dabei vom eingestellten 
Abtastintervall abhängig.

Der dunkelblaue Bereich zeigt die Ultraschallsensormessungen. Der 
__blaue Punkt__ stellt den Roboter dar und die __roten Punkte__ zeigen 
Hindernisse, die von den Sensoren registriert werden.

\section map_monitor map_monitor

\subsection BeschreibungMAP Beschreibung

\subsection BedienungMAP Bedienung

\image html map-monitor.png

\section mouse_monitor_pc mouse_monitor_pc

