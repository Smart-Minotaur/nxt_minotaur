GUI Tools {#gui-tools}
===

\tableofcontents

\section pid_monitor-gui-tools pid_monitor

Im folgenden soll die Anwendung __pid_monitor__ näher beschrieben 
werden. Dabei wird auf ihren Nutzen und die Bedienung eingegangen.

\subsection beschreibung-pid-gui-tools Beschreibung

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

\subsection bedienung-pid-gui-tools Bedienung

Um __pid_monitor__ zu starten werden ein Beagle Bone Black und ein 
Computer benötigt. Beide müssen sich im gleichen Netzwerk 
befinden. Nun muss in das Wurzelverzeichnis des Repositorys gewechselt 
werden. Dort muss nun die Datei __start__ mit einem beliebigen Editor
geöffnet und die IP Variablen sowohl auf dem BBB als auch auf dem PC __mit 
den entsprechenden IPs__ belegt werden. Zuerst muss die Anwendung auf dem PC 
mit folgendem Befehl gestartet werden:

~~~
./start pm_p
~~~

Auf dem BBB nun folgenden Befehl ausführen:

~~~
./start rc_b
~~~

Die Bedienung des Programms soll anhand des folgenden Screenshots 
erläutert werden.

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
Die Zeit zwischen zwei Messungen ist dabei vom eingestellten 
Abtastintervall abhängig.

Der dunkelblaue Bereich zeigt die Ultraschallsensormessungen. Der 
__blaue Punkt__ stellt den Roboter dar und die __roten Punkte__ zeigen 
Hindernisse, die von den Sensoren registriert werden.

\section map_monitor-gui-tools map_monitor

\subsection beschreibung-map-gui-tools Beschreibung

Die Anwendung __map_monitor__ dient der Beobachtung der Odometrie des 
Roboters und der Aufzeichnung der Umgebungskarte. Mithilfe der 
Odometrie und Sensordaten des Roboters werden seine Bewegungsroute und 
Hindernisse in der Umgebung in eine grafische Oberfläche 
eingezeichnet. Mithilfe dieser Anwendung konnte festgestellt werden, 
wie schlecht die Odometrie des Roboters tatsächlich ist.

\subsection bedienung-map-gui-tools Bedienung

Um __map_monitor__ zu starten werden ein Beagle Bone Black und ein 
Computer benötigt. Beide müssen sich im gleichen Netzwerk 
befinden. Nun muss in das Wurzelverzeichnis des Repositorys gewechselt 
werden. Dort muss nun die Datei __start__ mit einem beliebigen Editor
geöffnet und die IP Variablen sowohl auf dem BBB als auch auf dem PC __mit 
den entsprechenden IPs__ belegt werden. Zuerst muss die Anwendung auf dem PC 
mit folgendem Befehl gestartet werden:

~~~
./start map_p
~~~

Auf dem BBB nun folgenden Befehl ausführen:

~~~
./start rc_b
~~~

Die Bedienung des Programms soll anhand des folgenden Screenshots 
erläutert werden.

\image html map-monitor.png

Die Anwendung zeigt im Hauptfenster die aufgezeichnete Karte. Die 
roten Bereiche stellen hierbei die Hindernisse in der Umgebung dar. 
Schwarze Quadrate sind ehemalige Positionen des Roboters. Das blaue 
Quadrat ist die aktuelle Position des Roboters. Außerdem wird im 
Kopfbereich der Anwendung die aktuelle Position des Roboters in Zahlen 
dargestellt.

Um den Roboter zusätzlich bewegen zu können, kann die Applikation 
__pid_monitor__ verwendet werden. Dazu muss auf dem PC folgender 
Befehl ausgeführt werden.

~~~
./start pm_p
~~~

Mehr Informationen zu __pid_monitor__ finden sich im Abschnitt \ref 
pid_monitor-gui-tools.

\section mouse_monitor_pc-gui-tools mouse_monitor_pc

