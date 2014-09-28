Ausblick {#ausblick}
===

\tableofcontents

\section fazit-ausblick Fazit

Aufgrund verschiedener Faktoren konnte die Ziel des Projekts nicht erreicht
werden.

Zum einen bietet Lego keine stabile Platform für die Maussensoren. 

Durch das instabile Verhalten der Konstruktion können die Maussensoren nicht
optimal auf dem Roboter montiert werden. Da der Sensor nicht im optimalen
Winkel (Y-Achse geradeaus) am Roboter befestigt ist, wird dieser kalibriert.
Nach der Kalibrierung ergibt eine geradeaus Fahrt nur eine Änderung der
Y-Koordinaten. Trotz Kalibrierung verändert sich während der Fahrt die Position
des Sensors auf dem Roboter leicht. Da während der Fahrt nicht mehr kalibriert
werden kann errechnet die Software trotz geradeaus Fahrt eine Kurvenfahrt, da
sich nun Y- und X-Koordinaten ändern.

Die Maussensoren müssen außerdem einen gewissen Abstand zum Boden konstant
einhalten. Da das Legogerüst zu instabil ist, liegen die Sensorplatinen auf 
Schraubenköpfen auf dem Boden auf und schleifen damit auf dem Boden. Dadurch
ändert sich der Abstand der Sensoren zur Oberfläche je nach Roboterbewegung
sehr. Ist der Abstand zu groß wird das Lift-Bit des Sensors gesetzt, er liefert
keine Messungen mehr. Der Untergrund des Labyrinths ist sehr uneben sodass die 
Maussensoren während der Fahrt auf und ab wippen und somit schlechte Messungen
liefern.

Neben den Problemen mit den Maussensoren liefern auch die 
Ultraschallsensoren von Lego nicht die gewünschten Ergebnisse. Es kann 
passieren, dass die Ultraschallsensoren falsche, also deutlich zu 
große oder zu kleine Werte liefern. Wenn dieser Fehler nur einmal 
vorkommen würde, wäre es kein großes Problem diese falschen Werte 
mit Hilfe eines Medianfilters heraus zu filtern. Da die Sensoren aber 
für mehrere Messungen denselben falschen Wert liefern, muss man den Medianfilter 
vergrößern um die Fehler herauszufiltern. Dies führt jedoch 
wiederum zu einer verlängerten Reaktionszeit des Roboters, wodurch 
der Roboter Hindernissen nicht mehr rechtzeitig ausweichen kann. 
Beim \ref graphen-ansatz fängt der Roboter durch die Fehlmessungen an seine 
Position falsch zu korrigieren. Die Ursache für diese Fehler ist 
unbekannt. Mehr dazu siehe: \ref ultraschallsensoren-roboter.

Zuletzt sei noch gesagt, dass das ROS-Framework überdimensioniert für 
dieses Projekt war und mehr Einarbeitungsaufwand generiert hat, als es 
später eine Erleichterung war. Da wir nur einen Microcontroller (das BBB) 
benutzt haben, war unser System nicht wirklich verteilt. Daher sind 
auch die ROS Kommunikationsmechanismen eher unnötiger Overhead 
gewesen. Ohne ROS wäre außerdem auch eine Cross-Kompilierung unserer 
Software möglich gewesen. Da unsere Programme jedoch auch dem BBB 
kompiliert werden mussten, nahm dieser Vorgang jedes mal zwischen 10 und 
20 Minuten in Beschlag.

Trotz all der negativen Punkte und dem gescheiterten Projektziel, war 
das Projekt für alle beteiligten ein Erfolg, da wir viel gelernt und 
viel Spaß im Roboterlabor hatten.

\section ausblick-ausblick Ausblick

Für zukünftige Projekte wäre natürlich die __Implementierung der 
Maussensoren in den Roboter__ ein realistisches Ziel. Dazu wäre jedoch 
noch mehr Forschungsarbeit an den Sensoren notwendig. Darunter fällt:

* Auswirkung von Licht auf die Sensoren (evtl. abdunkeln)
* Stabilere Montage der Sensoren
* grobe Fehlertoleranz der Sensoren herausfinden
* bessere Gleitfähigkeit der Sensorunterlage

Ein weiteres Problem waren natürlich die __Ultraschallsensoren__ von Lego. 
Die Ursache der genannten Fehlmessungen und warum die Sensoren mehrere 
Messungen auf einem Sensorwert beharren sollte herausgefunden werden. 
Der Zugriff auf die Sensoren ist jedoch nicht so einfach, da dieser 
stark durch die Lego Firmware abstrahiert wird. Man könnte

* das Auslesen der Sensoren direkt auf dem Brick mit NXC realisieren

Dies ist jedoch sehr aufwändig und kann evtl. auch Performance 
Probleme bereiten. Außerdem ist es absolut nicht sicher, ob diese 
Variante zum Erfolg führt. Für eine sichere Navigation in einem 
Labyrinth sind die Ultraschallsensoren trotzdem unerlässlich. Unsere 
Vorgängergruppe hat ebenfalls einen Ultraschallsensor benutzt, 
um ein Hindernis zu registrieren und entsprechend zu handeln. Die 
Registrierung eines Objektes - also ob etwas im Weg ist oder nicht - 
ist mit den Sensoren auch problemlos möglich. Jedoch ist während 
unserem Projekt die Frage aufgekommen, ob die Lego Ultraschallsensoren 
überhaupt dazu gemacht sind Distanzen konkret zu messen, um z.B. die 
Umgebung zu karthografieren. Da wir über kein Datenblatt der Sensoren 
verfügen und somit auch nicht deren Fehlertoleranz kennen, kann es 
durchaus sein, dass die Sensoren ungeeignet sind.

Sind zuguter letzt alle Hardwareprobleme gelöst, können Navigations- 
und Erkundungsalgorithmen implementiert werden.

Als Resultat dieses Projektes würden wir jedoch empfehlen, dass ein 
zukünftiges Labyrinth-Projekt

* einen normalen Roboter verwendet, anstatt Lego Mindstorm
* einen kleineren und einfacheren Microcontroller nutzen (ohne Linux)
* kein ROS benutzt, da es überdimensioniert für so ein Projekt ist

Als Vorschlag für einen alternativen Roboter, könnte der __Pololu 
3pi__ genannt werden.
