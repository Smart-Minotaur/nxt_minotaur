Graphen Ansatz {#graphen}
===

\tableofcontents

\section beschreibung-graphen Beschreibung

Da der \ref histogramm ohne gute Odometrie nicht gut umsetzbar ist wurde ein zweiter Ansatz gestartet. 
Das Labyrinth wurde in quadratische Zellen eingeteilt. Die Seitenlänge einer Zelle enspricht dabei
der Länge einer Holzwand aus dem Labyrinth. Der Roboter bewegt sich dann stets __von Zelle zu Zelle__.
Wenn der Roboter etwa in der Mitte einer Zelle angekommen ist prüft er zunächst in welchen Richtungen 
sich Wände befinden und in welche Richtungen er weiterfahren kann. Dies wird in einer __Karte__ dokumentiert 
und anschließend wird durch einen Algorithmus bestimmt welche benachbarte Zelle der Roboter als nächstes 
ansteuert.

\section umsetzung-graphen Umsetzung

Die Soucedateien sind im package __minotaur_maze__ zu finden.

\subsection navigation-graphen Navigation

Bei einer Fahrt von Zelle zu Zelle kann der Roboter aufgrund von Ungenauigkeiten der Motoren sowie Durchdrehen eines Rades eine schlechte Ausrichtung und somit fehlerhaftes Verhalten entwickeln.
Da keine Maussensoren zum erkennen dieser Fehler zur Verfügung stehen, müssen wir dies anhand der Messwerte der __Ultraschallsensoren__ ausgleichen. Hierbei werden die Werte der Sensoren rechts und 
links verglichen und es wird eine Korrektur in die Wege geleitet, die versucht den Abstand beider Sensoren zur Wand gleich zu halten. 
Je höher dabei der Unterschied der Sensorwerte, desto höher das Entgegenlenken durch die __Korrektur__. 
Wenn ein Sensor keine nützlichen Messwerte liefern kann, da sich an seiner Seite keine Wand sondern eine Abzweigung befindet, werden nur die Daten des anderen Sensors mit einem __Sollabstand__ verglichen. 

Um einzelne Ausreisser in den Messwerten auszugleichen wurde ein __Medianfilter__ angewendet der die letzten 5 bis 10 Werte eines Sensors nutzt.

Beim Fahren entscheidet der Roboter anhand des zurückgelegten Weges, ob er in der nächsten Zelle angekommen ist. Daraufhin wird ermittelt welche 
Aktion als nächstes ausgeführt wird (Mehr dazu siehe \ref algorithmus-graphen ). Der Roboter hat hier immer genau zwei Möglichkeiten: 

* Rotation: Der Roboter dreht sich auf der Stelle in eine angegebene Richtung um 90° oder 180°.
* Fahrt geradeaus: Der Roboter bewegt sich zur nächsten Zelle.


\subsection algorithmus-graphen Algorithmus

Es handelt sich hierbei um einen sehr simplen Algorithmus. Nachdem erkannt wurde in welche Richtung ein Weiterfahren möglich ist, wird der Roboter,
falls mehrere Möglichkeiten zur Verfügung stehen, unbesuchte Zellen als Ziel bevorzugen. 
Wenn mehrere unbesuchte Zellen zur Verfügung stehen wird momentan bevorzugt geradeaus gefahren, da bei Drehungen ab und an Fehler auftreten können. 
Daher wird versucht, so wenige Rotoationen wie möglich zu machen. Mehr dazu siehe: \ref probleme-graphen.

\section probleme-graphen Probleme

Auch in diesem Ansatz entstehen viele Probleme und Fehler aufgrund von __ungenauen Sensorwerten__ und __schlechter Odometrie__.

Besonders anfällig für Fehler sind die Rotationen. Es kann passieren, dass der Roboter vor der Rotation nicht gerade steht oder bei der Rotation ein Rad durchdreht. 
Als Resultat dreht sich der Roboter zu weit oder zu wenig und fährt im schlechtesten Fall anschließend gegen eine Wand, da er nicht schnell genug gegenlenken kann. 
Diese Probleme könnten eventuell beide durch funktionsfähige Maussensoren abgefangen und korrigiert werden, die uns jedoch nicht zur Verfügung stehen.

Aber auch beim einfachen geradeausfahren können Fehler auftreten. So kann es vorkommen, dass der Roboter einem von ihm erkannten Fehler gegensteuern will, 
dies jedoch viel zu stark macht und er kann sich nicht mehr gerade in der Mitte halten und fährt entweder gegen eine Wand oder steht vor der nächsten Rotation enorm schief.
Dieses Problem tritt zumeist aufgrund von unerklärlich schlechten Sensormesswerten auf. 
Wir haben versucht dieses Problem mittels eines Medianfilters zu beheben, jedoch ohne großen Erfolg.

Das Problem des Medianfilters besteht darin, dass die Ultraschallsensoren stets mehrere fehlerhafte Werte in Folge liefern und der Filter dadurch nutzlos wird. 
Mehr dazu siehe: \ref ultraschallsensoren-projekt.

Ein weiteres Problem besteht im Aufbau des Labyrinths. Zwischen zwei Holzplatten entsteht stets eine kleine Rille, da die Platten nicht exakt gleich groß sind. 
An dieser Rille kann der Roboter, vor allem beim Rotieren, hängen bleiben.
