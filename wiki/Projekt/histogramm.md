Histogramm Ansatz {#histogramm}
===

\tableofcontents

\section beschreibung-histogramm Beschreibung

Der Histogramm Ansatz wurde in dem Package __minotaur_map__ realisiert.
In diesem Ansatz wurde die Karte des Labyrinths mithilfe eines 
Histogrammfeldes aufgezeichnet. Wie in jeder Anwendung läuft auf dem 
Beagle Bone die __RobotControl Node__. Außerdem läuft auf dem Beagle 
Bone die ROS-Node __Movebase__. Diese realisiert die grundlegende 
Kommunikation des __ROS-Navigation Stack__. Dadurch kann Movebase eine 
globale Zielkoordinate für den Roboter übergeben werden und Movebase 
navigiert den Roboter an diese Koordinate. Während seiner Navigation 
wird über Ultraschallsensoren die Umgebungskarte angefertigt.

Aufgrund sehr schlechter Odometrie und der daraus resultierenden 
unbenutzbaren Karte, gibt es hierfür keine fertig implementierten 
Navigationsalgorithmen. Die Histogrammkarte kann jedoch mit dem Tool 
__map_monitor__ benutzt und ausprobiert werden. Mehr findet sich dazu 
auf der Seite \ref gui-tools.

\section umsetzung-histogramm Umsetzung

Wie bereits erwähnt gibt es in diesem Ansatz keine fertiggestellten 
Navigationsalgorithmen. Jedoch sind bereits __Templates__ für einen Bug 0 
und [Pledge-Algorithmus](http://de.wikipedia.org/wiki/L%C3%B6sungsalgorithmen_f%C3%BCr_Irrg%C3%A4rten#Pledge-Algorithmus)
vorhanden. Der grundlegende Gedanke war, dass der Roboter sich mithilfe 
der __Karte__ und des __ROS Navigation Stack__ durch das Labyrinth navigiert.

\subsection kartografierung-histogramm Kartografierung

In einem eigenen Thread wird die Histogrammkarte __parallel__ zur 
Navigation aufgezeichnet. Dabei wird jedesmal, sobald eine neue 
Messung eintrifft, anhand der aktuellen Position ein Eintrag in der 
Histogrammkarte gemacht. Über ein internes ID Mapping wird 
unterschieden, zu welchem Sensor die eingehende Messung gehört. Über 
die aktuelle Position des Roboters und mithilfe der Ausrichtung des Sensors 
wird ermittelt, in welcher Zelle der Histogrammkarte der Wert erhöht 
werden muss. Zusätzlich wird den Messungen noch ein Fächer 
hinzugefügt, sodass der Wert aller Zellen erhöht wird, die auf der 
Kreisbahn des entsprechenden Kreisabschnitts liegen.

\subsection navigation-histogramm Navigation

In einem seperaten Thread findet die Navigation statt. Anhand der
aufgezeichneten Karte legt ein Navigationsalgorithmus die __nächste 
Zielkoordinate__ fest, an die sich der Roboter bewegen soll. Diese 
Koordinate wird an den ROS Navigation Stack übergeben, sodass dieser 
den Roboter dorthin navigiert. Da die Histogrammkarte nicht in den ROS 
Navigation Stack integriert ist, werden Hindernisse in der 
Histogrammkarte von dem ROS Navigation Stack nicht berücksichtigt.
Daher muss der eigene Navigationsalgorithmus darauf achtgeben nur 
Zielkoordinaten zu übergeben, die nicht durch ein Hindernis blockiert 
werden.

\section probleme-histogramm Probleme

Wie schon oben erwähnt wurde dieser ganze Ansatz jedoch __verworfen__, da 
die Odometriedaten des Roboters so schlecht waren, dass der Roboter 
bereits nach kurzer Fahrt komplett die Orientierung verloren hat. Zur 
Verbesserung der Odometrie sollten eigentlich die Maussensoren genutzt 
werden. Zu diesem Teil des Projektes finden sich mehr Informationen 
auf der Seite \ref maussensoren.

