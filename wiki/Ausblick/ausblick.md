Ausblick {#ausblick}
===

\tableofcontents

\section fazit-ausblick Fazit

Aufgrund verschiedener Faktoren konnte die Zielstellung des Projekts 
nicht erfüllt werden.

Zum einen bietet Lego keine stabile Platform für die Maussensoren. 
Durch das instabile Verhalten der Konstruktion kann es sein, dass sich 
der Abstand vom Maussensor zum Untergrund vergrößert oder 
verkleinert, was wiederum zu falschen Messergebnissen führt. Mit Hilfe 
eines Medianfilters kann dieses Problem zwar ein wenig behoben werden, 
aber um eine gute Odometriedaten zu bekommen reicht es immer noch 
nicht aus. Außerdem ist der Untergrund des Labyrinths so uneben, dass 
die Maussensoren dazu neigen auf und ab zu springen. Mehr Details 
hierzu kann unter \ref Maussensoren2 gefunden werden.

Auch die Ultraschallsensoren von Lego sind nicht so gut wie erwünscht. Es kann 
passieren, dass die Ultraschallsensoren falsche, also zu große oder zu 
kleine Werte liefern. Wenn dies nur einmal vorkommen würde, wäre es 
kein großes Problem diese falschen Werte mit Hilfe eines Medianfilters 
raus zu filtern. Da die Sensoren aber für mehrere Messungen falsche 
Werte liefern und auch immer die gleichen, müsste man den Medianfilter 
größer machen, was sich wiederrum negativ auf die Reaktionszeit des 
Roboters auswirkt. Durch diese Messfehler fängt der Roboter an seine 
Position falsch zu korrigieren. Die Ursache für diese Fehler sind 
unbekannt und liegen wahrscheinlich tief in der Firmware des Lego 
Bricks verwurzelt ist. Mehr dazu siehe: \ref ultraschallsensoren-projekt.

\section ausblick-ausblick Ausblick

tuh duh
