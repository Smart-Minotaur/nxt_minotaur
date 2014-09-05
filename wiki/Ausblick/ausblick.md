Ausblick {#ausblick}
===

\tableofcontents

\section Ausblick Ausblick

tuh duh

\section Fazit Fazit

Aufgrund verschiedener Faktoren konnten wir die Vorgaben leider nicht erfuellen. 

Zum einen bietet Lego keine stabile Platform fuer die Maussensoren. Durch das 
leichte zittern der Konstruktion kann es sein, dass sich der Abstand vom 
Maussensor zum Untergrund vergroessert, was wiederum zu falschen Messergebnisen 
fuehrt. Mit Hilfe eines Medianfilters kann dieses Problem zwar ein wenig behoben 
werden, aber um eine gute Odometrie zu bekommen reicht es immer noch nicht. 
Problem mit den Maussensoren: \ref Maussensoren2

Auch die Ultraschallsensoren von Lego sind nicht so gut wie erwuenscht. Es kann 
passieren, dass die Ultraschallsensoren falsche, also zu grosse Werte liefern. 
Wenn dies nur einmal vorkommen wuerde, waere es kein grosses Problem diese 
falschen Werte mit Hilfe eines Medianfilters raus zu filtern. Da die Sensoren 
aber fuer mehrere Messungen falsche Werte liefern und auch immer die gleichen, 
muesste man den Medianfilter groesser machen, was sich wieder negativ auswirkt. 
Durch diese Messfehler faengt der Roboter das "zittern" an und faehrt nicht mehr 
richtig. Wir konnten das Problem nicht beheben, da es unserer Meinung nach tief 
in der Firmware des Lego Bricks verwurzelt ist. 
Mehr dazu siehe: \ref Ultraschallsensoren


