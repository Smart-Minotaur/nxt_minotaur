Zusatzinformation {#zusatz}
===

\section code-lite Installation von CodeLite

CodeLite ist eine cross-platform IDE. Dieser Abschnitt erklärt die Installation der neuesten CodeLite Version.

Für die Installation wird wxWidgets 3.0 benötigt. Dieses Paket kann nach dem Hinzufügen des Repositories via apt-get installiert werden.

Weitere Informationen: ​http://codelite.org/Developers/Linux

~~~
sudo apt-get install libgtk2.0-dev pkg-config build-essential git cmake libssh-dev libedit-dev

apt-key adv --fetch-keys http://repos.codelite.org/CodeLite.asc
sudo apt-add-repository 'deb http://repos.codelite.org/wx3.0.1/ubuntu/ precise universe'
sudo apt-get update
sudo apt-get install libwxbase3.0-0-unofficial libwxbase3.0-dev libwxgtk3.0-0-unofficial libwxgtk3.0-dev wx3.0-headers wx-common libwxbase3.0-dbg libwxgtk3.0-dbg wx3.0-i18n wx3.0-examples wx3.0-doc
~~~

\section bare-metal-toolchain Bare-Metal Toolchain für BBB

Eine bereits kompilierte bare-metal toolchain für ARM CPUs kann über folgende Befehle installiert werden.

Zuerst muss das PPA hinzugefügt werden:

~~~
https://launchpad.net/gcc-arm-embedded
~~~

Die Installation erfolgt üpber apt-get:

~~~
sudo add-apt-repository ppa:terry.guo/gcc-arm-embedded
sudo apt-get update
sudo apt-get install gcc-arm-none-eabi
~~~