# nxtInterface

## Preparation

### install Python

* Python 2.x is required
* not compatible with 1.x and 3.x

### install PyUSB

* download PyUSB from http://sourceforge.net/projects/pyusb/
* extract and search for __setup.py__
* execute __sudo python setup.py install__

### install nxt-python

* download nxt-python from http://code.google.com/p/nxt-python/downloads/list
* extract and search for __setup.py__
* execute __sudo python setup.py install__
* execute __usermod -a -G lego <username>__ (replace <username> with your username)
* create file __/etc/udev/rules.d/70-lego.rules__
* add the line __SUBSYSTEM=="usb", ATTRS{idVendor}=="0694", GROUP="lego", MODE="0660"__ to that file
* restart yout computer