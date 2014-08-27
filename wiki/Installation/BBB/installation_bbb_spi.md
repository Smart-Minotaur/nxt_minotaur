SPI {#installation_bbb_spi}
===

This page explains how to setup SPI0 and SPI1 on BBB with the device tree.

The script installs the overlay files for '''SPI0 (1 chip select)''' and '''SPI1 (2 chips selects)'''.

'''Only for angstrom linux'''
Warning! On the newest angstrom image there is no dtc in opkg available.
A simple solution is to create the compiled dtbo file on a desktop ubuntu machine
and copy it to the BBB.

* Clone the bbb_setup repository
* Install dtc (device tree compiler)


    sudo ./dtc.sh


* Run the addOverlayToDeviceTree script (compiles and installs SPI0 and SPI1 overlay file)


    sudo ./addOverlayToDeviceTree


* To enable SPI1 on startup (and disable HDMI which uses SPI1 pins)
* Mount boot partition


    mkdir /mnt/boot
    mount /dev/mmcblk0p1 /mnt/boot


* Edit uEnv.txt file on boot partition


    optargs=quiet drm.debug=7 capemgr.disable_partno=BB-BONELT-HDMI,BB-BONELT-HDMIN capemgr.enable_partno=MINOTAUR-SPI1


* Reboot bbb


    sudo reboot



