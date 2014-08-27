Ubuntu 12.04 {#installation_bbb_ubuntu}
============

* connect micro sd to your pc


    wget -O ubuntu-precise-12.04.3-armhf-3.8.13-bone30.img.xz http://s3.armhf.com/debian/precise/bone/ubuntu-precise-12.04.3-armhf-3.8.13-bone30.img.xz
    sudo su
    xz -cd ubuntu-precise-12.04.3-armhf-3.8.13-bone30.img.xz > /dev/sdX


* /dev/sdX is the device file of your microsd (also possible /dev/mmcXXX)
* user: ubuntu
* password: ubuntu
