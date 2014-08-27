Wifi {#installation_bbb_wifi}
====


* copy the following to '''/etc/network/interfaces'''


    auto wlan0
    iface wlan0 inet static
    wpa-ssid "yourSSID"
    wpa-psk "yourWPA-PSK-Key"
    address 192.168.11.143
    netmask 255.255.255.0
    gateway 192.168.11.1

