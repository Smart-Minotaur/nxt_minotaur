NXT {#installation_bbb_nxt}
===


* clone repository and compile only nxt_beagle


    git clone git://github.com/Smart-Minotaur/nxt_minotaur.git
    cd nxt_minotaur
    catkin_make -DCATKIN_WHITELIST_PACKAGES="nxt_beagle" -DUSE_MOUSE_SENSOR="TRUE"

