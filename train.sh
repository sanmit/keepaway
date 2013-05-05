#!/bin/bash

#
#   LinearSarsaAgent options:
#       --keeper-learn
#       --keeper-policy
#       --keeper-input
#       --keeper-output
#
#   LSPIAgent options:
#       --getopen-learn
#       --getopen-input
#       --getopen-output
#       
#   Training options:
#       --monitor
#       --synch-mode
#       --log-game
#       --restricted-vision
#      
#
#

SARSA=weights/sarsaWeightsX
LSPI=weights/lspiWeightsX

# Handcoded pass, learn getopen
#./keepaway.py --monitor --keeper-policy hand --getopen-learn --getopen-output $LSPI --stop-after 5000 --synch-mode 

# Random pass, learn getopen
#./keepaway.py --monitor --keeper-policy random --getopen-learn --getopen-output $LSPI --stop-after 2500 --synch-mode 

# Learn pass, fix getopen
#./keepaway.py --monitor --keeper-policy learned --keeper-learn --keeper-output $SARSA --getopen-input $LSPI --stop-after 5000 --synch-mode 

# Fix pass, learn getopen
#./keepaway.py --monitor --keeper-policy learned --keeper-input $SARSA --getopen-learn --getopen-input $LSPI --getopen-output $LSPI --stop-after 15000 --synch-mode

# Learn pass, fix getopen
#./keepaway.py --monitor --keeper-policy learned --keeper-learn --keeper-input $SARSA --keeper-output $SARSA --getopen-input $LSPI --synch-mode 

# Learn both
#./keepaway.py --monitor --keeper-policy learned --keeper-learn --keeper-input $SARSA --keeper-output $SARSA --getopen-learn --getopen-input $LSPI --getopen-output $LSPI --synch-mode


# Watch learned policy
#./keepaway.py --monitor --keeper-policy learned --keeper-input $SARSA --getopen-input $LSPI


# Watch only LSPI policy
./keepaway.py --monitor --keeper-policy hand --getopen-input $LSPI --stop-after 15


# Random pass, continue getopen
#./keepaway.py --monitor --keeper-policy random --getopen-learn --getopen-input $LSPI --getopen-output $LSPI --stop-after 5000 --synch-mode 



#wait
#./keepaway.py --monitor --keeper-policy hand --getopen-input $LSPI --stop-after 5
