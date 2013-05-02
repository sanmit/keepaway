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

# Random pass, learn getopen
#./keepaway.py --monitor --keeper-policy random --getopen-learn --getopen-output weights/lspiWeightsX --synch-mode 

# Learn pass, fix getopen
#./keepaway.py --monitor --keeper-policy learned --keeper-learn --keeper-output weights/sarsaWeightsX --getopen-input weights/lspiWeightsX --synch-mode 

# Fix pass, learn getopen
#./keepaway.py --monitor --keeper-policy learned --keeper-input weights/sarsaWeightsX --getopen-learn --getopen-input weights/lspiWeightsX --getopen-output weights/lspiWeightsX --synch-mode

# Learn pass, fix getopen
#./keepaway.py --monitor --keeper-policy learned --keeper-learn --keeper-input weights/sarsaWeightsX --keeper-output weights/sarsaWeightsX --getopen-input weights/lspiWeightsX --synch-mode 

# Learn both
#./keepaway.py --monitor --keeper-policy learned --keeper-learn --keeper-input weights/sarsaWeightsX --keeper-output weights/sarsaWeightsX --getopen-learn --getopen-input weights/lspiWeightsX --getopen-output weights/lspiWeightsX --synch-mode


# Watch learned policy
./keepaway.py --monitor --keeper-policy learned --keeper-input weights/sarsaWeightsX --getopen-input weights/lspiWeightsX 