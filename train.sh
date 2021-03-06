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
#       --getopen-hand
#       
#   Training options:
#       --monitor
#       --synch-mode
#       --log-game
#       --restricted-vision
#       --single-learner 
#
#

export PATH=$PATH:/u/sanmit/rl/project/keepaway/tools

SARSA_INPUT=weights/SARSA/sarsaRand5k/sarsaWeightsX 

LSPI=weights/lspiWeightsX







### VIEWING

# 
./keepaway.py --single-learner --monitor --keeper-policy hand --getopen-input weights/lspiWeights1 --stop-after 500 --synch-mode

#./keepaway.py --monitor --keeper-policy hand --getopen-hand --synch-mode --stop-after 1000

#./keepaway.py --monitor --keeper-policy hand --getopen-input weights/LSPI/handMultiLspi5k/lspiWeightsX --stop-after 1000 --synch-mode


#./keepaway.py --monitor --keeper-policy hand --getopen-input weights/lspiWeights1 --stop-after 20 --load-same
#./keepaway.py --monitor --keeper-policy hand --getopen-input $LSPI --stop-after 20
#./keepaway.py --monitor --keeper-policy learned --keeper-input $SARSA_INPUT --getopen-input $LSPI --stop-after 5000 --synch-mode 

#### BASELINE


# Handcoded pass, random getopen (baseline)
#./keepaway.py --monitor --keeper-policy hand --stop-after 1000 --synch-mode 


### SINGLE LEARNER

# Handcoded pass, single learn getopen (5k, 10k)
#./keepaway.py --single-learner --monitor --keeper-policy hand --getopen-learn --getopen-output $LSPI --stop-after 5000 --synch-mode 


### NORMAL (ALL 3) LEARNING. Training on Handcoded versus SARSA

# Handcoded pass, learn getopen(5k, 10k)
./keepaway.py --keeper-policy hand --getopen-learn --single-learner --monitor --getopen-output $LSPI --stop-after 5000 --synch-mode 

# Sarsa pass, learn getopen (5k, 10k)
#./keepaway.py --monitor --keeper-policy learned --keeper-input $SARSA_INPUT --getopen-learn --getopen-output $LSPI --stop-after 5000 --synch-mode 


### IF TIME, COMMUNICATION VS NO COMMUNICATION VARIABLES

#./keepaway.py --monitor --keeper-policy hand --getopen-learn --getopen-output $LSPI --stop-after 5000 --synch-mode 
#./keepaway.py --monitor --keeper-policy learned --keeper-input $SARSA_INPUT --getopen-learn --getopen-output $LSPI --stop-after 5000 --synch-mode 




####################################

# Handcoded pass, continue getopen
#./keepaway.py --monitor --keeper-policy hand --getopen-learn --getopen-input $LSPI --getopen-output $LSPI --stop-after 2500 --synch-mode 


# Random pass, learn getopen
#./keepaway.py --monitor --keeper-policy random --getopen-learn --getopen-output $LSPI --stop-after 2500 --synch-mode 




# Fix pass, learn getopen
#./keepaway.py --monitor --keeper-policy learned --keeper-input $SARSA --getopen-learn --getopen-output $LSPI --stop-after 5000 --synch-mode

# Random pass, continue getopen
#./keepaway.py --monitor --keeper-policy random --getopen-learn --getopen-output $LSPI --getopen-input $LSPI --stop-after 2500 --synch-mode 

# Continue pass, random getopen
#./keepaway.py --monitor --keeper-policy learned --keeper-learn --keeper-input $SARSA --keeper-output $SARSA  --stop-after 5000 --synch-mode 

# Continue pass, hand getopen
#./keepaway.py --monitor --keeper-policy learned --keeper-learn --keeper-input $SARSA --keeper-output $SARSA --getopen-hand --stop-after 5000 --synch-mode 

# Learn pass, fix getopen
#./keepaway.py --monitor --keeper-policy learned --keeper-learn --keeper-output $SARSA --getopen-input $LSPI --stop-after 5000 --synch-mode 

# Fix pass, continue getopen
#./keepaway.py --monitor --keeper-policy learned --keeper-input $SARSA --getopen-learn --getopen-input $LSPI --getopen-output $LSPI --stop-after 15000 --synch-mode

# Learn pass, fix getopen
#./keepaway.py --monitor --keeper-policy learned --keeper-learn --keeper-input $SARSA --keeper-output $SARSA --getopen-input $LSPI --synch-mode 

# Learn both
#./keepaway.py --monitor --keeper-policy learned --keeper-learn --keeper-input $SARSA --keeper-output $SARSA --getopen-learn --getopen-input $LSPI --getopen-output $LSPI --synch-mode


# Watch learned policy
#./keepaway.py --monitor --keeper-policy learned --keeper-input $SARSA --getopen-input $LSPI


# Watch only LSPI policy
#./keepaway.py --monitor --keeper-policy hand --getopen-input $LSPI --stop-after 20
#./keepaway.py --monitor --keeper-policy hand --getopen-input weights/lspiWeights1 --stop-after 20 --load-same
#./keepaway.py --monitor --keeper-policy learned --keeper-input weights/SARSA/sarsaRand5k/sarsaWeights1 --getopen-input weights/lspiWeights1 --stop-after 20 --load-same

# Random pass, continue getopen
#./keepaway.py --monitor --keeper-policy random --getopen-learn --getopen-input $LSPI --getopen-output $LSPI --stop-after 5000 --synch-mode 

#./keepaway.py --monitor --keeper-policy hand --getopen-hand 


