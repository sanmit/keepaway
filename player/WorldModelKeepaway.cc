/*
Copyright (c) 2004 Gregory Kuhlmann, Peter Stone
University of Texas at Austin
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
this list of conditions and the following disclaimer in the documentation
and/or other materials provided with the distribution.

3. Neither the name of the University of Amsterdam nor the names of its
contributors may be used to endorse or promote products derived from this
software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <assert.h>

#include "WorldModel.h"

int WorldModel::getNumKeepers()
{
  return m_numKeepers;
}

void WorldModel::setNumKeepers( int iNum )
{
  m_numKeepers = iNum;
}

int WorldModel::getNumTakers()
{
  return m_numTakers;
}

void WorldModel::setNumTakers( int iNum )
{
  m_numTakers = iNum;
}

double WorldModel::congestion( VecPosition pos, bool considerMe ) 
{
  double congest = 0;
  if ( considerMe && pos != getAgentGlobalPosition() ) 
    congest = 1 / getAgentGlobalPosition().getDistanceTo( pos );

  VecPosition playerPos;

  int iIndex;
  for( ObjectT obj = iterateObjectStart( iIndex, OBJECT_SET_PLAYERS );
       obj != OBJECT_ILLEGAL;
       obj = iterateObjectNext ( iIndex, OBJECT_SET_PLAYERS ) ) {
    if ( ( playerPos = getGlobalPosition( obj ) ) != pos )
      if ( obj != getAgentObjectType() )
      /* Don't want to count a player in its own congestion measure */
      congest += 1/playerPos.getDistanceTo( pos );
  }

  return congest;
}

void WorldModel::resetEpisode()
{
  LogDraw.logText( "episode", VecPosition( 0, 0 ), "Reset", 
		      40, COLOR_WHITE );
  Ball.setTimeLastSeen( -1 );
  for ( int i = 0; i < MAX_TEAMMATES; i++ )
    Teammates[i].setTimeLastSeen( -1 );
  for ( int i = 0; i < MAX_OPPONENTS; i++ )
    Opponents[i].setTimeLastSeen( -1 );
  for ( int i = 0; i < MAX_TEAMMATES+MAX_OPPONENTS; i++ )
    UnknownPlayers[i].setTimeLastSeen( -1 );
  iNrUnknownPlayers = 0;
  for ( int i = 0; i < MAX_FLAGS; i++ )
    Flags[i].setTimeLastSeen( -1 );
  for ( int i = 0; i < MAX_LINES; i++ )
    Lines[i].setTimeLastSeen( -1 );

  setNewEpisode( true );
}

void WorldModel::setNewEpisode( bool bNewEp )
{
  m_newEpisode = bNewEp;
}

bool WorldModel::isNewEpisode()
{
  return m_newEpisode;
}

double WorldModel::keeperReward()
{
  double reward = getCurrentCycle() - getTimeLastAction();
  return reward;
}

void WorldModel::setLastAction( int iAction )
{
  m_lastAction = iAction;
  m_timeLastAction = 
    ( iAction == UnknownIntValue ) ? UnknownTime : getCurrentCycle();
}

int WorldModel::getLastAction()
{
  return m_lastAction;
}

int WorldModel::getTimeLastAction()
{
  return m_timeLastAction;
}

double WorldModel::keeperTeammateReward() {
    double reward = getCurrentCycle() - getTimeLastTeammateAction();
    return reward;
}

void WorldModel::setLastTeammateAction(int iAction){
    m_lastTeammateAction = iAction;
    m_timeLastTeammateAction = 
        ( iAction == UnknownIntValue ) ? UnknownTime : getCurrentCycle();
}

int WorldModel::getLastTeammateAction() {
    return m_lastTeammateAction;
}

int WorldModel::getTimeLastTeammateAction() {
    return m_timeLastTeammateAction;
}

// Populates the state vector for all future positions
int WorldModel::passerStateVars(double state[]){
  // HARDCODED: Number of states per position is 10. 
  int index = 0;
  const int numStates = 10; 
  for (double l=-7.0; l< 8; l+=3.5){
    for (double w=-7.0; w < 8; w+= 3.5){
         
        VecPosition targetLocation(l, w); 
        double actionState[10];      
        
        // Find values for this state. 
        passerStateVars(actionState, targetLocation);

        // Populate into whole state vector
        for (int i = 0; i < numStates; i++){
            state[index++] = actionState[i];
        }
    }
  }
  return index;
}

// Populates the state for the current agent evaluating position targetLocation
int WorldModel::passerStateVars(double state[], VecPosition targetLocation){

    int numK = getNumKeepers();
    int numT = getNumTakers();

    int myIndex = getAgentIndex();  // I believe this will find the player index. TODO: VERIFY THIS.

    // Keeper objects for teammates. Can't tell if this is just a type of if its the actual keeper... 
    ObjectT K[ numK ];
    for ( int i = 0; i < numK; i++ ){
        K[ i ] = SoccerTypes::getTeammateObjectFromIndex( i );
    }

    // Taker objects for opponents
    ObjectT T[ numT ];
    for ( int i = 0; i < numT; i++ )
        T[ i ] = SoccerTypes::getOpponentObjectFromIndex( i );

    // Need to sort the keeper and taker lists by who is closest to targetLocation. Also need to enforce that the current agent is in slot 0, or ignored entirely

    // Sort the keepers by their distance to the target location. Later, we will need to ignore the keeper that is the current agent
    double K1Prime_dist_to_K[ numK ];
    if ( !sortClosestTo( K, numK, targetLocation, K1Prime_dist_to_K ) )
        return 0;

    // Distance from takers to the target location
    double K1Prime_dist_to_T[ numT ];
    if ( !sortClosestTo( T, numT, targetLocation, K1Prime_dist_to_T ) )
        return 0;

  // Distance from each keeper to the closest opponent 
  double nearest_Opp_dist_K[ numK ];
  for ( int i = 1; i < numK; i++ ) {
    VecPosition pos = getGlobalPosition( K[ i ] );      // Location of each keeper
    for ( int j = 0; j < numT; j++ ) {
      double tmp = getGlobalPosition( T[ j ] ).getDistanceTo( pos );    // Distance to taker j
      if ( j == 0 || tmp < nearest_Opp_dist_K[ i ] ) {              // If this is the first iteration, or this taker is closer, set this to be the new distance
        nearest_Opp_dist_K[ i ] = tmp;
      }
    }
  }

  // Angle between the target location, each teammate, and each opponent
  double nearest_Opp_ang_K[ numK ];
  for ( int i = 1; i < numK; i++ ) {
    VecPosition pos = getGlobalPosition( K[ i ] );
    for ( int j = 0; j < numT; j++ ) {
      double tmp = targetLocation.getAngleBetweenPoints( pos, getGlobalPosition( T[ j ] ) );
      if ( j == 0 || tmp < nearest_Opp_ang_K[ i ] ) {
        nearest_Opp_ang_K[ i ] = tmp;
      }
    }
  }

    // Distance from K1 to K1'
    double K1_dist_to_K1Prime = getAgentGlobalPosition().getDistanceTo(targetLocation);

    // Minimum angle between K1, K1', and each taker 
    double K1_minAngleTo_K1Prime;
    for (int j = 0; j < numT; j++){
        double tmp = getAgentGlobalPosition().getAngleBetweenPoints(targetLocation, getGlobalPosition( T[j] ) );
        if (j == 0 || tmp < K1_minAngleTo_K1Prime){
            K1_minAngleTo_K1Prime = tmp;
        }
    }

    // Populate the state vector, ignoring the things above as necessary
    // TODO: VERIFY.
    int j = 0;
    for (int i = 0; i < numK; i++){
        if (SoccerTypes::getIndex(K[i]) != myIndex)
            state[j++] = K1Prime_dist_to_K[i]; 
    }
    for (int i = 0; i < numT; i++)
        state[j++] = K1Prime_dist_to_T[i];
    for (int i = 0; i < numK; i++){
        if (SoccerTypes::getIndex(K[i]) != myIndex)
            state[j++] = nearest_Opp_dist_K[i];
    }
    for (int i = 0; i < numK; i++){
        if (SoccerTypes::getIndex(K[i]) != myIndex)
            state[j++] = nearest_Opp_ang_K[i];
    }
    state[j++] = K1_dist_to_K1Prime;
    state[j++] = K1_minAngleTo_K1Prime;

    return j;

}



// SANMIT
int WorldModel::keeperStateVars( double state[] )
{
  ObjectT PB = getClosestInSetTo( OBJECT_SET_TEAMMATES, OBJECT_BALL );
  if ( !SoccerTypes::isTeammate( PB ) )
    return 0;

  // Center of field
  VecPosition C = getKeepawayRect().getPosCenter();

  // Distance of player closest to the ball from the center. So technically when this is called, this agent is the closest to the ball. 
  double WB_dist_to_C = getGlobalPosition( PB ).getDistanceTo( C );
  
  int numK = getNumKeepers();
  int numT = getNumTakers();

  // Keeper objects for teammates. Can't tell if this is just a type of if its the actual keeper... 
  ObjectT K[ numK ];
  for ( int i = 0; i < numK; i++ )
    K[ i ] = SoccerTypes::getTeammateObjectFromIndex( i );

  // Taker objects for opponents
  ObjectT T[ numT ];
  for ( int i = 0; i < numT; i++ )
    T[ i ] = SoccerTypes::getOpponentObjectFromIndex( i );

    // Note that these next two MUST BE DONE in order to sort the keeper and taker lists appropriately

  // Distances from other keepers to player with the ball. I'm guessing the 0th entry should be 0 since he has the ball.  
  double WB_dist_to_K[ numK ];
  if ( !sortClosestTo( K, numK, PB, WB_dist_to_K ) )
    return 0;
  
  // Distances from takers to the person with the ball. This is probably almost directly transferrable. 
  double WB_dist_to_T[ numT ];
  if ( !sortClosestTo( T, numT, PB, WB_dist_to_T ) )
    return 0;

    // END of MUST BE DONE

  // Distance from the keepers to the center
  double dist_to_C_K[ numK ];
  for ( int i = 1; i < numK; i++ ) {
    dist_to_C_K[ i ] = getGlobalPosition( K[ i ] ).getDistanceTo( C );
  }

  // Distance from the takers to the center
  double dist_to_C_T[ numT ];
  for ( int i = 0; i < numT; i++ ) {
    dist_to_C_T[ i ] = getGlobalPosition( T[ i ] ).getDistanceTo( C );
  }

  // Distance from each keeper to the closest opponent 
  double nearest_Opp_dist_K[ numK ];
  for ( int i = 1; i < numK; i++ ) {
    VecPosition pos = getGlobalPosition( K[ i ] );      // Location of each keeper
    for ( int j = 0; j < numT; j++ ) {
      double tmp = getGlobalPosition( T[ j ] ).getDistanceTo( pos );    // Distance to taker j
      if ( j == 0 || tmp < nearest_Opp_dist_K[ i ] ) {              // If this is the first iteration, or this taker is closer, set this to be the new distance
        nearest_Opp_dist_K[ i ] = tmp;
      }
    }
  }
  
  // Angle between this keeper, each teammate, and each opponent
  double nearest_Opp_ang_K[ numK ];
  VecPosition posPB = getGlobalPosition( PB );
  for ( int i = 1; i < numK; i++ ) {
    VecPosition pos = getGlobalPosition( K[ i ] );
    for ( int j = 0; j < numT; j++ ) {
      double tmp = posPB.getAngleBetweenPoints( pos, getGlobalPosition( T[ j ] ) );
      if ( j == 0 || tmp < nearest_Opp_ang_K[ i ] ) {
        nearest_Opp_ang_K[ i ] = tmp;
      }
    }
  }

  // Ahh. This is where the filter out the keeper at position 0. But how can i fool the system into thinking the current agent is at the target location... 
  int j = 0;
  state[ j++ ] = WB_dist_to_C;
  for ( int i = 1; i < numK; i++ )
    state[ j++ ] = WB_dist_to_K[ i ];
  for ( int i = 0; i < numT; i++ )
    state[ j++ ] = WB_dist_to_T[ i ];
  for ( int i = 1; i < numK; i++ )
    state[ j++ ] = dist_to_C_K[ i ];
  for ( int i = 0; i < numT; i++ )
    state[ j++ ] = dist_to_C_T[ i ];
  for ( int i = 1; i < numK; i++ )
    state[ j++ ] = nearest_Opp_dist_K[ i ];
  for ( int i = 1; i < numK; i++ )
    state[ j++ ] = nearest_Opp_ang_K[ i ];

  return j;
}

// Greg:
// This really doesn't belong here,
// because it is related to a specific 
// learner.  I don't know exactly
// where to put it, though because
// I want to keep the LinearSarsa
// class generic.

// Yaxin: changed from keeperTileWidths to keeperResolutions and keeperRanges,

int WorldModel::keeperStateRangesAndResolutions( double ranges[], 
						 double minValues[], 
						 double resolutions[], 
						 int numK, int numT )
{
  if ( numK < 3 ) {
    cerr << "keeperTileWidths: num keepers must be at least 3, found: " 
	 << numK << endl;
    return 0;
  }
  
  if ( numT < 2 ) {
    cerr << "keeperTileWidths: num takers must be at least 2, found: " 
	 << numT << endl;
    return 0;
  }

  int j = 0;

  double maxRange = hypot( 25, 25 );
			  
  ranges[ j ] = maxRange / 2.0;        // WB_dist_to_center           
  minValues[ j ] = 0;
  resolutions[ j++ ] = 2.0; 
  for ( int i = 1; i < numK; i++ ) {     // WB_dist_to_T          
    ranges[ j ] = maxRange;
    minValues[ j ] = 0;
    resolutions[ j++ ] = 2.0 + ( i - 1 ) / ( numK - 2 );
  }
  for ( int i = 0; i < numT; i++ ) {     // WB_dist_to_O
    ranges[ j ] = maxRange;
    minValues[ j ] = 0;
    resolutions[ j++ ] = 3.0 + ( i - 1 ) / ( numT - 1 );
  }
  for ( int i = 1; i < numK; i++ ) {     // dist_to_center_T    
    ranges[ j ] = maxRange / 2.0;
    minValues[ j ] = 0;
    resolutions[ j++ ] = 2.0 + ( i - 1 ) / ( numK - 2 );
  }
  for ( int i = 0; i < numT; i++ ) {     // dist_to_center_O  
    ranges[ j ] = maxRange / 2.0;
    minValues[ j ] = 0;
    resolutions[ j++ ] = 3.0;
  }
  for ( int i = 1; i < numK; i++ ) {     // nearest_Opp_dist_T 
    ranges[ j ] = maxRange / 2.0;
    minValues[ j ] = 0;
    resolutions[ j++ ] = 4.0;
  }
  for ( int i = 1; i < numK; i++ ) {     // nearest_Opp_ang_T  
    ranges[ j ] = 180;
    minValues[ j ] = 0;
    resolutions[ j++ ] = 10.0;
  }

  return j;
}


void WorldModel::setMoveSpeed( double speed )
{
  m_moveSpeed = speed;
}

double WorldModel::getMoveSpeed()
{
  return m_moveSpeed;
}

void WorldModel::setKeepawayRect( VecPosition pos1, VecPosition pos2 )
{
  m_keepawayRect.setRectanglePoints( pos1, pos2 );
}

Rect WorldModel::getKeepawayRect()
{
  return m_keepawayRect;
}

