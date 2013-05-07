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

#include "KeepawayPlayer.h"
#include "Parse.h"
#include "SayMsgEncoder.h"
#include "LSPIAgent.h"
#include <cstring>
#include "stdlib.h"
#include "SayMsgFactory.h"
#include "WorldModelSayMsgFactory.h"
#include "SayMsgDecoder.h"

extern LoggerDraw LogDraw;

KeepawayPlayer::KeepawayPlayer( SMDPAgent* sa, LSPIAgent* sa2, ActHandler* act, WorldModel *wm, 
				ServerSettings *ss, PlayerSettings *ps,
				char* strTeamName, int iNumKeepers, int iNumTakers,
				double dVersion, int iReconnect, int iStopAfter )

{
  char str[MAX_MSG];
  
  SA            = sa;
  SA2           = sa2;
  ACT           = act;
  WM            = wm;
  SS            = ss;
  PS            = ps;
  bContLoop     = true;
  WM->setTeamName( strTeamName );
  WM->setNumKeepers( iNumKeepers );
  WM->setNumTakers( iNumTakers ); 
  WM->setNewEpisode( false );
  WM->setLastAction( UnknownIntValue );
  m_timeLastSay = -5;
  m_timeStartEpisode = -5;
  stopAfter = iStopAfter;
  // create initialisation string
  if( iReconnect != -1 )
    sprintf( str, "(reconnect %s %d)", strTeamName, iReconnect );
  else
    sprintf( str, "(init %s (version %f))", strTeamName, dVersion );
  ACT->sendMessage( str );
  
}

/*! This is the main loop of the agent. This method calls the update methods
    of the world model after it is indicated that new information has arrived.
    After this, the correct main loop of the player type is called, which
    puts the best soccer command in the queue of the ActHandler. */
void KeepawayPlayer::mainLoop( )
{
  Timing timer;

    // TEMP TODO
    //WM->keeperXDestination[0] = 0;
    //WM->keeperYDestination[0] = 0;

  // wait for new information from the server
  // cannot say bContLoop=WM->wait... since bContLoop can be changed elsewhere
  if(  WM->waitForNewInformation() == false )
    bContLoop =  false;

    // Communication setup
    SayMsgFactory *msgfactory = new WorldModelSayMsgFactory(WM);
    SayMsgDecoder *decoder = new SayMsgDecoder(msgfactory);


  // *** BEGIN KEEPAWAY LOOP ***
  bool finished = false;
  while( bContLoop )                                 // as long as server alive
  {
    Log.logWithTime( 3, "  start update_all" );
    Log.setHeader( WM->getCurrentCycle() );
    LogDraw.setTime( WM->getCurrentCycle() );

    // UPDATE WORLD MODEL
    if( WM->updateAll( ) == true )
    {
      timer.restartTime();
      SoccerCommand soc;


    // Update Communication Information
    char serverSayMsg[MAX_SAY_MSG];
    WM->getStrPlayerMsg(serverSayMsg);
    int playernum = WM->getMsgSender(); 
    //cout << "Player " << playernum << " said " << serverSayMsg << endl;
    //cout << "HEARD: " << WM->strLastHearMessage << endl;
    //cout << "Agent index: " << WM->getAgentIndex() << " Number " << WM->getPlayerNumber() << endl;
    bool success = decoder->decodeStr(serverSayMsg, playernum);
    
    //cout << "Decoding message by " << playernum - 1 << "(" << WM->getAgentIndex() << "): " << serverSayMsg << endl;
    
    if (success){
        SayMsgDecoder::DecodedMsgIterator iter = decoder->getMsgIterator();
        for( ; iter != decoder->getMsgIteratorEnd(); iter++){
            SayMsgTypes *curunit = *iter;
            curunit->process();
        }
    }
    else {
        cout << "DECODING FAILED." << endl;
    }
    //cout << WM->getAgentIndex() << " moved to (" << targetL << "," << targetW << ")" << endl; 

    //cout << WM->getAgentIndex() << ": Player 0 headed to (" << WM->keeperXDestination[0] << "," << WM->keeperYDestination[0] << ")" << endl;
    //cout << WM->getAgentIndex() << ": Player 1 headed to (" << WM->keeperXDestination[1] << "," << WM->keeperYDestination[1] << ")" << endl;
    //cout << WM->getAgentIndex() << ": Player 2 headed to (" << WM->keeperXDestination[2] << "," << WM->keeperYDestination[2] << ")" << endl;


      // DETERMINE ACTION/COMMAND TO TAKE
      if ( WM->getSide() == SIDE_LEFT )
	soc = keeper();
      else
	soc = taker();


    // WHAT I WANT: Determine whether to communicate based on modulus of the cycle number, that way each keeper takes turns

    // Communicate intended destination
    // This is done in interpretKeeperTeammateAction since there is easier access to the required parameters (and keeperSupport for handcoded agents)

      // COMMUNICATE THE COMMAND TAKEN. Sanmit: I might change this to mimic the communication done in HFO... or is this already like that?
/*      if( shallISaySomething() == true )           // shall I communicate
        {
          m_timeLastSay = WM->getCurrentTime();
          char strMsg[MAX_SAY_MSG];
	      makeSayMessage( soc, strMsg );
          if( strlen( strMsg ) != 0 )
            Log.log( 600, "send communication string: %s", strMsg );
          // This is what sends the communication?
         //WM->setCommunicationString( strMsg );
        }
      Log.logWithTime( 3, "  determined action; waiting for new info" );
      // directly after see message, will not get better info, so send commands
*/      
      // Dunno what's going on here...
      if( WM->getTimeLastSeeMessage() == WM->getCurrentTime() ||
          (SS->getSynchMode() == true && WM->getRecvThink() == true ))
      {
        Log.logWithTime( 3, "  send messages directly" );
        ACT->sendCommands( );
        Log.logWithTime( 3, "  sent messages directly" );
        if( SS->getSynchMode() == true  )
        {
          WM->processRecvThink( false );
          ACT->sendMessageDirect( "(done)" );       // Also looks like this sends something...
        }
      }
    }  
    else
      Log.logWithTime( 3, "  HOLE no action determined; waiting for new info");
    // END UPDATE WORLD MODEL ***

    // I'm guessing this is some kind of logging... 
    int iIndex;
    double dConfThr = PS->getPlayerConfThr();
    char buffer[128];
    for( ObjectT o = WM->iterateObjectStart( iIndex, OBJECT_SET_TEAMMATES, dConfThr);
  	 o != OBJECT_ILLEGAL;
  	 o = WM->iterateObjectNext ( iIndex, OBJECT_SET_TEAMMATES, dConfThr ) ) {
      LogDraw.logCircle( "Players", WM->getGlobalPosition( o ), 1.6, 80,
			 false,
			 COLOR_ORANGE, WM->getConfidence( o ) );
      sprintf( buffer, "%d", SoccerTypes::getIndex( o ) + 1 );
      LogDraw.logText( "Players", WM->getGlobalPosition( o ),
		       buffer,
		       80, COLOR_ORANGE );
    }
    // Me
    ObjectT o = WM->getAgentObjectType();
    LogDraw.logCircle( "Players", WM->getGlobalPosition( o ), 1.6, 81,
		       false,
		       COLOR_PURPLE, WM->getConfidence( o ) );
    sprintf( buffer, "%d", SoccerTypes::getIndex( o ) + 1 );
    LogDraw.logText( "Players", WM->getGlobalPosition( o ),
		     buffer,
		     81, COLOR_PURPLE );
    for( ObjectT o = WM->iterateObjectStart( iIndex, OBJECT_SET_OPPONENTS, dConfThr);
  	 o != OBJECT_ILLEGAL;
  	 o = WM->iterateObjectNext ( iIndex, OBJECT_SET_OPPONENTS, dConfThr ) ) {
      LogDraw.logCircle( "Players", WM->getGlobalPosition( o ), 1.6, 80,
			 false,
			 COLOR_PINK, WM->getConfidence( o ) );
      sprintf( buffer, "%d", SoccerTypes::getIndex( o ) + 1 );
      LogDraw.logText( "Players", WM->getGlobalPosition( o ),
		       buffer,
		       80, COLOR_PINK );
    }

    Log.logWithTime( 604, "time for action: %f", timer.getElapsedTime()*1000 );
           
    // wait for new information from the server cannot say
    // bContLoop=WM->wait... since bContLoop can be changed elsewhere
    if(  WM->waitForNewInformation() == false )             // This basically blocks until a new sense or see comes from the server. If the timer times out, then it is over. 
        bContLoop =  false;

    // We train/test in segments of 5K
    if (stopAfter != -1 && SA->getEpochNum() > stopAfter && (!SA2 || (SA2 && SA2->getEpochNum() > stopAfter))){
        cout << "*** Player " << WM->getPlayerNumber() << " finished seeing " << stopAfter << " episodes ***" << endl;
        system("killserver");
    }
        // Can't do this because not all players finish all their epochs at the same time =/
        //bContLoop = false;
  }
  // *** END MAIN LOOP ***


// SANMIT TODO: SAVE WEIGHTS OF PLAYER HERE.
/* 
    char sarsaName[256];
    sprintf(sarsaName, "sarsaWeights%d", WM->getPlayerNumber());

    char lspiName[256];
    sprintf(lspiName, "lspiWeights%d", WM->getPlayerNumber());
*/  

    if (SA && SA->isLearning()){
        SA->saveWeights();
    }
    
    if (SA2 && SA2->isLearning()){
        cout << "LSPI Agent learning weights..." << endl;
        SA2->learn();
        SA2->saveWeights();
    }

  // shutdow, print hole and number of players seen statistics
  printf("Shutting down player %d\n", WM->getPlayerNumber() );
  printf("   Number of holes: %d (%f)\n", WM->iNrHoles,
                         ((double)WM->iNrHoles/WM->getCurrentCycle())*100 );
  printf("   Teammates seen: %d (%f)\n", WM->iNrTeammatesSeen,
                         ((double)WM->iNrTeammatesSeen/WM->getCurrentCycle()));
  printf("   Opponents seen: %d (%f)\n", WM->iNrOpponentsSeen,
                         ((double)WM->iNrOpponentsSeen/WM->getCurrentCycle()));

}


/********************** SAY **************************************************/

/*!This method determines whether a player should say something.
   \return bool indicating whether the agent should say a message */
bool KeepawayPlayer::shallISaySomething()
{
  bool        bReturn;

  bReturn  = ((WM->getCurrentTime() - m_timeLastSay) >= SS->getHearDecay());
  bReturn  &= (WM->getCurrentCycle() > 0 );

  return bReturn;
}

void KeepawayPlayer::makeSayMessage( SoccerCommand soc, char * strMsg )
{
  VecPosition posBall = WM->getGlobalPosition( OBJECT_BALL );
  VecPosition velBall = WM->getGlobalVelocity( OBJECT_BALL );
  int iDiff = 0;
  SayMsgEncoder myencoder;

  VecPosition posBallPred;
  WM->predictBallInfoAfterCommand( soc, &posBallPred );
  VecPosition posAgentPred = WM->predictAgentPosAfterCommand( soc );

  // If we have good information about the ball
  if( ( WM->getTimeChangeInformation(OBJECT_BALL) == WM->getCurrentTime() &&
	WM->getRelativeDistance( OBJECT_BALL ) < 20.0 &&
	WM->getTimeLastSeen( OBJECT_BALL ) == WM->getCurrentTime() )
      ||
      (
       WM->getRelativeDistance( OBJECT_BALL ) < SS->getVisibleDistance() &&
       WM->getTimeLastSeen( OBJECT_BALL ) == WM->getCurrentTime()  
       )
      ||
      (
       WM->getRelativeDistance( OBJECT_BALL ) < SS->getMaximalKickDist() &&
       posBallPred.getDistanceTo( posAgentPred ) > SS->getMaximalKickDist() 
       )
      ) 
  {
    // If we are kicking the ball 
    if( WM->getRelativeDistance( OBJECT_BALL ) < SS->getMaximalKickDist() )
    {
      // if kick and a pass
      if( soc.commandType == CMD_KICK )
      {
	WM->predictBallInfoAfterCommand( soc, &posBall, &velBall );
	VecPosition posAgent = WM->predictAgentPos( 1, 0 );
	if( posBall.getDistanceTo( posAgent ) > SS->getMaximalKickDist() + 0.2 )
	  iDiff = 1;
      }
      
      if( iDiff == 0 )
      {
	posBall = WM->getGlobalPosition( OBJECT_BALL );
	velBall.setVecPosition( 0, 0 );
      }
    }

    LogDraw.logCircle( "ball sending", posBall,
		       1.1, 90, false, COLOR_BLUE );
    
    myencoder.add( new BallInfo( posBall.getX(), posBall.getY(),
				 velBall.getX(), velBall.getY(), 1 - iDiff ) ); 
  }

  // Find closest opponent that was seen this cycle
  int numT = WM->getNumTakers();
  ObjectT T[ numT ];
  int numSeen = 0;
  for ( int i = 0; i < numT; i++ ) {
    T[ numSeen ] = SoccerTypes::getOpponentObjectFromIndex( i );
    if ( WM->getRelativeDistance( T[ numSeen ] ) < SS->getVisibleDistance() &&
	 WM->getTimeLastSeen( T[ numSeen ] ) == WM->getCurrentTime() )
      numSeen++;  // store this opponent if we just saw him
  }
  WM->sortClosestTo( T, numSeen, WM->getAgentObjectType() ); 

  if ( numSeen > 0 ) { // add closest
    VecPosition posOpp = WM->getGlobalPosition( T[ 0 ] );
    myencoder.add( new OppPos( SoccerTypes::getIndex( T[ 0 ] ) + 1,
			       posOpp.getX(), posOpp.getY(), 1 ) );
  }

  if ( myencoder.getSize() <= 7 &&  // if there is room
       WM->getConfidence( WM->getAgentObjectType() ) > PS->getPlayerHighConfThr() ) {
    myencoder.add( new OurPos( posAgentPred.getX(), posAgentPred.getY() ) );
  }
  //cout << strMsg << endl;  
  strcpy( strMsg, myencoder.getEncodedStr().c_str() );
  //cout << strMsg << endl << endl;  
  myencoder.clear();
}

/********************** Keepaway ***************************************/

SoccerCommand KeepawayPlayer::keeper()
{

  //  cout << "Keeper " << WM->getAgentIndex() << " calling keeper()" << endl;


    //cout << WM->strLastHearMessage << endl; 

  SoccerCommand soc;

  if ( WM->isNewEpisode() ) {
    SA->endEpisode( WM->keeperReward() );
    if (SA2)
        SA2->endEpisode(WM->keeperTeammateReward());
    WM->setNewEpisode( false );
    WM->setLastAction( UnknownIntValue );
    m_timeStartEpisode = WM->getCurrentTime();
  }

  LogDraw.logCircle( "ball belief", WM->getBallPos(),
		     1.1, 11, false, COLOR_RED, 
		     WM->getConfidence( OBJECT_BALL ) );
  char buffer[128];
  sprintf( buffer, "%.2f", WM->getConfidence( OBJECT_BALL ) );
  LogDraw.logText( "ball belief", WM->getBallPos(),
		   buffer,
		   11, COLOR_RED );	  

  // If we don't know where the ball is, search for it.
  if ( WM->getConfidence( OBJECT_BALL ) <
       PS->getBallConfThr() ) {
    ACT->putCommandInQueue( soc = searchBall() );
    ACT->putCommandInQueue( alignNeckWithBody() );
    LogDraw.logText( "state", VecPosition( 25, 25 ),
		     "lost ball",
		     1, COLOR_WHITE );
    return soc;
  }

  // If the ball is kickable,
  // call main action selection routine.
  if ( WM->isBallKickable() ) {
    Log.log( 100, "Ball is kickable for me." );
    return keeperWithBall();
  }

  // Get fastest to ball
  int iTmp;
  ObjectT fastest = WM->getFastestInSetTo( OBJECT_SET_TEAMMATES, 
					   OBJECT_BALL, &iTmp );
  
  // If fastest, intercept the ball.
  if ( fastest == WM->getAgentObjectType() ) {
    Log.log( 100, "I am fastest to ball; can get there in %d cycles", iTmp );
    LogDraw.logText( "state", VecPosition( 25, 25 ),
		     "fastest",
		     1, COLOR_WHITE );

    ObjectT lookObject = chooseLookObject( 0.98 );

    char buffer[128];
    LogDraw.logText( "lookObject", VecPosition( 25, -25 ), 
		     SoccerTypes::getObjectStr( buffer, lookObject ), 100, COLOR_WHITE );

    ACT->putCommandInQueue( soc = intercept( false ) );
    //ACT->putCommandInQueue( turnNeckToObject( lookObject, soc ) );
    //ACT->putCommandInQueue( turnNeckToPoint( SS->getKeepawayRect().getPosCenter(), soc ) );
    ACT->putCommandInQueue( turnNeckToObject( OBJECT_BALL, soc ) );
    return soc;
  }

  // Not fastest, get open
  Log.log( 100, "I am not fastest to ball" );
  LogDraw.logText( "state", VecPosition( 25, 25 ),
		   "support",
		   1, COLOR_WHITE );
  return keeperSupport( fastest );
}    

SoccerCommand KeepawayPlayer::keeperWithBall()
{
    
  //cout << "Keeper " << WM->getAgentIndex() << " calling keeperwithball()" << endl;
  double state[ MAX_STATE_VARS ];
  int action;

  if ( WM->keeperStateVars( state ) > 0 ) { // if we can calculate state vars
    // Call startEpisode() on the first SMDP step
    if ( WM->getTimeLastAction() == UnknownTime ) {
      action = SA->startEpisode( state );
    }
    else if ( WM->getTimeLastAction() == WM->getCurrentCycle() - 1 && 
	      WM->getLastAction() > 0 ) {   // if we were in the middle of a pass last cycle
      action = WM->getLastAction();         // then we follow through with it
    }
    // Call step() on all but first SMDP step
    else {
      action = SA->step( WM->keeperReward(), state );
    }
    WM->setLastAction( action );
  }
  else { // if we don't have enough info to calculate state vars
    action = 0;  // hold ball 
    LogDraw.logText( "state", VecPosition( 35, 25 ),
		     "clueless",
		     1, COLOR_RED );
  }

  return interpretKeeperAction( action );
}

// Interprets the action for teammate motion to a grid point. This should ensure consistency with the policy search function's ordering
SoccerCommand KeepawayPlayer::interpretTeammateAction(int action) {

    double dDistDashBack = 2;

    SoccerCommand soc;
    
    int counter = 0;

    float targetL = 0;
    float targetW = 0;
    for (double l=-7.0; l< 8; l+=3.5){
        for (double w=-7.0; w < 8; w+= 3.5){
            if (counter == action){
                VecPosition destination(l, w);
                soc = moveToPos(destination, PS->getPlayerWhenToTurnAngle(), dDistDashBack);
                targetL = l;
                targetW = w;
            }
            counter++;
        }
    }
    //cout << "Total actions: " << counter << endl;

    ACT->putCommandInQueue(soc);
    //ACT->putCommandInQueue( turnNeckToPoint( WM->getKeepawayRect().getPosCenter(), soc ) ); // Look center
    ACT->putCommandInQueue( turnNeckToObject( OBJECT_BALL, soc ) );     // Look for ball

    // If it's your cycle turn, communicate your action
    // Consider making it every cycle?
    //if (((WM->getCurrentCycle()) % (WM->getNumKeepers())) == WM->getAgentIndex()){ 
        
        // HARDCODED. Note: Since only 2 players will be trying to get open at any time (and hence executing this call), it is ok to say something every cycle (the players are allowed to say and hear a message).
        char strMsg[MAX_SAY_MSG];
        SayMsgEncoder myencoder;
        myencoder.add(new PassToCoord(targetL, targetW));
        strcpy(strMsg, myencoder.getEncodedStr().c_str());
        myencoder.clear();
        WM->setCommunicationString(strMsg);
    //}

    // Read the message back
/*    char serverSayMsg[256];
    strcpy(serverSayMsg, WM->strLastHearMessage);
    SayMsgFactory *msgfactory = new MySayMsgFactory();
    SayMsgDecoder *decoder = new SayMsgDecoder(msgfactory);
    bool success = decoder->decodeStr(serverSayMsg, playernum);
*/
   
    return soc;

}


SoccerCommand KeepawayPlayer::interpretKeeperAction( int action )
{
  SoccerCommand soc;
  
  if ( action == 0 ) { // interpret HOLD action
    ACT->putCommandInQueue( soc = holdBall() );
    LogDraw.logText( "state", VecPosition( 25, 25 ),
		     "holding",
		     1, COLOR_WHITE );
  }
  else { // interpret PASS action
    int numK = WM->getNumKeepers();
    ObjectT K[ numK ];
    for ( int i = 0; i < numK; i++ )
      K[ i ] = SoccerTypes::getTeammateObjectFromIndex( i );
    WM->sortClosestTo( K, numK, WM->getAgentObjectType() );
    VecPosition tmPos = WM->getGlobalPosition( K[ action ] );
    // Normal Passing
    ACT->putCommandInQueue( soc = directPass( tmPos, PASS_NORMAL ) );
    // Or Fast Passing
    //ACT->putCommandInQueue( soc = directPass( tmPos, PASS_FAST ) );
    char buffer[128];
    LogDraw.logText( "state", VecPosition( 25, 25 ),
		     SoccerTypes::getObjectStr( buffer, K[ action ] ),
		     1, COLOR_WHITE );
  }
  
  return soc;
}

// Checks if the player 
bool KeepawayPlayer::reachedPosition(int action, double epsilon){
    int counter = 0;
    double distance;

    for (double l=-7.0; l< 8; l+=3.5){
        for (double w=-7.0; w < 8; w+= 3.5){
            if (counter == action){
                VecPosition destination(l, w);
                distance = destination.getDistanceTo(WM->getAgentGlobalPosition());
            }
            counter++;
        }
    }

    return (distance < epsilon);

}

SoccerCommand KeepawayPlayer::keeperSupport( ObjectT fastest )
{
    
  //cout << "Keeper " << WM->getAgentIndex() << " calling keeperSupport()" << endl;
    // If we are using the learned getopen policy
if (SA2){  // SA2    
  
  // Need to calculate state variables for each point on the grid, and learn to map those state-points to values using policy search  
  // Then, need to plug in state variables into function approximator (neural net) to determine value. 
  // Choose the action that has the highest value
  // This action taken will receive feedback(reward).   

  int action;
  //int numK = WM->getNumKeepers();
  //int numT = WM->getNumTakers();
//      double fieldWidth = SS->getKeepawayWidth() / 2;
//      double fieldLength = SS->getKeepawayLength() / 2;    
// length is x axis


  //const int NUM_PASSER_STATE_VARS = (3 * (numK - 1)) + numT + 2;  
  //const int NUM_PASSER_FEATS = 35;
  //double state[NUM_PASSER_FEATS]; 
    
    double state[NUM_ACTIONS * NUM_FEATURES];

  if ( WM->passerStateVars( state ) > 0 ) {


/*
      // Random action if we are just starting or if we have reach the random point
    if (WM->getTimeLastAction() == UnknownTime || reachedPosition(WM->getLastAction())){
        action = rand() % 25;    
        WM->setLastAction(action);
    }
    // Otherwise keep the same action
    else {
        action = WM->getLastAction();
    }
*/
    // Need to change reward and WM->get/set Last action maybe? since these are on separate MDPs?
      // Call startEpisode() on the first SMDP step
    if (SA2->getLastAction() == -1){
        action = SA2->startEpisode(state);
        WM->setLastTeammateAction(action);
    }
    // If we were moving to a position, keep going to it. Remember these are SMDP actions.
    // Also we only do this while the agent is learning... 
    else if (!reachedPosition(SA2->getLastAction()) && SA2->isRandomPolicy() ){  // && SA2->isLearning()
        action = SA2->getLastAction();
        // Might comment this reward out...
        WM->setLastTeammateAction(action);
    }
    // Otherwise step and figure out a new action
    else {
        action = SA2->step(WM->keeperTeammateReward(), state);
        WM->setLastTeammateAction(action);
    }
  }
  else { // if we don't have enough info to calculate state vars
   // action = WM->getLastTeammateAction();
    //if (action == -1)
        return moveToPos(WM->getAgentGlobalPosition(), PS->getPlayerWhenToTurnAngle(), 2.0); // stay where you are
    LogDraw.logText( "state", VecPosition( 35, 25 ),
		     "clueless",
		     1, COLOR_RED );
  }
/*    if (WM->getAgentIndex() == 0)
        action = 22;
    else if (WM->getAgentIndex() == 1)
        action = 6;
    else
        action = 14;
*/
    return interpretTeammateAction( action );
}

// Otherwise handcoded getopen
  SoccerCommand soc;

  int iCycles = WM->predictNrCyclesToObject( fastest, OBJECT_BALL );
  VecPosition posPassFrom = 
    WM->predictPosAfterNrCycles( OBJECT_BALL, iCycles );
  LogDraw.logCircle( "BallPredict", posPassFrom, 1, 70, true, COLOR_BROWN );
  soc = getOpenForPassFromInRectangle( WM->getKeepawayRect(), posPassFrom );

  ObjectT lookObject = chooseLookObject( 0.97 );

  char buffer[128];
  LogDraw.logText( "lookObject", VecPosition( 25, -25 ), 
		   SoccerTypes::getObjectStr( buffer, lookObject ), 100, COLOR_WHITE );

  ACT->putCommandInQueue( soc );
  //ACT->putCommandInQueue( turnNeckToObject( lookObject, soc ) );
  ACT->putCommandInQueue( turnNeckToPoint( WM->getKeepawayRect().getPosCenter(), soc ) );

  // Communicate hand-coded position?
    char strMsg[MAX_SAY_MSG];
    SayMsgEncoder myencoder;
    VecPosition target = leastCongestedPointForPassInRectangle(WM->getKeepawayRect(), posPassFrom);
    myencoder.add(new PassToCoord(target.getX(), target.getY()));
    strcpy(strMsg, myencoder.getEncodedStr().c_str());
    myencoder.clear();
    WM->setCommunicationString(strMsg);


  return soc;
}

// It looks like this decides where the keepaway player should look. If his confidence on the location
// of the ball is greater than some threshold, then he will look in the direction of a player whose 
// location he is most unsure of. Otherwise, he will look for the ball. 
ObjectT KeepawayPlayer::chooseLookObject( double ballThr )
{
  if ( WM->getConfidence( OBJECT_BALL ) < ballThr )
    return OBJECT_BALL;

  ObjectT objLeast = OBJECT_ILLEGAL;
  double confLeast = 1.1;
  for ( int i = 0; i < WM->getNumKeepers(); i++ ) {
    ObjectT obj = SoccerTypes::getTeammateObjectFromIndex( i );
    if ( obj != WM->getAgentObjectType() ) {
      double conf = WM->getConfidence( obj );
      if ( conf < confLeast ) {
	confLeast = conf;
	objLeast = obj;
      }
    }
  }

  return objLeast;
}

SoccerCommand KeepawayPlayer::taker()
{
  SoccerCommand soc;

  LogDraw.logCircle( "ball pos", WM->getBallPos(),
		     1.1, 11, false, COLOR_RED, WM->getConfidence( OBJECT_BALL ) );

  //cout << "TAKER POSITION: " << WM->getAgentGlobalPosition() << endl;
  // If we don't know where the ball is, search for it.
  if ( WM->getConfidence( OBJECT_BALL ) <
       PS->getBallConfThr() ) {
    ACT->putCommandInQueue( soc = searchBall() );
    ACT->putCommandInQueue( alignNeckWithBody() );
    return soc;
  }

  // Maintain possession if you have the ball.
  if ( WM->isBallKickable() &&
       WM->getClosestInSetTo( OBJECT_SET_TEAMMATES, OBJECT_BALL) ==
       WM->getAgentObjectType() ) {
    ACT->putCommandInQueue( soc = holdBall( 0.3 ) );
    return soc;
  }  

  // If not first or second closest, then mark open opponent
  int numT = WM->getNumTakers();
  ObjectT T[ numT ];
  for ( int i = 0; i < numT; i++ )
    T[ i ] = SoccerTypes::getTeammateObjectFromIndex( i );
  WM->sortClosestTo( T, numT, OBJECT_BALL );
  if ( numT > 2 && T[ 0 ] != WM->getAgentObjectType() &&
       T[ 1 ] != WM->getAgentObjectType() ) {
    ObjectT withBall = WM->getFastestInSetTo( OBJECT_SET_OPPONENTS, 
					      OBJECT_BALL );
    ACT->putCommandInQueue( soc = markMostOpenOpponent( withBall ) );
    ACT->putCommandInQueue( turnNeckToObject( OBJECT_BALL, soc ) );
    return soc;
  }

  // If teammate has it, don't mess with it
  double dDist;
  ObjectT closest = WM->getClosestInSetTo( OBJECT_SET_PLAYERS, 
					   OBJECT_BALL, &dDist );
  if ( SoccerTypes::isTeammate( closest ) &&
       closest != WM->getAgentObjectType() &&
       dDist < SS->getMaximalKickDist() ) {
    ACT->putCommandInQueue( soc = turnBodyToObject( OBJECT_BALL ) );
    ACT->putCommandInQueue( alignNeckWithBody() );
    return soc;
  }
  

  // Otherwise try to intercept the ball
  ACT->putCommandInQueue( soc = intercept( false ) );
  ACT->putCommandInQueue( turnNeckToObject( OBJECT_BALL, soc ) );
  return soc;
}
