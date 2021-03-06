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

#ifndef KEEPAWAY_PLAYER
#define KEEPAWAY_PLAYER

#include "BasicPlayer.h"
#include "SMDPAgent.h"
#include "LSPIAgent.h"

/*! This class is a superclass from BasicPlayer and contains a more
    sophisticated decision procedure to determine the next action. */
class KeepawayPlayer:public BasicPlayer
{
 protected:
  bool          bContLoop;               /*!< is server is alive             */

  Time          m_timeLastSay;           /*!< last time communicated         */
  Time          m_timeStartEpisode;
  SMDPAgent     *SA;        // Holds passer policy and taker policy.
  LSPIAgent     *SA2;       // Holds GetOpen policy. For takers, this is NULL.
  int           stopAfter;
  // methods associated with saying (defined in KeepawayPlayer.cc)
  bool          shallISaySomething        (                                  );
  void          makeSayMessage            ( SoccerCommand  soc,
					    char *         str               );

 public:
  KeepawayPlayer                          ( SMDPAgent      *sa,
                                            LSPIAgent      *sa2,
					                        ActHandler     *a,
                                            WorldModel     *wm,
                                            ServerSettings *ss,
                                            PlayerSettings *cs,
                                            char           *strTeamName,
					                        int            iNumKeepers,
					                        int            iNumTakers,
                                            double         dVersion,
                                            int            iReconnect = -1,
                                            int            iStopAfter = -1);

  void          mainLoop                  (                                  );

    // facility methods
  bool reachedPosition(int action, double epsilon = 0.5);

  // behaviors
  SoccerCommand keeper();
  SoccerCommand keeperWithBall();
  SoccerCommand keeperSupport( ObjectT fastest );
  SoccerCommand interpretKeeperAction( int action );
  SoccerCommand interpretTeammateAction(int action);
  ObjectT chooseLookObject( double ballThr );

  SoccerCommand taker();

};

#endif

