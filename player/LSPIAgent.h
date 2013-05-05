#ifndef LSPI_AGENT
#define LSPI_AGENT

#include "SMDPAgent.h"
#include <Eigen/Dense> 
#include <vector>

/*
    TODO:

        - Need some way of loading, saving, and updating list of samples
        - Accessing a column:  .col(index)

*/

#define NUM_FEATURES 10     // 10 state variables 
//#define NUM_STATE_FEAT 10     // vestige definition
#define NUM_ACTIONS 25
#define MAX_CAPACITY 16777216 // 16MB   //2097152  // 2MB. Could probably raise this =) 

using namespace Eigen;
using namespace std;

class LSPIAgent:public SMDPAgent
{
  char weightsFile[256];   // File to store the weights in 
  bool bLearning;
  bool bSaveWeights;
  bool randomPolicy;
  int epochNum;

  int lastAction;
  VectorXd lastBasisFeature;//(NUM_FEATURES);    // Stores the last state-action pair

  double gamma;     // discount factor (default 1)
  double epsilon;   // exploration rate (default 0)
  double theta;     // convergence threshold

  VectorXd Q;   // This will store the values for each of the actions for the current state
    vector<double> D;   // This will store the samples of the form (state, action, reward, nextState). Thus each multiple of length  |S|+1+1+|S*A| is a sample. 

    // This is an |S| x |A| matrix, where each column corresponds to the features for an action 
    MatrixXd weights;   // Might want to consider fixing the size of these
    MatrixXd A[NUM_ACTIONS];    // One A and b for each action weight
    VectorXd b[NUM_ACTIONS];
    
  int  selectAction();              // Select argmaxQ with probability 1-epsilon, random otherwise
  void computeQ( double state[]); // fullState is the stateFeatures. actions will be added inside the function?
  
  
  int  argmaxQ();                   // Choose the action with the highest value
  void updateWeights();   // This will resolve for w
  void loadAbFromD();
  void updateA(VectorXd stateAction, VectorXd nextStateAction, int actionIndex);
  void updateb(VectorXd stateAction, double reward, int actionIndex);

  double weightDifference(MatrixXd w1, MatrixXd w2);
    
public:
  LSPIAgent                  ( int    numFeatures,
				      int    numActions,
				      bool   bLearn,
				      char   *loadWeightsFile,
				      char   *saveWeightsFile );

  // SMDP Sarsa implementation
  int  startEpisode( double state[] );
  int  step( double reward, double state[] );       // 
  void endEpisode( double reward );
  void setParams(int iCutoffEpisodes, int iStopLearningEpisodes);

  // Accessors
  int getLastAction() {return lastAction; }

  // Functions
  bool loadWeights( char *filename );
  bool saveWeights();

  bool saveExperiences(char *filename);
  bool loadExperiences(char *filename);

  int getEpochNum() {return epochNum; }

  bool isLearning() {return bLearning; }
  bool learn();
  bool isRandomPolicy() { return randomPolicy; }

} ;

#endif
