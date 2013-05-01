#ifndef LSPI_AGENT
#define LSPI_AGENT

#include "SMDPAgent.h"
#include <Eigen/Dense> 
#include <vector>

/*
    TODO:

        - Need some way of loading, saving, and updating list of samples
        - Accessing a column:  .col(index)

        - Need to typecast actions in D to integers? Maybe not needed... 
*/

#define NUM_FEATURES 35     // 10 state variables + 25 binary action variables
#define NUM_STATE_FEAT 10
#define NUM_ACTIONS 25
#define MAX_CAPACITY 16777216 // 16MB   //2097152  // 2MB. Could probably raise this =) 

using namespace Eigen;
using namespace std;

class LSPIAgent:public SMDPAgent
{
  char weightsFile[256];   // File to store the weights in 
  bool bLearning;
  bool bSaveWeights;
  
  int epochNum;

  int lastAction;
  VectorXd lastBasisFeature;//(NUM_FEATURES);    // Stores the last state-action pair

//  double alpha;
  double gamma;     // discount factor (default 1)
//  double lambda;
  double epsilon;   // exploration rate (default 0)
  double theta;     // convergence threshold

    double Q[NUM_ACTIONS];   // This will store the values for each of the actions for the current state
    vector<double> D;   // This will store the samples of the form (state, action, reward, nextState). Thus each multiple of length  |S|+|A|+1+|S| is a sample. 

  // This is an |S|*|A| vector, where |S| is the number of state features
    VectorXd weights;   // Might want to consider fixing the size of these
    MatrixXd A;
    VectorXd b;
    
  int  selectAction();              // Select argmaxQ with probability 1-epsilon, random otherwise
  void computeQ( double state[]); // fullState is the stateFeatures. actions will be added inside the function?
  
  double computeQa(VectorXd features);  // Returns the value of taking a certain action in a certain state, which is parameterized by the features vector 
  
  int  argmaxQ();                   // Choose the action with the highest value
  void updateWeights();   // This will resolve for w
  void loadAbFromD();
  void updateA(VectorXd stateAction, VectorXd nextStateAction);
  void updateb(VectorXd stateAction, double reward);

  double weightDifference(VectorXd w1, VectorXd w2);
    
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
  bool saveWeights(char *filename);

  bool saveExperiences(char *filename);
  bool loadExperiences(char *filename);

  int getEpochNum() {return epochNum; }

  bool isLearning() {return bLearning; }
  bool learn();

} ;

#endif
