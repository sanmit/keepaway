#ifndef LSPI_AGENT
#define LSPI_AGENT

#include "SMDPAgent.h"
#include <Eigen/Dense> 
#include <vector>

/*
    TODO:

        - Need some way of loading, saving, and updating list of samples
        - Accessing a column:  .col(index)

        - Refactor lastBasisFeature to be a VectorXd

*/

#define NUM_FEATURES 35     // 10 state variables + 25 binary action variables
#define NUM_STATE_FEAT 10
#define NUM_ACTIONS 25
#define MAX_CAPACITY 2097152  // 2MB. Could probably raise this =) 

class LSPIAgent:public SMDPAgent
{
  char weightsFile[256];   // File to store the weights in 
  bool bLearning;
  bool bSaveWeights;
  
  int epochNum;

  int lastAction;
  double lastBasisFeature[NUM_FEATURES];    // Stores the last state-action pair

  double alpha;
  double gamma;     // Used. Default to one. 
  double lambda;
  double epsilon;
  
    double Q[NUM_ACTIONS];   // This will store the values for each of the actions for the current state
    vector<double> D;   // This will store the samples of the form (state, action, reward, nextState). Thus each multiple of length  |S|+|A|+1+|S| is a sample. 

  // This is an |S|*|A| vector, where |S| is the number of state features
    VectorXd weights;   // Might want to consider fixing the size of these
    MatrixXd A;
    VectorXd b;
    

  // Load / Save weights from/to disk
  bool loadWeights( char *filename );
  bool saveWeights( char *filename );

  int  selectAction();              // Select argmaxQ with probability 1-epsilon, random otherwise
  void computeQ( double[] state); // fullState is the stateFeatures. actions will be added inside the function?
  
  double computeQa(VectorXd features);  // Returns the value of taking a certain action in a certain state, which is parameterized by the features vector 
  
  int  argmaxQ();                   // Choose the action with the highest value
  void updateWeights();   // This will resolve for w
  void loadAbFromD();
  void updateA(VectorXd stateAction, VectorXd nextStateAction);
  void updateb(VectorXd stateAction, double reward);
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
} ;

#endif
