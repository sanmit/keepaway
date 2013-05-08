#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <time.h>
#include "LSPIAgent.h"
#include "LoggerDraw.h"
#include <Eigen/Dense>
#include <iostream>
#include <fstream>
#include <math.h>

using namespace std;
using namespace Eigen;

// If all is well, there should be no mention of anything keepaway- or soccer-
// related in this file. 

extern LoggerDraw LogDraw;

LSPIAgent::LSPIAgent( int numFeatures, int numActions, bool bLearn,
				    char *loadWeightsFile, char *saveWeightsFile ):
  SMDPAgent( numFeatures, numActions )
{
  // This will determine if we are training/learning, or just exploiting/moving randomly  
  bLearning = bLearn;

  // Save only the weights!
  if ( bLearning && strlen( saveWeightsFile ) > 0 ) {
    strcpy( weightsFile, saveWeightsFile );
    bSaveWeights = true;
  }
  else {
    bSaveWeights = false;
  }


  epochNum = 0;
  lastAction = -1;
  lastBasisFeature = VectorXd::Constant(NUM_FEATURES, 0);

  D.reserve(MAX_CAPACITY);
  Q = VectorXd::Constant(NUM_ACTIONS, 0);

  A = MatrixXd::Constant(NUM_FEATURES, NUM_FEATURES, 0);
  b = VectorXd::Constant(NUM_FEATURES, 0);

  weights = VectorXd::Constant(NUM_FEATURES, 0);    // Default to random policy 
  randomPolicy = true;  
  if ( strlen( loadWeightsFile ) > 0 ){
    loadWeights( loadWeightsFile );
    randomPolicy = false;                           // This is approximate, though I'm guessing once you save any weights, they won't be completely random... 
  }
  
  gamma = 1.0;
  if (bLearning) // && !randomPolicy)
    epsilon = 0; //0.25;  // Since LSPI is off policy, we want as many different examples as possible        //0.01;
  else
    epsilon = 0;    // No exploration if we aren't learning or we are behaving randomly already
  theta = 0; //0.000001;


}


// Here, we set up the last action and lastBasisFeature vector so that subsequent steps can use them to update D, and later, the policy.  
// COMPLETE.
int LSPIAgent::startEpisode( double state[] )
{
    epochNum++;
    computeQ(state);  
    lastAction = selectAction();
    
    // Populate lastBasisFeature vector
    int startIndex = lastAction * NUM_FEATURES; 
    // first populate the states
    for (int i = 0; i < NUM_FEATURES; i++){
         lastBasisFeature(i) = state[startIndex];
         startIndex++;
    }
          
    return lastAction;
}

// Takes in an array of the state values of taking each action. 
// Our main goal here is to update D, return the new action, and (maybe) update A
int LSPIAgent::step( double reward, double state[] )
{

    // Add (s, a, r, s') to D, where s' is the basis feature of the current state
    // This adds s
    VectorXd stateAction(NUM_FEATURES);
    for (int i = 0; i < NUM_FEATURES; i++){
        D.push_back(lastBasisFeature(i));
        stateAction(i) = lastBasisFeature(i);
    }

    // This adds a
    D.push_back(lastAction);
    int action = lastAction;

    // This adds r
    D.push_back(reward);

    // Compute s' a', which is the basis feature of the current state. 
    computeQ(state);    // This updates our Q matrix
    lastAction = selectAction();    // this determines which action we're taking
    
    // Now use the action to determine what the next state feature is -- remember our state features are action dependent. 
    // This is the s' that we actually transition to, and will be added in the *next example*
    int startIndex = lastAction * NUM_FEATURES; 
    for (int i = 0; i < NUM_FEATURES; i++){
         lastBasisFeature(i) = state[startIndex];
         startIndex++;
    }

    // Push all possible s'
    for (int i = 0; i < NUM_FEATURES * NUM_ACTIONS; i++){
         D.push_back(state[i]);
    }

    // Update A
    //updateA(stateAction, lastBasisFeature); 

    // Update b
    //updateb(stateAction, reward);    


    return lastAction;
}

void LSPIAgent::endEpisode( double reward )
{
  if ( bLearning && lastAction != -1 ) { /* otherwise we never ran on this episode */
    // Technically we need to update A, but we can't do that without the next state. 
    // Going to assume nextState is same as previous state. Anyways at this point, it has reached the "death spiral" or "point of no return" 
    
    // This adds s
    for (int i = 0; i < NUM_FEATURES; i++){
        D.push_back(lastBasisFeature(i));
    }
    // This adds a
    D.push_back(lastAction);
    // This adds r
    D.push_back(reward);
    // This adds s'
    for (int a = 0; a < NUM_ACTIONS; a++){
        for (int i = 0; i < NUM_FEATURES; i++){
            D.push_back(lastBasisFeature(i));
        }   
    }
    
    // Update A
    //updateA(lastBasisFeature, lastBasisFeature);

    // Update b
    //updateb(lastBasisFeature, reward);

  }
  lastAction = -1;      // Episode ended. 
}

// This is hardcoded for 25 grid points action space
int LSPIAgent::selectAction()
{
  int action;

  action = argmaxQ();
  
  // Epsilon-greedy
  if ( bLearning && drand48() < epsilon ) {     /* explore */
      // Find neighboring actions to argmax instead of just exploring completely randomly (that should be done when weight vector is all 0s) 
      int local[8] = {-6, -5, -4, -1, 1, 4, 5, 6};
      vector<int> destinations;
      for (int i = 0; i < 8; i++){
          int newDest = action + local[i];
          if (newDest >= 0 && newDest < NUM_ACTIONS)
              destinations.push_back(newDest);
      }
      int actionIndex = rand() % destinations.size();
      action = destinations[actionIndex];
      //action = rand() % NUM_ACTIONS;
  }

  return action;
}
// Load w
bool LSPIAgent::loadWeights( char *filename )
{
    cout << "Loading weights from " << filename << " ...";
    // Load the weights from the file to the double array
    ifstream inStream(filename, ios::in | ios::binary);
    if (!inStream)
        return false;
    for (int f = 0; f < NUM_FEATURES; f++){
        double el;
        inStream.read(reinterpret_cast<char*> (&el), sizeof el);
        weights(f) = el;
    }
    inStream.close();
    // Copy the weights to weight vector
    cout << "done" << endl;
    return true;
}

// Save w
bool LSPIAgent::saveWeights()
{
    cout << "Saving weights to " << weightsFile << " ...";
    // Write the weight array to file
    ofstream outStream(weightsFile, ios::out | ios::binary | ios::trunc);
    if (!outStream)
        return false;
    for (int f = 0; f < NUM_FEATURES; f++){
        double el = weights(f);
        outStream.write(reinterpret_cast<char*> (&el), sizeof el);
    }
    outStream.close();
    cout << "done" << endl;
    return true;
}

bool LSPIAgent::loadExperiences(char *filename){
    cout << "Loading experiences from " << filename << " ...";
    ifstream inStream(filename, ios::in | ios::binary);
    if (!inStream)
        return false;
    // Read in the size
    int size;
    inStream.read(reinterpret_cast<char*>( &size), sizeof size);
    // Read in the elements
    for (int i = 0; i < size; i++){
        double el;
        inStream.read(reinterpret_cast<char*> (&el), sizeof el);
        D.push_back(el);
    }
    inStream.close();
    cout << " done" << endl;
    return true;
}

bool LSPIAgent::saveExperiences(){
    char filename[256];
    sprintf(filename, "%sexperiences", weightsFile);
    cout << "Saving experiences to " << filename << " ...";
    ofstream outStream(filename, ios::out | ios::binary | ios::trunc);
    if (!outStream)
        return false;
    int size = D.size();
    // Write the size
    outStream.write(reinterpret_cast<char*>(&size), sizeof size);
    // Write the elements
    for (int i = 0; i < size; i++){
        double el = D[i];
        outStream.write(reinterpret_cast<char*>( &el), sizeof el);
    }
    outStream.close();
    cout << "done" << endl;
    return true;
}


// Use the weights matrix and state features to compute Q
void LSPIAgent::computeQ( double state[])
{
    // state should contain the features of the states for all 25 points on the grid
   
    for (int a = 0; a < NUM_ACTIONS; a++){
    
        RowVectorXd features(NUM_FEATURES);
            
        // Extract each state (for a state-action pair)
        for (int s = 0; s < NUM_FEATURES; s++){
            features(s) = state[(a * NUM_FEATURES) + s];
        }
        
        // Multiply by weight vector (i.e. call computeQa) and store in Q[a]
        Q(a) = features.dot(weights);
    }
    
}


// Returns index (action) of largest entry in Q vector, breaking ties randomly 
int LSPIAgent::argmaxQ()
{
  int bestAction = 0;
  double bestValue = Q(bestAction);
  int numTies = 0;
  for ( int a = 1; a < NUM_ACTIONS; a++ ) {
    double value = Q(a);
    if ( value > bestValue ) {
      bestValue = value;
      bestAction = a;
    }
    else if ( value == bestValue ) {
      numTies++;
      if ( rand() % ( numTies + 1 ) == 0 ) {
	bestValue = value;
	bestAction = a;
      }
    }
  }

  return bestAction;
}

// Assumes D is populated, and a policy exists (i.e. a weight vector w)
// This should only be called when (loading from the file or) when the policy changes. Right now, the policy changes when the weights change. 
void LSPIAgent::loadAbFromD() {

    // Reset A, b for each action
    A = MatrixXd::Constant(NUM_FEATURES, NUM_FEATURES, 0);
    b = VectorXd::Constant(NUM_FEATURES, 0);

    for (unsigned int i = 0; i < D.size();){
        
        // Recover s
        VectorXd stateAction(NUM_FEATURES);
        for (int j = 0; j < NUM_FEATURES; j++){
            stateAction(j) = D[i];
            i++;
        }

        // Recover a
        int action = floor(D[i++] + 0.5);

        // Recover r
        double reward = D[i++];

        // Recover s'
        double sPrime[NUM_FEATURES * NUM_ACTIONS];
        for (int a = 0; a < NUM_ACTIONS; a++){
            for (int f = 0; f < NUM_FEATURES; f++){
                sPrime[(a * NUM_FEATURES) + f] = D[i++];
            }
        }
        
        // Find out what a' should be under current policy
        computeQ(sPrime);
        int nextAction = selectAction();
        
        // Use a' to extract next state feature 
        VectorXd nextStateAction(NUM_FEATURES);
        int startIndex = nextAction * NUM_FEATURES; 
        for (int i = 0; i < NUM_FEATURES; i++){
             nextStateAction(i) = sPrime[startIndex];
             startIndex++;
        }

        // Update A
        updateA(stateAction, nextStateAction);

        // Update b
        updateb(stateAction, reward);

    }

}

// Might want to make these inline functions
void LSPIAgent::updateA(VectorXd stateAction, VectorXd nextStateAction){
    A += (stateAction * ( stateAction.transpose() - (gamma * nextStateAction.transpose())));
}

void LSPIAgent::updateb(VectorXd stateAction, double reward){
    b += (stateAction * reward);
}

// This calculates weights for a GIVEN A and b. Then it adjusts A and b according to the new policy. 
// If attaching loadAbFromD, this should not be called frequently!
void LSPIAgent::updateWeights()
{
    // Solve  Aw = b, accounting for the fact that A might be singular (not full rank)
    weights = A.colPivHouseholderQr().solve(b); 
    
    // Reset A and b using the new policy derived from w, using the samples stored in D
    //loadAbFromD();    
}

bool LSPIAgent::learn() {

    // Keep track of previous weights
    VectorXd lastWeights(NUM_FEATURES);
    lastWeights << weights;

    //cout << "Original weights\n" << lastWeights << endl;

    loadAbFromD();

    //cout << "A\n" << A << endl;
    //cout << "b\n" << b << endl;

    // Update weights. This first call assumes that A and b have been updating from the step and end episode methods
    updateWeights();
    
    //cout << "New weights\n" << weights << endl;

    double weightDiff = weightDifference(lastWeights, weights);
    cout << "Initial weight difference: " << weightDiff << endl;

    const int MAX_ITERATIONS = 100;
    int iteration = 0;
    while (iteration < MAX_ITERATIONS && weightDiff > theta){
        cout << "Learning iteration " << ++iteration << " Weight Difference: " << weightDiff << endl;
        lastWeights << weights;     // Keep track of current weights
        loadAbFromD();              // Update A and b with out new weights
        updateWeights();            // Calculate new weights
        weightDiff = weightDifference(lastWeights, weights);
    }
    cout << "Final Weight difference: " << weightDifference(lastWeights, weights) << endl;
    
    cout << "Learned weights:\n" << weights << endl;
    
    return (iteration < MAX_ITERATIONS);    // Converged, or just ran out of iterations
}

// Euclidean distance between the weight vectors
double LSPIAgent::weightDifference(VectorXd w1, VectorXd w2){
    VectorXd w3 = w1 - w2;
    double dotProd = w3.dot(w3);
    return sqrt(dotProd);
}


// TODO:
void LSPIAgent::setParams(int iCutoffEpisodes, int iStopLearningEpisodes)
{
  /* set learning parameters */
}
