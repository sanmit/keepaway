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

  gamma = 1.0;
  epsilon = 0; //0.01;
  theta = 1;

  epochNum = 0;
  lastAction = -1;
  lastBasisFeature = VectorXd::Constant(NUM_FEATURES, 0);


  D.reserve(MAX_CAPACITY);
  Q = VectorXd::Constant(NUM_ACTIONS, 0);

  for (int i = 0; i < NUM_ACTIONS; i++){
    A[i] = MatrixXd::Constant(NUM_FEATURES, NUM_FEATURES, 0);
    b[i] = VectorXd::Constant(NUM_FEATURES, 0);
  }
  
  weights = MatrixXd::Constant(NUM_FEATURES, NUM_ACTIONS, 0);    // Default to random policy 
  randomPolicy = true;  
  if ( strlen( loadWeightsFile ) > 0 ){
    loadWeights( loadWeightsFile );
    randomPolicy = false;                           // This is approximate, though I'm guessing once you save any weights, they won't be completely random... 
  }
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
    updateA(stateAction, lastBasisFeature, action); 

    // Update b
    updateb(stateAction, reward, action);    


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
    updateA(lastBasisFeature, lastBasisFeature, lastAction);

    // Update b
    updateb(lastBasisFeature, reward, lastAction);

  }
  lastAction = -1;      // Episode ended. 
}

int LSPIAgent::selectAction()
{
  int action;

  // Epsilon-greedy
  if ( bLearning && drand48() < epsilon ) {     /* explore */
    action = rand() % NUM_ACTIONS;
  }
  else{
    action = argmaxQ();
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
    for (int a = 0; a < NUM_ACTIONS; a++){
        for (int f = 0; f < NUM_FEATURES; f++){
            double el;
            inStream.read(reinterpret_cast<char*> (&el), sizeof el);
            weights(f, a) = el;
        }
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
    for (int a = 0; a < NUM_ACTIONS; a++){
        for (int f = 0; f < NUM_FEATURES; f++){
            double el = weights(f, a);
            outStream.write(reinterpret_cast<char*> (&el), sizeof el);
        }
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

bool LSPIAgent::saveExperiences(char *filename){
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
        Q(a) = features.dot(weights.col(a));
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
    for (int i = 0; i < NUM_ACTIONS; i++){
        A[i] = MatrixXd::Constant(NUM_FEATURES, NUM_FEATURES, 0);
        b[i] = VectorXd::Constant(NUM_FEATURES, 0);
    }

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
        updateA(stateAction, nextStateAction, action);

        // Update b
        updateb(stateAction, reward, action);

    }

}

// Might want to make these inline functions
void LSPIAgent::updateA(VectorXd stateAction, VectorXd nextStateAction, int actionIndex){
    A[actionIndex] += (stateAction * ( stateAction.transpose() - (gamma * nextStateAction.transpose())));
}

void LSPIAgent::updateb(VectorXd stateAction, double reward, int actionIndex){
    b[actionIndex] += (stateAction * reward);
}

// This calculates weights for a GIVEN A and b. Then it adjusts A and b according to the new policy. 
// If attaching loadAbFromD, this should not be called frequently!
void LSPIAgent::updateWeights()
{
    // Solve  Aw = b, accounting for the fact that A might be singular (not full rank)
    for (int i = 0; i < NUM_ACTIONS; i++){
        weights.col(i) = A[i].colPivHouseholderQr().solve(b[i]); 
    }
    // Reset A and b using the new policy derived from w, using the samples stored in D
    //loadAbFromD();    
}

bool LSPIAgent::learn() {

    // Keep track of previous weights
    MatrixXd lastWeights(NUM_FEATURES, NUM_ACTIONS);
    lastWeights << weights;

    // Update weights. This first call assumes that A and b have been updating from the step and end episode methods
    updateWeights();
    
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
    return (iteration < MAX_ITERATIONS);    // Converged, or just ran out of iterations
}

// Euclidean distance between the weight vectors
double LSPIAgent::weightDifference(MatrixXd w1, MatrixXd w2){
    MatrixXd w3 = w1 - w2;
    double dotProd = 0;
    for (int r = 0; r < w3.rows(); r++){
        for (int c = 0; c < w3.cols(); c++){
            dotProd += (w3(r, c) * w3(r, c));    
        }
    }
    return sqrt(dotProd);
}


// TODO:
void LSPIAgent::setParams(int iCutoffEpisodes, int iStopLearningEpisodes)
{
  /* set learning parameters */
}
