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

//  alpha = 0.125;
  gamma = 1.0;
//  lambda = 0;
  epsilon = 0; //0.01;
  theta = 0.000001;

  epochNum = 0;
  lastAction = -1;
  lastBasisFeature = VectorXd::Constant(NUM_FEATURES, 0);


  D.reserve(MAX_CAPACITY);
  A = MatrixXd::Constant(NUM_FEATURES, NUM_FEATURES, 0);
  b = VectorXd::Constant(NUM_FEATURES, 0);

  if ( strlen( loadWeightsFile ) > 0 ){
    loadWeights( loadWeightsFile );
  }
  else{
    // Random policy (weights 0)
    weights = VectorXd::Constant(NUM_FEATURES, 0);  
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
    int startIndex = lastAction * NUM_STATE_FEAT; 
    // first populate the states
    for (int i = 0; i < NUM_STATE_FEAT; i++){
         lastBasisFeature(i) = state[startIndex];
         startIndex++;
    }
    // and then the actions
    int counter = 0;
    for (int i = NUM_STATE_FEAT; i < NUM_FEATURES; i++){
        if (counter == lastAction)
            lastBasisFeature(i) = 1;
        else
            lastBasisFeature(i) = 0;
        counter++;
    }        

    return lastAction;
}

// Takes in an array of the state values of taking each action. 
// Our main goal here is to update D, return the new action, and (maybe) update A
int LSPIAgent::step( double reward, double state[] )
{

    // Add (s, a, r, s') to D, where s' is the basis feature of the current state
    // This adds s, a
    VectorXd stateAction(NUM_FEATURES);
    for (int i = 0; i < NUM_FEATURES; i++){
        D.push_back(lastBasisFeature(i));
        stateAction(i) = lastBasisFeature(i);
    }
    // This adds r
    D.push_back(reward);


    // Compute s' a', which is the basis feature of the current state. 
    computeQ(state);    // This updates our Q matrix
    lastAction = selectAction();    // this determines which action we're taking
    

    // now use the action to determine what the next state feature is
    int startIndex = lastAction * NUM_STATE_FEAT; 
    // first populate the states
    for (int i = 0; i < NUM_STATE_FEAT; i++){
         lastBasisFeature(i) = state[startIndex];
         startIndex++;
    }
    // and then the actions
    int counter = 0;
    for (int i = NUM_STATE_FEAT; i < NUM_FEATURES; i++){
        if (counter == lastAction)
            lastBasisFeature(i) = 1;
        else
            lastBasisFeature(i) = 0;
        counter++;
    }        

    // This adds s' 
    for (int i = 0; i < NUM_STATE_FEAT; i++){
        D.push_back(lastBasisFeature(i));
    }

    // Update A
    updateA(stateAction, lastBasisFeature); 

    // Update b
    updateb(stateAction, reward);    


    return lastAction;
}

void LSPIAgent::endEpisode( double reward )
{
  if ( bLearning && lastAction != -1 ) { /* otherwise we never ran on this episode */
/*    char buffer[128];
    sprintf( buffer, "reward: %.2f", reward ); 
    LogDraw.logText( "reward", VecPosition( 25, 30 ),
		     buffer,
		     1, COLOR_NAVY );
*/    
    // TODO: technically we need to update A, but we can't do that without the next state. 
    // Going to assume nextState is same as previous state. 
     // This adds s, a
    
    for (int i = 0; i < NUM_FEATURES; i++){
        D.push_back(lastBasisFeature(i));
    }
    // This adds r
    D.push_back(reward);
    // This adds s'
    for (int i = 0; i < NUM_STATE_FEAT; i++){
        D.push_back(lastBasisFeature(i));
    }

    // Update A
    updateA(lastBasisFeature, lastBasisFeature);

    // Update b
    updateb(lastBasisFeature, reward);

  }
  // SANMIT EDIT: Changed 200 to 5 in order to see that saving is working... need to find a better way to do this... 
//  if ( bLearning && bSaveWeights && rand() % 200 == 0 ) {
//    saveWeights( weightsFile );
//  }
  lastAction = -1;
}

int LSPIAgent::selectAction()
{
  int action;

  // Epsilon-greedy
  if ( bLearning && drand48() < epsilon ) {     /* explore */
    action = rand() % getNumActions();
  }
  else{
    action = argmaxQ();
  }

  return action;
}

// Load w
bool LSPIAgent::loadWeights( char *filename )
{
    double dWeights[NUM_FEATURES];
    cout << "Loading weights from " << filename << " ...";
    // Load the weights from the file to the double array
    ifstream inStream(filename, ios::in | ios::binary);
    if (!inStream)
        return false;
    inStream.read((char *) &dWeights, sizeof dWeights);
    inStream.close();

    // Copy the weights to weight vector
    for (int i = 0; i < NUM_FEATURES; i++){
        weights(i) = dWeights[i];
    }
    cout << "done" << endl;
    return true;
}

// Save w
bool LSPIAgent::saveWeights(char *filename)
{
    cout << "Saving weights to " << filename << " ...";
    // Put weights into a double array
    double dWeights[NUM_FEATURES];
    for (int i = 0; i < NUM_FEATURES; i++){
        dWeights[i] = weights(i); 
    }
    // Write the weight array to file
    ofstream outStream(filename, ios::out | ios::binary | ios::trunc);
    if (!outStream)
        return false;
    outStream.write( (char *) &dWeights, sizeof dWeights);
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
// TODO:
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
        for (int s = 0; s < NUM_STATE_FEAT; s++){
            features(s) = state[(a * NUM_STATE_FEAT) + s];
        }
        // Combine with action
        for (int i = 0; i < NUM_ACTIONS; i++){
            if (a == i)
                features(NUM_STATE_FEAT + i) = 1;
            else
                features(NUM_STATE_FEAT + i) = 0;
        }

        // Multiply by weight vector (i.e. call computeQa) and store in Q[a]
        Q[a] = computeQa(features);

    }
    
}

// Consider making this inline
double LSPIAgent::computeQa(VectorXd features){
    // Multiply feature vector by weight vector
    return features.dot(weights);     
}

// Returns index (action) of largest entry in Q array, breaking ties randomly 
int LSPIAgent::argmaxQ()
{
  int bestAction = 0;
  double bestValue = Q[ bestAction ];
  int numTies = 0;
  for ( int a = 1; a < getNumActions(); a++ ) {
    double value = Q[ a ];
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

    A = MatrixXd::Constant(NUM_FEATURES, NUM_FEATURES, 0);
    b = VectorXd::Constant(NUM_FEATURES, 0);

    for (unsigned int i = 0; i < D.size();){
        
        // Recover s, a
        VectorXd stateAction(NUM_FEATURES);
        for (int j = 0; j < NUM_FEATURES; j++){
            stateAction(j) = D[i];
            i++;
        }

        // Recover r
        double reward = D[i++];

        // Recover s'
        VectorXd nextStateAction(NUM_FEATURES);
        double sPrime[NUM_STATE_FEAT * NUM_ACTIONS];
        for (int j = 0; j < NUM_STATE_FEAT; j++){
            nextStateAction(j) = D[i];
            for (int k = 0; k < NUM_ACTIONS; k++){
                sPrime[ (k * NUM_STATE_FEAT) + j ] = D[i];     
            }
            i++;
        }
        // Find out what a' should be and populate into next state-action feature vector
        computeQ(sPrime);
        int action = selectAction();
        for (int j = 0; j < NUM_ACTIONS; j++){
            nextStateAction(NUM_STATE_FEAT + j) = (j == action) ? 1 : 0;
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

    // Update weights. This first call assumes that A and b have been updating from the step and end episode methods
    updateWeights();

    while (weightDifference(lastWeights, weights) > theta){
        lastWeights << weights;     // Keep track of current weights
        loadAbFromD();              // Update A and b with out new weights
        updateWeights();            // Calculate new weights
    }
    return true;
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
