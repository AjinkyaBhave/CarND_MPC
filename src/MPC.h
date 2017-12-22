#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;
#define MPH2mps 0.447

class MPC {
 public:
  // State vector dimension
  int n_states;
  // Number of actuators in vehicle
  int n_actuators;
  // Constructor and destructor
  MPC();
  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
};

#endif /* MPC_H */
