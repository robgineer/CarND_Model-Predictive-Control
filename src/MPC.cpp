#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;

size_t N = 10;
double dt = 0.1;

// Lf value (taken over from tutorial)
const double Lf = 2.67;

// The reference velocity is set to 40 mph.
double ref_v = 100;

// Define positions of control inputs within the vars vector
size_t x_start 		= 0;
size_t y_start 		= x_start + N;
size_t psi_start 	= y_start + N;
size_t v_start 		= psi_start + N;
size_t cte_start 	= v_start + N;
size_t epsi_start 	= cte_start + N;
size_t delta_start 	= epsi_start + N;
size_t a_start 		= delta_start + N - 1;


class FG_eval {
 public:

// Fitted polynomial coefficients
  Eigen::VectorXd coeffs;

  FG_eval(Eigen::VectorXd coeffs)
  {
	  this->coeffs = coeffs;
  }

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;

  void operator()(ADvector& fg, const ADvector& vars)
  {
	  // Define the cost function. Stored at fg[0]
	  // takeover from lecture notes: J = SUM((cte_t-cte_ref)**2+(ephi_t-ephi_ref)**2+...)

	  // reset vector values for each execution of the operator() function
	  fg[0] = 0;

	  // iterate through the horizon steps
	  for(int t = 0; t < N; t++)
	  {
		  // CTE part of cost function for each horizon step
		  fg[0] += 2000*CppAD::pow(vars[cte_start + t], 2);
		  // Orientation error part of cost function for each horizon step
		  fg[0] += 2000*CppAD::pow(vars[psi_start + t], 2);
		  // Velocity error part of cost function for each horizon step
		  fg[0] += CppAD::pow(vars[v_start + t] - ref_v, 2);
	  }

	  // iterate through the horizon steps
	  for (int t = 0; t < N - 1; t++)
	  {
		  fg[0] += 1000 * CppAD::pow(vars[delta_start + t], 2);
		  fg[0] += 100 * CppAD::pow(vars[a_start + t], 2);
	  }

	  // Minimize the discretization error
	  for (int t = 0; t < N - 2; t++)
	  {
		  // add a smooth delta between two steering actuation
		  fg[0] += 3000 * CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
		  // add a smooth delta between two throttle valve actuation
	      fg[0] += 1000 * CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
	  }

	  // Set initial values for x, y, orientation, velocity, cte and psi error
	  // Stored at fg with index > 0
	  fg[1 + x_start] 	= vars[x_start];
	  fg[1 + y_start] 	= vars[y_start];
	  fg[1 + psi_start] 	= vars[psi_start];
	  fg[1 + v_start] 	= vars[v_start];
	  fg[1 + cte_start] 	= vars[cte_start];
	  fg[1 + epsi_start] = vars[epsi_start];

	  // Define the constraints of the input parameters for each horizon step
	  for (int t = 1; t < N; t++)
	  {
		  // Current state vector.
		  AD<double> x0 		= vars[x_start + t - 1];
		  AD<double> y0 		= vars[y_start + t - 1];
		  AD<double> psi0 	= vars[psi_start + t - 1];
		  AD<double> v0 		= vars[v_start + t - 1];
		  AD<double> cte0 	= vars[cte_start + t - 1];
		  AD<double> epsi0 	= vars[epsi_start + t - 1];

		  // Next state vector.
		  AD<double> x1 		= vars[x_start + t];
	      AD<double> y1 		= vars[y_start + t];
	      AD<double> psi1 	= vars[psi_start + t];
	      AD<double> v1 		= vars[v_start + t];
	      AD<double> cte1 	= vars[cte_start + t];
	      AD<double> epsi1 	= vars[epsi_start + t];

	      // Current steering actuation.
	      AD<double> delta0 	= vars[delta_start + t - 1];
	      // Current throttle actuation.
	      AD<double> a0 		= vars[a_start + t - 1];

	      AD<double> f0 		= coeffs[0]
								+ coeffs[1] * x0
									+ coeffs[2] * CppAD::pow(x0,2)
	      	  	  	  	  	  	  	  + coeffs[3] * CppAD::pow(x0,3);

	      AD<double> psides0 = CppAD::atan(	3 * coeffs[3] * CppAD::pow(x0,2)
	      	  	  	  	  	  	  	  	  + 2 * coeffs[2] * x0
										  	  + coeffs[1]);

	      //AD<double> f0 		= coeffs[0] + coeffs[1] * x0;
	      //AD<double> psides0 = CppAD::atan(coeffs[1]);

	      // Update values of state values based on vehicle model
	      fg[1 + x_start + t] 		= x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
	      fg[1 + y_start + t] 		= y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
	      fg[1 + psi_start + t] 		= psi1 - (psi0 - v0 * delta0 / Lf * dt);
	      fg[1 + v_start + t] 		= v1 - (v0 + a0 * dt);
	      fg[1 + cte_start + t] 		= cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
	      fg[1 + epsi_start + t] 	= epsi1 - ((psi0 - psides0) - v0 * delta0 / Lf * dt);
	 }
  }
};

//
// MPC class definition implementation.
//
MPC::MPC() {}
MPC::~MPC() {}

vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {
  bool ok = true;
  size_t i;
  typedef CPPAD_TESTVECTOR(double) Dvector;

  // get initial values of the state vector
  double x 		= state[0];
  double y 		= state[1];
  double psi 	= state[2];
  double v 		= state[3];
  double cte 	= state[4];
  double epsi 	= state[5];

  // variables contain 6 states (x,y,psi,v,cte,epsi) + 2 control inputs (delta, a)
  // while the states are defined for each horizon step (N), the control inputs are defined for each horizon
  // step transition (N-1).
  size_t n_vars 			= N * 6 + (N-1) * 2;
  size_t n_constraints 	= N * 6;

  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.
  Dvector vars(n_vars);
  for (int i = 0; i < n_vars; i++)
  {
    vars[i] = 0;
  }

  // Assign intial values of each state variable
  vars[x_start] 		= x;
  vars[y_start] 		= y;
  vars[psi_start] 	= psi;
  vars[v_start] 		= v;
  vars[cte_start] 	= cte;
  vars[epsi_start] 	= epsi;

  // Set all non-actuators upper and lowerlimits
  // to the max negative and positive values.
  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);
  for (int i = 0; i < delta_start; i++)
  {
	  vars_lowerbound[i] = -1.0e19;
	  vars_upperbound[i] = 1.0e19;
  }

  // The upper and lower limits of delta are set to -25 and 25
  // degrees (values in radians).
  // NOTE: Feel free to change this to something else.
  for (int i = delta_start; i < a_start; i++) {
	  vars_lowerbound[i] = -0.436332;
	  vars_upperbound[i] = 0.436332;
  }

  // Acceleration/decceleration upper and lower limits.
  // NOTE: Feel free to change this to something else.
  for (int i = a_start; i < n_vars; i++) {
	  vars_lowerbound[i] = -1.0;
	  vars_upperbound[i] = 1.0;
  }

  // Lower and upper limits for the constraints
  // Should be 0 besides initial state.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (int i = 0; i < n_constraints; i++)
  {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }

  constraints_lowerbound[x_start] 	= x;
  constraints_lowerbound[y_start] 	= y;
  constraints_lowerbound[psi_start] 	= psi;
  constraints_lowerbound[v_start] 	= v;
  constraints_lowerbound[cte_start] 	= cte;
  constraints_lowerbound[epsi_start] = epsi;

  constraints_upperbound[x_start] 	= x;
  constraints_upperbound[y_start] 	= y;
  constraints_upperbound[psi_start] 	= psi;
  constraints_upperbound[v_start] 	= v;
  constraints_upperbound[cte_start] 	= cte;
  constraints_upperbound[epsi_start] = epsi;

  // object that computes objective and constraints
  FG_eval fg_eval(coeffs);

  //
  // NOTE: You don't have to worry about these options
  //
  // options for IPOPT solver
  std::string options;
  // Uncomment this if you'd like more print information
  options += "Integer print_level  0\n";
  // NOTE: Setting sparse to true allows the solver to take advantage
  // of sparse routines, this makes the computation MUCH FASTER. If you
  // can uncomment 1 of these and see if it makes a difference or not but
  // if you uncomment both the computation time should go up in orders of
  // magnitude.
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";
  // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
  // Change this as you see fit.
  options += "Numeric max_cpu_time          0.5\n";

  // place to return solution
  CppAD::ipopt::solve_result<Dvector> solution;

  // solve the problem
  CppAD::ipopt::solve<Dvector, FG_eval>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution);

  // Check some of the solution values
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

  // Cost
  auto cost = solution.obj_value;
  std::cout << "Cost " << cost << std::endl;

  vector<double> result;
  result.push_back(solution.x[delta_start]);
  result.push_back(solution.x[a_start]);

  for(int i = 0; i < N-1; i++)
  {
	  result.push_back(solution.x[x_start + i + 1]);
	  result.push_back(solution.x[y_start + i + 1]);
  }
  return result;

}
