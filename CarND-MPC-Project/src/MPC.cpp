#include "MPC.h"
#include <math.h>
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"
#include "stdlib.h"
#include "math.h"

using CppAD::AD;

// TODO: Set the timestep length and duration
size_t N = 0;
double dt = 0.1;

// This value assumes the model presented in the classroom is used.
//
// It was obtained by measuring the radius formed by running the vehicle in the
// simulator around in a circle with a constant steering angle and velocity on a
// flat terrain.
//
// Lf was tuned until the the radius formed by the simulating the model
// presented in the classroom matched the previous radius.
//
// This is the length from front to CoG that has a similar radius.
const double Lf = 2.67;

// Both the reference cross track and orientation errors are 0.
// The reference velocity is set to 40 mph.
double ref_cte = 0;
double ref_epsi = 0;
double ref_v = 50;

// The solver takes all the state variables and actuator
// variables in a singular vector. Thus, we should to establish
// when one variable starts and another ends to make our lifes easier.
size_t x_start = 0;
size_t y_start = x_start + N;
size_t psi_start = y_start + N;
size_t v_start = psi_start + N;
size_t cte_start = v_start + N;
size_t epsi_start = cte_start + N;
size_t delta_start = epsi_start + N;
size_t a_start = delta_start + N - 1;

class FG_eval {
public:
	// Fitted polynomial coefficients
	Eigen::VectorXd coeffs;
	FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }

	typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
	void operator()(ADvector& fg, const ADvector& vars) {
		// TODO: implement MPC
		// fg a vector of constraints, x is a vector of constraints.
		// NOTE: You'll probably go back and forth between this function and
		// the Solver function below.

		// The cost is stored is the first element of `fg`.
		// Any additions to the cost should be added to `fg[0]`.
		fg[0] = 0;

		// The part of the cost based on the reference state.
		for (int i = 0; i < N; i++) {
			fg[0] += CppAD::pow(vars[cte_start + i] - ref_cte, 2);
			fg[0] += CppAD::pow(vars[epsi_start + i] - ref_epsi, 2);
			fg[0] += CppAD::pow(vars[v_start + i] - ref_v, 2);
		}

		// Minimize the use of actuators.
		for (int i = 0; i < N - 1; i++) {
			fg[0] += 100*CppAD::pow(vars[delta_start + i], 2);
			fg[0] += 100*CppAD::pow(vars[a_start + i], 2);
		}

		// Minimize the value gap between sequential actuations.
		for (int i = 0; i < N - 2; i++) {
			fg[0] += CppAD::pow(vars[delta_start + i + 1] - vars[delta_start + i], 2);
			fg[0] += CppAD::pow(vars[a_start + i + 1] - vars[a_start + i], 2);
		}

		//
		// Setup Constraints
		//
		// NOTE: In this section you'll setup the model constraints.

		// Initial constraints
		//
		// We add 1 to each of the starting indices due to cost being located at
		// index 0 of `fg`.
		// This bumps up the position of all the other values.
		fg[1 + x_start] = vars[x_start];
		fg[1 + y_start] = vars[y_start];
		fg[1 + psi_start] = vars[psi_start];
		fg[1 + v_start] = vars[v_start];
		fg[1 + cte_start] = vars[cte_start];
		fg[1 + epsi_start] = vars[epsi_start];

		// The rest of the constraints
		for (int i = 0; i < N - 1; i++) {
			// The state at time t+1 .
			AD<double> x1 = vars[x_start + i + 1];
			AD<double> y1 = vars[y_start + i + 1];
			AD<double> psi1 = vars[psi_start + i + 1];
			AD<double> v1 = vars[v_start + i + 1];
			AD<double> cte1 = vars[cte_start + i + 1];
			AD<double> epsi1 = vars[epsi_start + i + 1];

			// The state at time t.
			AD<double> x0 = vars[x_start + i];
			AD<double> y0 = vars[y_start + i];
			AD<double> psi0 = vars[psi_start + i];
			AD<double> v0 = vars[v_start + i];
			AD<double> cte0 = vars[cte_start + i];
			AD<double> epsi0 = vars[epsi_start + i];

			// Only consider the actuation at time t.
			AD<double> delta0 = vars[delta_start + i];
			AD<double> a0 = vars[a_start + i];

			AD<double> f0 = 0;
			AD<double> psides0 = 0;
			for (int i = 0; i < coeffs.size(); i++) {
				f0 += coeffs[i] * pow(x0, i);
				if(i > 0)
					psides0 += i*coeffs[i] * pow(y0, (i-1));
			}
			psides0 = CppAD::atan(psides0);

			// Here's `x` to get you started.
			// The idea here is to constraint this value to be 0.
			//
			// Recall the equations for the model:
			// x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
			// y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
			// psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
			// v_[t+1] = v[t] + a[t] * dt
			// cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
			// epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt
			fg[2 + x_start + i] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
			fg[2 + y_start + i] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
			fg[2 + psi_start + i] = psi1 - (psi0 + v0 * delta0 / Lf * dt);
			fg[2 + v_start + i] = v1 - (v0 + a0 * dt);
			fg[2 + cte_start + i] =
					cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
			fg[2 + epsi_start + i] =
					epsi1 - ((psi0 - psides0) + v0 * delta0 / Lf * dt);
		}

	}
};

//
// MPC class definition implementation.
//
MPC::MPC() {}
MPC::~MPC() {}

vector<double> MPC::Solve(Eigen::VectorXd x0, Eigen::VectorXd coeffs,size_t aN) {
	N = aN;

	x_start = 0;
	y_start = x_start + N;
	psi_start = y_start + N;
	v_start = psi_start + N;
	cte_start = v_start + N;
	epsi_start = cte_start + N;
	delta_start = epsi_start + N;
	a_start = delta_start + N - 1;

	bool ok = true;
	size_t i;
	typedef CPPAD_TESTVECTOR(double) Dvector;

	double x = x0[0];
	double y = x0[1];
	double psi = x0[2];
	double v = x0[3];
	double cte = x0[4];
	double epsi = x0[5];

	ref_v = 80;
	//reduce referenz speed when the coeef indicate a sharp turn
	if(abs(coeffs[1])>0.03){
		ref_v = 40;
		std::cout << "ref_v:" << ref_v << std::endl;
	}

	// number of independent variables
	// N timesteps == N - 1 actuations
	size_t n_vars = N * 6 + (N - 1) * 2;
	// Number of constraints
	size_t n_constraints = N * 6;

	// Initial value of the independent variables.
	// SHOULD BE 0 besides initial state.
	Dvector vars(n_vars);
	for (int i = 0; i < n_vars; i++) {
		vars[i] = 0.0;
	}

	// Set the initial variable values
	vars[x_start] = x;
	vars[y_start] = y;
	vars[psi_start] = psi;
	vars[v_start] = v;
	vars[cte_start] = cte;
	vars[epsi_start] = epsi;
	Dvector vars_lowerbound(n_vars);
	Dvector vars_upperbound(n_vars);

	// Set all non-actuators upper and lowerlimits
	// to the max negative and positive values.
	for (int i = 0; i < delta_start; i++) {
		vars_lowerbound[i] = -1.0e19;
		vars_upperbound[i] = 1.0e19;
	}
	//limit x to 0 or positive values
	for (int i = psi_start; i < y_start; i++) {
		vars_lowerbound[i] = 0;
	}

	// The upper and lower limits of psi is 45 degrees left right per step
	double PI_HALF = M_PI*0.25;
	for (int i = psi_start; i < v_start; i++) {
		vars_lowerbound[i] = - PI_HALF;
		vars_upperbound[i] = PI_HALF;
	}

	// The upper and lower limits of delta are set to -1 and 1
	for (int i = delta_start; i < a_start; i++) {
		vars_lowerbound[i] = -1;
		vars_upperbound[i] = 1;
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
	for (int i = 0; i < n_constraints; i++) {
		constraints_lowerbound[i] = 0;
		constraints_upperbound[i] = 0;
	}

	constraints_lowerbound[x_start] = x;
	constraints_lowerbound[y_start] = y;
	constraints_lowerbound[psi_start] = psi;
	constraints_lowerbound[v_start] = v;
	constraints_lowerbound[cte_start] = cte;
	constraints_lowerbound[epsi_start] = epsi;

	constraints_upperbound[x_start] = x;
	constraints_upperbound[y_start] = y;
	constraints_upperbound[psi_start] = psi;
	constraints_upperbound[v_start] = v;
	constraints_upperbound[cte_start] = cte;
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
	options += "Numeric max_cpu_time          0.05\n";

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
	std::cout << "Cost " << cost << " status:" << solution.status << " sollution size: " << solution.x.size() << std::endl;

	// TODO: Return the first actuator values. The variables can be accessed with
	// `solution.x[i]`.
	//
	// {...} is shorthand for creating a vector, so auto x1 = {1.0,2.0}
	// creates a 2 element double vector.

	int point_step = N/5;
	return {solution.x[x_start + 1],   solution.x[y_start + 1],
		solution.x[psi_start + 1], solution.x[v_start + 1],
		solution.x[cte_start + 1], solution.x[epsi_start + 1],
		solution.x[delta_start],   solution.x[a_start],
		solution.x[delta_start+1],   solution.x[a_start+1],
		(double)solution.status,
		solution.x[x_start + 1],   solution.x[y_start + 1],
		solution.x[x_start + point_step],   solution.x[y_start + point_step],
		solution.x[x_start + point_step*2],   solution.x[y_start + point_step*2],
		solution.x[x_start + point_step*3],   solution.x[y_start + point_step*3],
		solution.x[x_start + N-1],   solution.x[y_start + N-1]
	};
}

