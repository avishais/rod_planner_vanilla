/*
 * Checker.h
 *
 *  Created on: Oct 28, 2016
 *      Author: avishai
 */

#ifndef CHECKER_H_
#define CHECKER_H_

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/StateValidityChecker.h>
#include "ompl/base/MotionValidator.h"
#include "ompl/base/State.h"
#include <ompl/config.h>

#include "Rod_ODE_class.h"
#include "abb_apc_class.h"
#include "kdl_class.h"
#include "collisionDetection.h"

#include <iostream>

#define ROBOTS_DISTANCE 1085.85

namespace ob = ompl::base;
using namespace std;

class StateVector {
public:
	StateVector(int num) {
		n = num;
		q1.resize(n);
		q2.resize(n);
		a.resize(6);
	}

	void copy(State aa, State qq1, State qq2) {
		q1 = qq1;
		q2 = qq2;
		for (int i = 0; i < n; i++) {
			q[i] = qq1[i];
			q[i+6] = qq2[i];
		}
		a = aa;
	}
	void copy(State aa, State qq) {
		for (int i = 0; i < n; i++) {
			q1[i] = qq[i];
			q2[i] = qq[i+6];
		}
		q = qq;
		a = aa;
	}

	void print() {
		cout << "a: [";
		for (int i = 0; i < a.size(); i++)
			cout << a[i] << " ";
		cout << "]\n";
		cout << "q: [";
		for (int i = 0; i < q.size(); i++)
			cout << q[i] << " ";
		cout << "]\n";
	}

	int n;
	State q1;
	State q2;
	State q;
	State a;
};

class StateValidityChecker : public rod_ode, public two_abb_arms, public kdl, public collisionDetection
{
public:
	StateValidityChecker(const ob::SpaceInformationPtr &si) : mysi_(si.get()), two_abb_arms({-ROBOTS_DISTANCE/2, 0, 0 }, {ROBOTS_DISTANCE/2, 0, PI_}), kdl(ROBOTS_DISTANCE), collisionDetection(ROBOTS_DISTANCE,0,0,0)
{
		q_temp.resize(6);
		close_chain_return_IK.resize(2);
		q1_prev.resize(6);
		q2_prev.resize(6);
}; //Constructor // Avishai
	StateValidityChecker() : two_abb_arms({-ROBOTS_DISTANCE/2, 0, 0 }, {ROBOTS_DISTANCE/2, 0, PI_}), kdl(ROBOTS_DISTANCE), collisionDetection(ROBOTS_DISTANCE,0,0,0) {};

	// ----------------------- v APC functions v ----------------------------

	/** Identify the IK solutions of a configuration using two passive chains */
	State identify_state_ik(const ob::State *);
	State identify_state_ik(const ob::State *state, Matrix Q);
	State identify_state_ik(State, State, Matrix);

	/** Project a configuration in the ambient space to the constraint surface (and check collisions and joint limits) */
	bool APCproject(State a, State &q1, State &q2, int &active_chain, State ik_nn); // Try to project to both the neighbors connected components
	bool APCproject(State a, State &q1, State &q2, int &active_chain, int); // Try to project to a specific connected component of a certain active chain

	/** Sample a random configuration */
	bool APCsample(ob::State *);

	// ----------------------- ^ APC functions ^ ----------------------------
	// ----------------------- v GD functions v ----------------------------

	/** Sample a random configuration */
	bool GDsample(ob::State *);

	/** Project a random configuration */
	bool GDproject(State, State&);
	bool GDproject(State &, Matrix);

	// ----------------------- ^ GD functions ^ ----------------------------

	/** Log a configuration into path file for simulation */
	void log_q(State a, State q1, State q2);

	/** Validity check of a configuration - update state after projection */
	bool isValid(StateVector &S, int active_chain, int IK_sol); // For APC
	bool isValid(StateVector &S); // For GD

	bool checkMotionRBS(const ob::State *s1, const ob::State *s2);
	bool checkMotionRBS(StateVector S1, StateVector S2, int recursion_depth, int non_decrease_count);
	bool checkMotionRBS(const ob::State *s1, const ob::State *s2, int active_chain, int ik_sol);
	bool checkMotionRBS(StateVector S1, StateVector S2, int active_chain, int ik_sol, int recursion_depth, int non_decrease_count);


	double normDistanceStateVector(StateVector S1, StateVector S2);
	void midpoint(StateVector S1, StateVector S2, StateVector &S_mid);

	// Retrieve and update
	void retrieveStateVector(const ob::State *state, State &a, State &q);
	void retrieveStateVector(const ob::State *state, State &a, State &q1, State &q2);
	void retrieveStateVector(const ob::State *state, State &a);
	void updateStateVector(const ob::State *state, State q1, State q2);
	void updateStateVector(const ob::State *state, State a, State q1, State q2);
	void updateStateVector(const ob::State *state, State a);
	void printStateVector(const ob::State *state);

	void defaultSettings();
	double normDistance(State, State);
	double StateDistance(const ob::State *s1, const ob::State *s2);

	/** Join the two robots joint vectors */
	State join_Vectors(State, State);

	/** Decouple the two robots joint vectors */
	void seperate_Vector(State, State &, State &);

	int get_valid_solution_index() {
		return valid_solution_index;
	}

	State get_close_chain_return_IK() {
		return close_chain_return_IK;
	}

	// Performance parameters
	int isValid_counter;
	int get_isValid_counter() {
		return isValid_counter;
	}
private:
	ob::StateSpace *stateSpace_;
	ob::SpaceInformation    *mysi_;
	State q_temp;
	int valid_solution_index;
	State close_chain_return_IK;

	State q1_prev, q2_prev;

	bool withObs = true; // Include obstacles?
	double RBS_tol = 0.05;
	double RBS_max_depth = 100;

};





#endif /* CHECKER_H_ */
