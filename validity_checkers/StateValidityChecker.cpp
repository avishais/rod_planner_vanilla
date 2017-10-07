/*
 * Checker.cpp
 *
 *  Created on: Oct 28, 2016
 *      Author: avishai
 */

/*
myStateValidityCheckerClass::myStateValidityCheckerClass(const ob::SpaceInformationPtr &si) {

}*/

#include "StateValidityChecker.h"
#include <queue>

void StateValidityChecker::defaultSettings()
{
	stateSpace_ = mysi_->getStateSpace().get();
	if (!stateSpace_)
		OMPL_ERROR("No state space for motion validator");
}

void StateValidityChecker::retrieveStateVector(const ob::State *state, State &a, State &q) {
	// cast the abstract state type to the type we expect
	const ob::CompoundStateSpace::StateType *C_state = state->as<ob::CompoundStateSpace::StateType>();
	const ob::RealVectorStateSpace::StateType *A = C_state->as<ob::RealVectorStateSpace::StateType>(0);
	const ob::RealVectorStateSpace::StateType *Q = C_state->as<ob::RealVectorStateSpace::StateType>(1);

	for (unsigned i = 0; i < 6; i++) {
		a[i] = A->values[i]; // Get state of rod
		q[i] = Q->values[i]; // Get state of robots
	}
}

void StateValidityChecker::retrieveStateVector(const ob::State *state, State &a) {
	// cast the abstract state type to the type we expect
	const ob::CompoundStateSpace::StateType *C_state = state->as<ob::CompoundStateSpace::StateType>();
	const ob::RealVectorStateSpace::StateType *A = C_state->as<ob::RealVectorStateSpace::StateType>(0);

	for (unsigned i = 0; i < 6; i++)
		a[i] = A->values[i]; // Get state of rod
}

void StateValidityChecker::retrieveStateVector(const ob::State *state, State &a, State &q1, State &q2) {
	// cast the abstract state type to the type we expect
	const ob::CompoundStateSpace::StateType *C_state = state->as<ob::CompoundStateSpace::StateType>();
	const ob::RealVectorStateSpace::StateType *A = C_state->as<ob::RealVectorStateSpace::StateType>(0);
	const ob::RealVectorStateSpace::StateType *Q = C_state->as<ob::RealVectorStateSpace::StateType>(1);

	for (unsigned i = 0; i < 6; i++) {
		a[i] = A->values[i]; // Set state of rod
		q1[i] = Q->values[i]; // Set state of robot1
		q2[i] = Q->values[i+6]; // Set state of robot1
	}
}

void StateValidityChecker::updateStateVector(const ob::State *state, State q1, State q2) {
	// cast the abstract state type to the type we expect
	const ob::CompoundStateSpace::StateType *C_state = state->as<ob::CompoundStateSpace::StateType>();
	//const ob::RealVectorStateSpace::StateType *A = C_state->as<ob::RealVectorStateSpace::StateType>(0);
	const ob::RealVectorStateSpace::StateType *Q = C_state->as<ob::RealVectorStateSpace::StateType>(1);

	for (unsigned i = 0; i < 6; i++) {
		Q->values[i] = q1[i];
		Q->values[i+6]= q2[i];
	}
}

void StateValidityChecker::updateStateVector(const ob::State *state, State a, State q1, State q2) {
	// cast the abstract state type to the type we expect
	const ob::CompoundStateSpace::StateType *C_state = state->as<ob::CompoundStateSpace::StateType>();
	const ob::RealVectorStateSpace::StateType *A = C_state->as<ob::RealVectorStateSpace::StateType>(0);
	const ob::RealVectorStateSpace::StateType *Q = C_state->as<ob::RealVectorStateSpace::StateType>(1);

	for (unsigned i = 0; i < 6; i++) {
		A->values[i] = a[i];
		Q->values[i] = q1[i];
		Q->values[i+6]= q2[i];
	}
}

void StateValidityChecker::updateStateVector(const ob::State *state, State a) {
	// cast the abstract state type to the type we expect
	const ob::CompoundStateSpace::StateType *C_state = state->as<ob::CompoundStateSpace::StateType>();
	const ob::RealVectorStateSpace::StateType *A = C_state->as<ob::RealVectorStateSpace::StateType>(0);

	for (unsigned i = 0; i < 6; i++)
		A->values[i] = a[i];
}

void StateValidityChecker::printStateVector(const ob::State *state) {
	// cast the abstract state type to the type we expect
	const ob::CompoundStateSpace::StateType *C_state = state->as<ob::CompoundStateSpace::StateType>();
	const ob::RealVectorStateSpace::StateType *A = C_state->as<ob::RealVectorStateSpace::StateType>(0);
	const ob::RealVectorStateSpace::StateType *Q = C_state->as<ob::RealVectorStateSpace::StateType>(1);

	State a(6), q1(6), q2(6);

	for (unsigned i = 0; i < 6; i++) {
		a[i] = A->values[i]; // Set state of rod
		q1[i] = Q->values[i]; // Set state of robot1
		q2[i] = Q->values[i+6]; // Set state of robot1
	}

	cout << "a: "; kdl::printVector(a);
	cout << "q1: "; kdl::printVector(q1);
	cout << "q2: "; kdl::printVector(q2);
}

// -------------------------------- APC functions --------------------------------------

bool StateValidityChecker::APCproject(State a, State &q1, State &q2, int &active_chain, int ik_sol) {

	two_abb_arms::IK_counter++;
	bool valid = false;
	clock_t sT = clock();

	if (!isRodFeasible(a))
		return false;
	Matrix Q = getT(get_Points_on_Rod()-1);

	if (!active_chain) {
		if (calc_specific_IK_solution_R1(Q, q1, ik_sol)) {
			q2 = get_IK_solution_q2();
			valid = true;
		}
	}
	else {
		if (calc_specific_IK_solution_R2(Q, q2, ik_sol)) {
			q1 = get_IK_solution_q1();
			valid = true;
		}
	}

	two_abb_arms::IK_time += double(clock() - sT) / CLOCKS_PER_SEC;

	if (valid && withObs && collision_state(getPMatrix(), q1, q2))
		valid = false;

	return valid;
}

bool StateValidityChecker::APCproject(State a, State &q1, State &q2, int &active_chain, State ik_nn) {

	two_abb_arms::IK_counter++;
	bool valid = true;
	clock_t sT = clock();

	if (!isRodFeasible(a))
		return false;
	Matrix Q = getT(get_Points_on_Rod()-1);

	if (!active_chain) {
		if (!calc_specific_IK_solution_R1(Q, q1, ik_nn[0])) {
			if (!calc_specific_IK_solution_R2(Q, q2, ik_nn[1]))
				valid = false;
			else {
				active_chain = !active_chain;
				q1 = get_IK_solution_q1();
			}
		}
		else {
			q2 = get_IK_solution_q2();
		}
	}
	else {
		if (!calc_specific_IK_solution_R2(Q, q2, ik_nn[0])) {
			if (!calc_specific_IK_solution_R1(Q, q1, ik_nn[1]))
				valid = false;
			else {
				active_chain = !active_chain;
				q2 = get_IK_solution_q2();
			}
		}
		else {
			q1 = get_IK_solution_q1();
		}
	}

	two_abb_arms::IK_time += double(clock() - sT) / CLOCKS_PER_SEC;

	if (valid && withObs && collision_state(getPMatrix(), q1, q2))
		valid = false;

	return valid;
}

bool StateValidityChecker::APCsample(ob::State *st) {

	State a(6), q(12), q1(6), q2(6), ik(2);

	bool flag = true;
	while (flag) {
		// Random rod configuration
		for (int i = 0; i < 6; i++)
			a[i] = ((double) rand() / (RAND_MAX)) * 2 * 30 - 30;
		if (!isRodFeasible(a))
			continue;
		Matrix Q = getT(get_Points_on_Rod()-1);

		// Try to close the robots for the given rod configuration
		for (int k = 0; k < 10; k++) {

			// Random active chain
			if (withObs)
				q1 = rand_q(6);
			else
				for (int i = 0; i < 6; i++)
					q1[i] = ((double) rand() / (RAND_MAX)) * 2 * PI_ - PI_;

			int ik_sol = rand() % 8;

			if (calc_specific_IK_solution_R1(Q, q1, ik_sol))
				q2 = get_IK_solution_q2();
			else
				continue;

			ik = identify_state_ik(q1, q2, Q);
			if (ik[0]==-1 && ik[1]==-1)
				continue;

			if (withObs && collision_state(getPMatrix(), q1, q2) && !two_abb_arms::check_angle_limits(q1, q2))
				continue;

			flag = false;
			break;
		}
	}

	updateStateVector(st, a, q1, q2);
	return true;

}

State StateValidityChecker::identify_state_ik(const ob::State *state) {

	State a(6), q1(6), q2(6), ik(2);
	retrieveStateVector(state, a, q1, q2);

	if (!isRodFeasible(a))
		return {-1, -1};
	Matrix Q = getT(get_Points_on_Rod()-1);

	ik = identify_state_ik(q1, q2, Q);

	return ik;
}

State StateValidityChecker::identify_state_ik(const ob::State *state, Matrix Q) {

	State q1(6), q2(6), ik(2);
	retrieveStateVector(state, q1, q2);

	ik = identify_state_ik(q1, q2, Q);

	return ik;
}

State StateValidityChecker::identify_state_ik(State q1, State q2, Matrix Q) {

	State ik = {-1, -1};

	// q1 is the active chain
	FKsolve_rob(q1, 1);
	Matrix T2 = MatricesMult(get_FK_solution_T1(), Q); // Returns the opposing required matrix of the rods tip at robot 2
	T2 = MatricesMult(T2, {{-1, 0, 0, 0}, {0, -1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}}); // Returns the REQUIRED matrix of the rods tip at robot 2
	int n = calc_all_IK_solutions_2(T2);
	if (n == 0)
		return ik;

	for (int i = 0; i < n; i++) {
		q_temp = get_all_IK_solutions_2(i);
		if (normDistance(q_temp, q2)<1e-1) {
			ik[0] = get_valid_IK_solutions_indices_2(i);
			break;
		}
	}

	// q2 is the active chain
	Matrix Tinv = Q;
	InvertMatrix(Q, Tinv); // Invert matrix
	FKsolve_rob(q2, 2);
	Matrix T1 = MatricesMult(get_FK_solution_T2(), {{-1, 0, 0, 0}, {0, -1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}}); // Returns the opposing required matrix of the rods tip at robot 2
	T1 = MatricesMult(T1, Tinv); // Returns the REQUIRED matrix of the rods tip at rob
	n = calc_all_IK_solutions_1(T1);
	if (n == 0)
		return ik;

	for (int i = 0; i < n; i++) {
		q_temp = get_all_IK_solutions_1(i);
		if (normDistance(q_temp,q1)<1e-1) {
			ik[1] = get_valid_IK_solutions_indices_1(i);
			break;
		}
	}

	return ik;
}

// ----------------------- v GD functions v ----------------------------

bool StateValidityChecker::GDsample(ob::State *st) {

	State a(6), q(12), q1(6), q2(6);

	if (!GDsample(a, q))
		return false;

	seperate_Vector(q, q1, q2);

	updateStateVector(st, a, q1, q2);
	return true;
}

bool StateValidityChecker::GDsample(State &a, State &q) {

	State q1(6), q2(6);

	bool flag = true;
	while (flag) {
		// Random rod configuration
		for (int i = 0; i < 6; i++)
			a[i] = ((double) rand() / (RAND_MAX)) * 2 * 30 - 30;
		if (!isRodFeasible(a))
			continue;
		Matrix Q = getT(get_Points_on_Rod()-1);

		for (int k = 0; k < 10; k++) {

			// Random joint angles
			for (int i = 0; i < 12; i++)
				q[i] = ((double) rand() / (RAND_MAX)) * 2 * PI_ - PI_;

			if (!GD(q, Q)) // GD checks for joint limits
				continue;

			q = get_GD_result();

			seperate_Vector(q, q1, q2);
			if (withObs && collision_state(getPMatrix(), q1, q2))
				continue;

			flag = false;
			break;
		}
	}

	return true;
}

bool StateValidityChecker::GDproject(State a, State &q) {

	State q1(6), q2(6);

	kdl::IK_counter++;
	clock_t sT = clock();

	if (!isRodFeasible(a))
		return false;
	Matrix Q = getT(get_Points_on_Rod()-1);

	if (!GD(q, Q)) { // GD checks for joint limits
		kdl::IK_time += double(clock() - sT) / CLOCKS_PER_SEC;
		return false;
	}

	q = get_GD_result();

	seperate_Vector(q, q1, q2);
	if (withObs && collision_state(getPMatrix(), q1, q2)) {
		kdl::IK_time += double(clock() - sT) / CLOCKS_PER_SEC;
		return false;
	}

	kdl::IK_time += double(clock() - sT) / CLOCKS_PER_SEC;
	return true;
}

bool StateValidityChecker::GDproject(State &q, Matrix Q) {

	State q1(6), q2(6);

	kdl::IK_counter++;
	clock_t sT = clock();

	if (!GD(q, Q)) { // GD checks for joint limits
		kdl::IK_time += double(clock() - sT) / CLOCKS_PER_SEC;
		return false;
	}

	q = get_GD_result();

	seperate_Vector(q, q1, q2);
	if (withObs && collision_state(getPMatrix(), q1, q2)) {
		kdl::IK_time += double(clock() - sT) / CLOCKS_PER_SEC;
		return false;
	}

	kdl::IK_time += double(clock() - sT) / CLOCKS_PER_SEC;
	return true;
}

// ----------------------- ^ GD functions ^ ----------------------------


// ---------------------------------------------------------------

void StateValidityChecker::log_q(State a, State q1, State q2) {
	std::ofstream qfile, afile, pfile, ai;
	qfile.open("../paths/path.txt");
	afile.open("../paths/afile.txt");
	pfile.open("../paths/rod_path.txt");

	qfile << 1 << endl;
	pfile << 501 << endl;

	for (int j = 0; j < 6; j++) {
		qfile << q1[j] << " ";
		afile << a[j] << " ";
	}
	for (int j = 0; j<6; j++) {
		qfile << q2[j] << " ";
	}

	rod_solve(a);
	State temp(3);
	// Log points on rod to file
	for (int k = 0; k < get_Points_on_Rod(); k++) {
		temp = getP(k);
		pfile << temp[0] << " " << temp[1] << " "  << temp[2] << endl;
	}
	pfile << endl;

	qfile.close();
	afile.close();
	pfile.close();
}

// ---------------------------------------------------------------

double StateValidityChecker::normDistance(State a1, State a2) {
	double sum = 0;
	for (int i=0; i < a1.size(); i++)
		sum += pow(a1[i]-a2[i], 2);
	return sqrt(sum);
}

State StateValidityChecker::join_Vectors(State q1, State q2) {

	State q(q1.size()+q2.size());

	for (int i = 0; i < q1.size(); i++) {
		q[i] = q1[i];
		q[i+q1.size()] = q2[i];
	}

	return q;
}

void StateValidityChecker::seperate_Vector(State q, State &q1, State &q2) {

	for (int i = 0; i < q.size()/2; i++) {
		q1[i] = q[i];
		q2[i] = q[i+q.size()/2];
	}
}

double StateValidityChecker::StateDistance(const ob::State *s1, const ob::State *s2) {

	State aa(6), qa1(6), qa2(6);
	State ab(6), qb1(6), qb2(6);

	retrieveStateVector(s1, aa, qa1, qa2);
	retrieveStateVector(s2, ab, qb1, qb2);

	double sum = 0;
	for (int i=0; i < aa.size(); i++)
		sum += pow(aa[i]-ab[i], 2);
	for (int i=0; i < qa1.size(); i++)
		sum += pow(qa1[i]-qb1[i], 2) + pow(qa2[i]-qb2[i], 2);
	return sqrt(sum);
}


// ====================== Check validity ===============================

// Validates a state by computing the passive chain based on a specific IK solution (input) and checking collision
// Also updates the state after projection
bool StateValidityChecker::isValid(StateVector &S, int active_chain, int IK_sol) {

	isValid_counter++;

	if (!isRodFeasible(S.a))
		return false;

	switch (active_chain) {
	case 0:
		if (calc_specific_IK_solution_R1(getT(get_Points_on_Rod()-1), S.q1, IK_sol)) {
			S.q2 = get_IK_solution_q2();
			if (!collision_state(getPMatrix(), S.q1, S.q2))
				return true;
		}
		else
			return false;
		break;
	case 1:
		if (calc_specific_IK_solution_R2(getT(get_Points_on_Rod()-1), S.q2, IK_sol)) {
			S.q1 = get_IK_solution_q1();
			if (!collision_state(getPMatrix(), S.q1, S.q2))
				return true;
		}
		else
			return false;
	}
	return false;
}

// Validates a state by computing the passive chain based on a specific IK solution (input) and checking collision
bool StateValidityChecker::isValid(StateVector &S) {

	isValid_counter++;

	if (GDproject(S.a, S.q))
		return true;

	return false;
}

// ====================== v RBS - GD v ===============================

// Calls the Recursive Bi-Section algorithm (Hauser)
bool StateValidityChecker::checkMotionRBS(const ob::State *s1, const ob::State *s2)
{
	// We assume motion starts and ends in a valid configuration - due to projection
	bool result = true;

	State aa(6), ab(6), qa1(6), qa2(6), qb1(6), qb2(6);
	retrieveStateVector(s1, aa, qa1, qa2);
	retrieveStateVector(s2, ab,  qb1, qb2);

	StateVector S1(6);
	S1.copy(aa, qa1, qa2);
	StateVector S2(6);
	S2.copy(ab, qb1, qb2);


	result = checkMotionRBS(S1, S2, 0, 0);

	return result;
}

// Implements local-connection using Recursive Bi-Section Technique (Hauser)
bool StateValidityChecker::checkMotionRBS(StateVector S1, StateVector S2, int recursion_depth, int non_decrease_count) {

	// Check if reached the required resolution
	double d = normDistanceStateVector(S1, S2);

	if (d < RBS_tol)
		return true;

	if (recursion_depth > RBS_max_depth)// || non_decrease_count > 10)
		return false;

	StateVector S_mid(6);
	midpoint(S1, S2, S_mid);

	// Check obstacles collisions and joint limits
	if (!isValid(S_mid)) // Also updates s_mid with the projected value
		return false;

	//if ( normDistanceStateVector(S1, S_mid) > d || normDistanceStateVector(S_mid, S2) > d )
	//		non_decrease_count++;

	if ( checkMotionRBS(S1, S_mid, recursion_depth+1, non_decrease_count) && checkMotionRBS(S_mid, S2, recursion_depth+1, non_decrease_count) )
		return true;
	else
		return false;
}

// *************** Reconstruct the RBS - for post-processing and validation

// Calls the Recursive Bi-Section algorithm (Hauser)
bool StateValidityChecker::reconstructRBS(const ob::State *s1, const ob::State *s2, Matrix &Confs, Matrix &A)
{
	// We assume motion starts and ends in a valid configuration - due to projection
	bool result = true;

	State aa(6), ab(6), qa1(6), qa2(6), qb1(6), qb2(6);
	retrieveStateVector(s1, aa, qa1, qa2);
	retrieveStateVector(s2, ab,  qb1, qb2);

	StateVector S1(6);
	S1.copy(aa, qa1, qa2);
	StateVector S2(6);
	S2.copy(ab, qb1, qb2);

	Confs.push_back(S1.q);
	Confs.push_back(S2.q);
	A.push_back(S1.a);
	A.push_back(S2.a);

	return reconstructRBS(S1, S2, Confs, A, 0, 1, 1);
}

bool StateValidityChecker::reconstructRBS(StateVector S1, StateVector S2, Matrix &M, Matrix &A, int iteration, int last_index, int firstORsecond) {
	// firstORsecond - tells if the iteration is from the first or second call for the recursion (in the previous iteration).
	// last_index - the last index that was added to M.

	iteration++;

	// Check if reached the required resolution
	double d = normDistanceStateVector(S1, S2);

	if (d < RBS_tol)
		return true;

	if (iteration > RBS_max_depth)// || non_decrease_count > 10)
		return false;

	StateVector S_mid(6);
	midpoint(S1, S2, S_mid);

	// Check obstacles collisions and joint limits
	if (!isValid(S_mid)) // Also updates s_mid with the projected value
		return false; // Not suppose to happen since we run this function only when local connection feasibility is known

	if (firstORsecond==1) {
		M.insert(M.begin()+last_index, S_mid.q); // Inefficient operation, but this is only for post-processing and validation
		A.insert(A.begin()+last_index, S_mid.a);
	}
	else {
		M.insert(M.begin()+(++last_index), S_mid.q); // Inefficient operation, but this is only for post-processing and validation
		A.insert(A.begin()+last_index, S_mid.a);
	}

	int prev_size = M.size();
	if (!reconstructRBS(S1, S_mid, M, A, iteration, last_index, 1))
		return false;
	last_index += M.size()-prev_size;
	if (!reconstructRBS(S_mid, S2, M, A, iteration, last_index, 2))
		return false;

	return true;
}

// ====================== ^ RBS - GD ^ ===============================

/*
// Calls the Recursive Bi-Section algorithm (Hauser)
bool StateValidityChecker::checkMotionRBS(const ob::State *s1, const ob::State *s2, int active_chain, int ik_sol)
{
	// We assume motion starts and ends in a valid configuration - due to projection
	bool result = true;

	State aa(6), ab(6), qa1(6), qa2(6), qb1(6), qb2(6);
	retrieveStateVector(s1, aa, qa1, qa2);
	retrieveStateVector(s2, ab,  qb1, qb2);

	StateVector S1(6);
	S1.copyVector(aa, qa1, qa2);
	StateVector S2(6);
	S2.copyVector(ab, qb1, qb2);


	result = RBS(S1, S2, active_chain, ik_sol, 0, 0);

	return result;
}

// Implements local-connection using Recursive Bi-Section Technique (Hauser)
bool StateValidityChecker::checkMotionRBS(StateVector S1, StateVector S2, int active_chain, int ik_sol, int recursion_depth, int non_decrease_count) {

	// Check if reached the required resolution
	double d = normDistanceStateVector(S1, S2);

	if (d < 0.1)
		return true;

	if (recursion_depth > 100)// || non_decrease_count > 10)
		return false;

	StateVector S_mid(6);
	midpoint(S1, S2, S_mid);

	// Check obstacles collisions and joint limits
	if (!isValidRBS(S_mid, active_chain, ik_sol)) // Also updates s_mid with the projected value
		return false;

	if ( normDistanceStateVector(S1, S_mid) > d || normDistanceStateVector(S_mid, S2) > d )
			non_decrease_count++;

	if ( checkMotionRBS(S1, S_mid, active_chain, ik_sol, recursion_depth+1, non_decrease_count) && checkMotionRBS(S_mid, S2, active_chain, ik_sol, recursion_depth+1, non_decrease_count) )
		return true;
	else
		return false;
}

 */

double StateValidityChecker::normDistanceStateVector(StateVector S1, StateVector S2) {
	double sum = 0;
	for (int i=0; i < S1.n; i++)
		sum += pow(S1.q1[i]-S2.q1[i], 2) + pow(S1.q2[i]-S2.q2[i], 2);
	for (int i=0; i < 6; i++)
		sum += pow(S1.a[i]-S2.a[i], 2);
	return sqrt(sum);
}

void StateValidityChecker::midpoint(StateVector S1, StateVector S2, StateVector &S_mid) {

	for (int i = 0; i < S_mid.n; i++) {
		S_mid.q1[i] = (S1.q1[i]+S2.q1[i])/2;
		S_mid.q2[i] = (S1.q2[i]+S2.q2[i])/2;
	}

	for (int i = 0; i < 6; i++)
		S_mid.a[i] = (S1.a[i]+S2.a[i])/2;
}
