/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Rice University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Rice University nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Ioan Sucan, Avishai Sintov */

#include "plan.h"

bool isStateValid(const ob::State *state)
{
	return true;
}

ob::PlannerPtr plan_C::allocatePlanner(ob::SpaceInformationPtr si, plannerType p_type)
{
    switch (p_type)
    {
        case PLANNER_CBIRRT:
        {
            return std::make_shared<og::CBiRRT>(si, maxStep);
            break;
        }
        // case PLANNER_RRT:
        // {
        //     return std::make_shared<og::RRT>(si, maxStep);
        //     break;
        // }
        // /*case PLANNER_LAZYRRT:
        // {
        //     return std::make_shared<og::LazyRRT>(si, maxStep);
        //     break;
        // }*/
        // case PLANNER_PRM:
        // {
        //     return std::make_shared<og::PRM>(si);
        //     break;
        // }
        // case PLANNER_SBL:
        // {
        //     return std::make_shared<og::SBL>(si, maxStep);
        //     break;
        // }
        default:
        {
            OMPL_ERROR("Planner-type enum is not implemented in allocation function.");
            return ob::PlannerPtr(); // Address compiler warning re: no return value.
            break;
        }
    }
}

void plan_C::plan(State c_start, State c_goal, double runtime, plannerType ptype, double max_step) {

	// construct the state space we are planning inz
	ob::CompoundStateSpace *cs = new ob::CompoundStateSpace(); // Compound R^12 configuration space
	ob::StateSpacePtr A(new ob::RealVectorStateSpace(6)); // A-space - state space of the rod - R^6
	ob::StateSpacePtr Q(new ob::RealVectorStateSpace(12)); // Angles of Robot 1 & 2 - R^12
	cs->addSubspace(A, 1.0);
	cs->addSubspace(Q, 1.0);

	// set the bounds for the A=R^6
	ob::RealVectorBounds Abounds(6);
	Abounds.setLow(-30);
	Abounds.setHigh(30);

	// set the bounds for the Q=R^12 part of 'Cspace'
	ob::RealVectorBounds Qbounds(12);
	Qbounds.setLow(0, -2.88); // Robot 1
	Qbounds.setHigh(0, 2.88);
	Qbounds.setLow(1, -1.919);
	Qbounds.setHigh(1, 1.919);
	Qbounds.setLow(2, -1.919);
	Qbounds.setHigh(2, 1.22);
	Qbounds.setLow(3, -2.79);
	Qbounds.setHigh(3, 2.79);
	Qbounds.setLow(4, -2.09);
	Qbounds.setHigh(4, 2.09);
	Qbounds.setLow(5, -PI);// -6.98); // Should be -6.98 but currently the IK won't allow it - this impacts the sampler
	Qbounds.setHigh(5, PI);// 6.98); // Should be 6.98 but currently the IK won't allow it
	Qbounds.setLow(6, -2.88); // Robot 2
	Qbounds.setHigh(6, 2.88);
	Qbounds.setLow(7, -1.919);
	Qbounds.setHigh(7, 1.919);
	Qbounds.setLow(8, -1.919);
	Qbounds.setHigh(8, 1.22);
	Qbounds.setLow(9, -2.79);
	Qbounds.setHigh(9, 2.79);
	Qbounds.setLow(10, -2.09);
	Qbounds.setHigh(10, 2.09);
	Qbounds.setLow(11, -PI);// -6.98); // Should be -6.98 but currently the IK won't allow it
	Qbounds.setHigh(11, PI);// 6.98); // Should be 6.98 but currently the IK won't allow it

	// set the bound for the compound space
	cs->as<ob::RealVectorStateSpace>(0)->setBounds(Abounds);
	cs->as<ob::RealVectorStateSpace>(1)->setBounds(Qbounds);

	// construct a compound state space using the overloaded operator+
	ob::StateSpacePtr Cspace(cs);

	// construct an instance of  space information from this state space
	ob::SpaceInformationPtr si(new ob::SpaceInformation(Cspace));

	// set state validity checking for this space
	//si->setStateValidityChecker(ob::StateValidityCheckerPtr(new myStateValidityCheckerClass(si)));
	si->setStateValidityChecker(std::bind(&isStateValid, std::placeholders::_1));
	si->setStateValidityCheckingResolution(0.1); // 3% ???

	// create start state
	ob::ScopedState<ob::CompoundStateSpace> start(Cspace);
	for (int i = 0; i < 6; i++)
			start->as<ob::RealVectorStateSpace::StateType>(0)->values[i] = c_start[i]; // Access the first component of the start a-state
	for (int i = 0; i < 12; i++)
		start->as<ob::RealVectorStateSpace::StateType>(1)->values[i] = c_start[i+6];

	// create goal state
	ob::ScopedState<ob::CompoundStateSpace> goal(Cspace);
	for (int i = 0; i < 6; i++)
		goal->as<ob::RealVectorStateSpace::StateType>(0)->values[i] = c_goal[i]; // Access the first component of the goal a-state
	for (int i = 0; i < 12; i++)
		goal->as<ob::RealVectorStateSpace::StateType>(1)->values[i] = c_goal[i+6];

	// create a problem instance
	ob::ProblemDefinitionPtr pdef(new ob::ProblemDefinition(si));

	// set the start and goal states
	pdef->setStartAndGoalStates(start, goal);
	pdef->print();

// 	// Register new projection evaluator
// 	if (ptype == PLANNER_SBL) {
// 		//Cspace->registerProjection("myProjection", ob::ProjectionEvaluatorPtr(new MyProjection(Cspace)));
// 		Cspace->registerDefaultProjection(ob::ProjectionEvaluatorPtr(new MyProjection(Cspace)));
// 	}

	maxStep = max_step;
	// create a planner for the defined space
	// To add a planner, the #include library must be added above
	ob::PlannerPtr planner = allocatePlanner(si, ptype);

	//planner->as<og::SBL>()->setProjectionEvaluator("myProjection");

	// set the problem we are trying to solve for the planner
	planner->setProblemDefinition(pdef);

	// perform setup steps for the planner
	planner->setup();

	//planner->printSettings(std::cout); // Prints some parameters such as range
	//planner->printProperties(std::cout); // Prints some decisions such as multithreading, display approx solutions, and optimize?

	// print the settings for this space
	//si->printSettings(std::cout); // Prints state space settings such as check resolution, segmant count factor and bounds
	//si->printProperties(std::cout); // Prints state space properties, average length, dimension ...

	// print the problem settings
	//pdef->print(std::cout); // Prints problem definition such as start and goal states and optimization objective

	// attempt to solve the problem within one second of planning time
	clock_t begin = clock();
	ob::PlannerStatus solved = planner->solve(runtime);
	clock_t end = clock();
	cout << "Runtime: " << double(end - begin) / CLOCKS_PER_SEC << endl;

	if (solved) {
		// get the goal representation from the problem definition (not the same as the goal state)
		// and inquire about the found path
		//ob::PathPtr path = pdef->getSolutionPath();

		// print the path to screen
		//path->print(std::cout);  // Print as vectors

		// Save path to file
		//std::ofstream myfile;
		//myfile.open("pathGD.txt");
		//og::PathGeometric& pog = static_cast<og::PathGeometric&>(*path); // Transform into geometric path class
		//pog.printAsMatrix(myfile); // Print as matrix to file
		//myfile.close();

		std::cout << "Found solution:" << std::endl;
		solved_bool = true;
	}
	else {
		std::cout << "No solutions found" << std::endl;
		solved_bool = false;
	}
}

void load_random_nodes(Matrix &Cdb) {
	cout << "Getting random nodes." << endl;

	std::ifstream File;
	File.open("/home/avishai/Documents/workspace/rod_planner_vanilla/validity_checkers/randomnodes.txt");
	//File.open("randomnodes.txt");

	State c_temp(18);

	int i = 0;
	while(!File.eof()) {
		for (int j=0; j < 18; j++) {
			File >> c_temp[j];
			// cout << c_temp[j] << " ";
		}
		// cout << endl;
		Cdb.push_back(c_temp);
		i++;
		// cout << i << endl;
	}
	File.close();
	Cdb.pop_back();
}

bool extract_from_perf_file(ofstream &ToFile) {
	ifstream FromFile;
	FromFile.open("./paths/perf_log.txt");

	bool success = false;
	string line;
	int j = 1;
	while (getline(FromFile, line)) {
		ToFile << line << "\t";
		if (j==1 && stoi(line))
			success = true;
		j++;
	}

	FromFile.close();

	return success;
}

int main(int argn, char ** args) {
	std::cout << "OMPL version: " << OMPL_VERSION << std::endl;
	double runtime; // Maximum allowed runtime
	plannerType ptype; // Planner type
	string plannerName;
	int env; // Tested environment index

	// Input ./<exefile> <runtime> <planner> <environment>

	if (argn == 1) {
		runtime = 1; // sec
		ptype = PLANNER_CBIRRT;
		plannerName = "CBiRRT";
		env = 1;
	}
	else if (argn == 2) {
		runtime = atof(args[1]);
		ptype = PLANNER_CBIRRT;
		plannerName = "CBiRRT";
		env = 1;
	}
	else if (argn > 2) {
		runtime = atof(args[1]);
		switch (atoi(args[2])) {
		case 1 :
			ptype = PLANNER_CBIRRT;
			plannerName = "CBiRRT";
			break;
		case 2 :
			ptype = PLANNER_RRT;
			plannerName = "RRT";
			break;
		case 3 :
			ptype = PLANNER_LAZYRRT;
			plannerName = "LazyRRT";
			break;
		case 4 :
			ptype = PLANNER_PRM;
			plannerName = "PRM";
			break;
		case 5 :
			ptype = PLANNER_SBL;
			plannerName = "SBL";
			break;
		default :
			cout << "Error: Requested planner not defined.";
			exit(1);
		}
		if (argn == 4)
			env = atoi(args[3]);
		else
			env = 1;
	}

	plan_C Plan;

	// srand (time(NULL));

	State c_start, c_goal;
	if (env == 1) {
		c_start = {1.13317, -4.08401, 2.74606, 6.786018, 11.63367, -5.103594, -0.209439510239320, 0.122173047639603,	0.174532925199433, 1.30899693899575, 0.261799387799149, 0.698131700797732, -0.106584015572764, 1.06335198985049, 0.282882132165777, -0.115210802424076, -1.95829181139617, -1.35961844319303};
		c_goal = {1.8708,-1.3245,2.944,3.7388,6.5021,-0.01924,0.4,0.3,1,-0.1,-0.57118,-0.4,-0.84385,0.73392,0.2169,0.52291,-1.1915,2.6346};
	}
	else if (env == 2) {

	}

	int mode = 1;
	switch (mode) {
	case 1: {
		srand (time(NULL));

		Matrix Cdb;
		load_random_nodes(Cdb);

		cout << Cdb.size() << endl;

		std::ofstream ft;
		ft.open("timesC.txt", ios::app);

		int p1, p2;
		for (int j = 1; j < 100; j++) {
			// do {
			// 	p1 = rand() % Cdb.size();
			// 	p2 = rand() % Cdb.size();

			// } while (p1==p2);
			p1 = j;
			p2 = j + 1;

			cout << p1 << " " << p2 << endl;

			runtime = 200.;
			Plan.plan(Cdb[p1], Cdb[p2], runtime, PLANNER_CBIRRT, 2.6);

			// Log
			ft << p1 << "\t" << p2 << "\t";
			bool suc = extract_from_perf_file(ft);
			ft << endl;

			if (suc) {
				cout << "Found solution for random query.\n";
			}
		}
		ft.close();
		break;

		// Plan.plan(c_start, c_goal, runtime, ptype, 2.6);

		break;
	}
	case 2 : { // Benchmark planning time with constant maximum step size
		ofstream GD;
		GD.open("./matlab/Benchmark_" + plannerName + "_env1_temp.txt", ios::app);

		for (int k = 0; k < 10; k++) {
			//Plan.plan(c_start, c_goal, runtime, ptype, 2.6); // CBiRRT
			Plan.plan(c_start, c_goal, runtime, ptype, 1.7); // SBL & RRT

			// Extract from perf file
			ifstream FromFile;
			FromFile.open("./paths/perf_log.txt");
			string line;
			while (getline(FromFile, line))
				GD << line << "\t";
			FromFile.close();
			GD << endl;
		}
		GD.close();
		break;
	}
	case 3 : { // Benchmark maximum step size
		ofstream F;
		if (env == 1)
			F.open("./matlab/Benchmark_" + plannerName + "_env1_rB.txt", ios::app);
		else if (env == 2)
			F.open("./matlab/Benchmark_" + plannerName + "_env2_rB.txt", ios::app);

		//State v = {0.8, 1.7, 2.6};
		for (int k = 0; k < 100; k++) {

			for (int j = 0; j < 11; j++) {
				double maxStep = 0.2 + 0.3*j;

				cout << "** Running Rod planning, iteration " << k+1 << " with maximum step: " << maxStep << " **" << endl;

				Plan.plan(c_start, c_goal, runtime, ptype, maxStep);

				F << maxStep << "\t";

				// Extract from perf file
				ifstream FromFile;
				FromFile.open("./paths/perf_log.txt");
				string line;
				while (getline(FromFile, line))
					F << line << "\t";
				FromFile.close();
				F << endl;
			}
		}
		F.close();
		break;
	}
	}

	std::cout << std::endl << std::endl;

	return 0;
}

