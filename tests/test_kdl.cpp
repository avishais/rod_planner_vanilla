#include "../validity_checkers/StateValidityChecker.h"
#include "kdl_class.h"

#include <stdio.h>
#include <stdlib.h>
#include <fstream>

double fRand(double fMin, double fMax)
{
	double f = (double)rand() / RAND_MAX;
	return fMin + f * (fMax - fMin);
}

double dist(State p1, State p2) {
	double sum = 0;
	for (int i = 0; i < p1.size(); i++)
		sum += (p1[i]-p2[i])*(p1[i]-p2[i]);

	return sqrt(sum);
}

void load_random_nodes(Matrix &Cdb) {

	std::ifstream File;
	File.open("./results/rod_feasible_confs.txt");

	State a_temp(6);

	int i = 0;
	while(!File.eof()) {
		for (int j=0; j < 6; j++) {
			File >> a_temp[j];
			//cout << c_temp[j] << " ";
		}
		//cout << endl;
		Cdb.push_back(a_temp);
		i++;
	}
	File.close();
	Cdb.pop_back();

}

int main() {

	int Seed = time(NULL);
	srand( Seed );
	cout << "Seed in testing: " << Seed << endl;

	StateValidityChecker svc;
	kdl K(1085.85);

	Matrix A;
	load_random_nodes(A);

//	std::ofstream f;
//	f.open("./results/apc_verification.txt", ios::app);

	int N = 1;//0.5e6;
	State a(6), q(12), q1(6), q2(6);
	double proj_time = 0;
	bool suc;

	int i = 0;
	while (i < N) {

		int k = rand() % A.size();
		a = A[k];

		for (int tries = 0; tries < 8; tries++) {

			for (int j = 0; j < 12; j++) {
				q[j] = fRand(-3.14, 3.14);
			}

			if (!svc.isRodFeasible(a))
				continue;
			Matrix Q = svc.getT(svc.get_Points_on_Rod()-1);

			svc.printVector(q);
			clock_t begin = clock();
			suc = K.GD(q, Q);
			proj_time = double(clock() - begin) / CLOCKS_PER_SEC;

			q = K.get_GD_result();
			svc.printVector(q);

			if (suc) {
				i++;

				for (int j = 0; j < 6; j++) {
					q1[j] = q[j];
					q2[j] = q[j+6];
				}

				svc.log_q(a, q1, q2);
				break;
			}
		}

		//f << suc << " " << proj_time << endl;
	}

	//f.close();
}

