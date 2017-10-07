#include "../validity_checkers/StateValidityChecker.h"

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

	Matrix A;
	load_random_nodes(A);

	std::ofstream f;
	f.open("./results/apc_verification.txt", ios::app);

	int N = 1;//0.5e6;
	State a(6), q1(6), q2(6);
	double proj_time = 0;
	bool suc;

	int i = 0;
	while (i < N) {

		for (int j = 0; j < 6; j++) {
			q1[j] = fRand(-3.14, 3.14);
			q2[j] = fRand(-3.14, 3.14);
		}

		int k = rand() % A.size();
		a = A[k];
		//a = {2.17, -4.2, 2, 1.9, 13.6, -2.4};

		for (int tries = 0; tries < 8; tries++) {
			int active_chain = 1;//rand()/RAND_MAX * 2;
			int ik_sol = tries;//rand()/RAND_MAX * 8;
			clock_t begin = clock();
			suc = svc.APCproject(a, q1, q2, active_chain, ik_sol);
			proj_time = double(clock() - begin) / CLOCKS_PER_SEC;
			if (suc) {
				i++;
				svc.log_q(a, q1, q2);
				break;
			}
		}

		f << suc << " " << proj_time << endl;
	}

	f.close();


}

