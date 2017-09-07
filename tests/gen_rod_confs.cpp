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

int main() {

	int Seed = time(NULL);
	srand( Seed );
	cout << "Seed in testing: " << Seed << endl;

	StateValidityChecker svc;

	std::ofstream f;
	f.open("/home/avishai/Downloads/omplapp/ompl/Workspace/rod_planning/tests/results/rod_feasible_confs.txt", ios::app);

	int N = 0.5e6;
	State a(6);
	double proj_time = 0;
	bool suc;

	int i = 0;
	while (i < N) {
		for (int j = 0; j < 6; j++)
			a[j] = fRand(-30, 30);

		if (!svc.isRodFeasible(a))
			continue;

		svc.printVector(a);
		for (int j = 0; j < 6; j++)
			f << a[j] << " ";
		f << endl;

		i++;
	}

	f.close();


}

