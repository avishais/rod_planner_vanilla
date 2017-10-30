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

	int N = 10000;
	State a(6);
	double proj_time_valid = 0, proj_time_notvalid = 0;
	int proj_count_valid = 0, proj_count_notvalid = 0;
	bool suc;

	while (proj_count_valid < 100) {
		for (int j = 0; j < 6; j++)
			a[j] = fRand(-30, 30);

		clock_t sT = clock();
		bool valid = svc.isRodFeasible(a);
		double time = double(clock() - sT) / CLOCKS_PER_SEC;

		if (valid) {
			proj_time_valid += time;
			proj_count_valid++;
			cout << proj_count_valid << endl;
		}
		else {
			proj_time_notvalid += time;
			proj_count_notvalid++;
		}
	}

	cout << "Avg. ODE time - valid: " << proj_time_valid / proj_count_valid << " sec, count: " << proj_count_valid << endl;
	cout << "Avg. ODE time - not valid: " << proj_time_notvalid / proj_count_notvalid << " sec, count: " << proj_count_notvalid << endl;

}

