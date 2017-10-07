#include "../validity_checkers/StateValidityChecker.h"

#include <stdio.h>
#include <stdlib.h>
#include <fstream>

double fRand(double fMin, double fMax)
{
	double f = (double)rand() / RAND_MAX;
	return fMin + f * (fMax - fMin);
}

int main() {

	int Seed = time(NULL);
	srand( Seed );
	cout << "Seed in testing: " << Seed << endl;

	// KDL
	StateValidityChecker svc;

	State a(6), q(12), q1(6), q2(6);
	svc.GDsample(a, q);

	svc.seperate_Vector(q, q1, q2);
	svc.log_q(a, q1, q2);


/*
	int n = 12;
	State a(6), q1(n), q2(n);

	std::ofstream f;
	f.open("/home/avishai/Downloads/omplapp/ompl/Workspace/ckc3d/tests/results/gd_rbs_verification_withObs_env2_distMix.txt", ios::app);

	int N = 2e5, i = 0;
	while (i < N) {
		q1 = svc.sample_q();
		//q2 = svc.sample_q();
		if (q1[0]==-1000)
			continue;

		if (1){//rand()%2==0) {
			double s = fRand(0.01, 1);
			for (int j = 0; j < n; j++)
				q2[j] = q1[j] + s * (fRand(-PI, PI)-q1[j]);

			if (!svc.IKproject(q2))
				continue;
		}
		else {
			q2 = svc.sample_q();
		}

		clock_t begin = clock();
		bool vsuc = svc.checkMotionRBS(q1, q2, 0, 0);
		double rbs_time = double(clock() - begin) / CLOCKS_PER_SEC;

		if (vsuc) {
			//Matrix path;
			//path.push_back(q1);
			//path.push_back(q2);
			//svc.reconstructRBS(q1, q2, path, 0, 1, 1);

			bool path_valid = true;//vfc.verify_path(path);

			f << vsuc << " " << path_valid << " " << svc.normDistance(q1, q2) << " " << rbs_time << endl;
		}
		else
			f << 0 << " " << 0 << " " << svc.normDistance(q1, q2) << " " << rbs_time << endl;
		i++;
	}

	f.close();

	*/

	return 0;
}
