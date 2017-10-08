#include "../validity_checkers/StateValidityChecker.h"

#include <stdio.h>
#include <stdlib.h>
#include <fstream>

double fRand(double fMin, double fMax)
{
	double f = (double)rand() / RAND_MAX;
	return fMin + f * (fMax - fMin);
}

void interpolate(StateVector S, double r, StateVector &Si) {

	for (int i = 0; i < S.a.size(); i++) {
		Si.a[i] = S.a[i] + r*(Si.a[i]-S.a[i]);
	}
	for (int i = 0; i < S.q.size(); i++) {
		Si.q[i] = S.q[i] + r*(Si.q[i]-S.q[i]);
	}

}

int main() {

	int Seed = time(NULL);
	srand( Seed );
	cout << "Seed in testing: " << Seed << endl;

	// KDL
	StateValidityChecker svc;

	State a(6), q(12), q1(6), q2(6);
	StateVector S1(6), S2(6);

	bool flag = true;
	while (flag) {

		cout << "Sampling ... \n";

		//if (!svc.GDsample(a, q))
		//	continue;
		a = {1.45713, -3.37275, -1.8302, -2.04125, -22.1294, -2.98455};
		q = {0.0376216, 1.3736, -1.12007, -2.46517, 1.64635, 2.3698, -0.147351, -0.618145, 0.802898, 0.853924, -0.591288, 1.62258};
		S1.copy(a, q);
		S1.print();

		svc.seperate_Vector(q, q1, q2);
		svc.log_q(a, q1, q2);
		cout << "1...\n";
		//cin.ignore();

		while (1) {
			q = svc.rand_q(12);
			for (int i = 0; i < 6; i++)
				a[i] = fRand(-30, 30);
			//if (!svc.GDsample(a, q))
			//	continue;
			S2.copy(a, q);
			interpolate(S1, 0.5, S2);

			if (svc.GDproject(S2.a, S2.q))
				break;
		}
		S2.print();

		svc.seperate_Vector(S2.q, q1, q2);
		svc.log_q(S2.a, q1, q2);
		cout << "2...\n";

		cout << svc.normDistanceStateVector(S1, S2) << endl;
		//cin.ignore();

		flag = !svc.checkMotionRBS(S1, S2, 0);
	}
	cout << "Found RBS.\n";
	S1.print();
	S2.print();

	Matrix path, A;
	path.push_back(S1.q);
	path.push_back(S2.q);
	A.push_back(S1.a);
	A.push_back(S2.a);
	svc.reconstructRBS(S1, S2, path, A, 0, 1, 1);

	svc.log_q(A, path);

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

