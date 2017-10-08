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

		if (!svc.GDsample(a, q))
			continue;
		//a = {1.45713, -3.37275, -1.8302, -2.04125, -22.1294, -2.98455};
		//q = {0.0376216, 1.3736, -1.12007, -2.46517, 1.64635, 2.3698, -0.147351, -0.618145, 0.802898, 0.853924, -0.591288, 1.62258};
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
			interpolate(S1, 0.5, S2); //(double)rand() / RAND_MAX

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

	return 0;
}

