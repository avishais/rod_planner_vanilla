#include "StateValidityChecker.h"
#include <iostream>
#include <fstream>

// Qbounds.setLow(0, -2.88); // Robot 1
// 	Qbounds.setHigh(0, 2.88);
// 	Qbounds.setLow(1, -1.919);
// 	Qbounds.setHigh(1, 1.919);
// 	Qbounds.setLow(2, -1.919);
// 	Qbounds.setHigh(2, 1.22);
// 	Qbounds.setLow(3, -2.79);
// 	Qbounds.setHigh(3, 2.79);
// 	Qbounds.setLow(4, -2.09);
// 	Qbounds.setHigh(4, 2.09);
// 	Qbounds.setLow(5, -PI);// -6.98); // Should be -6.98 but currently the IK won't allow it - this impacts the sampler
// 	Qbounds.setHigh(5, PI);// 6.98); // Should be 6.98 but currently the IK won't allow it

int main() {

	StateValidityChecker SVC;
	State q = {0,0,0,0,0,0,0,0,0,0,0,0};
    State a = {0,0,0,0,0,0};

    std::ofstream File;
	File.open("randomnodes.txt");
    
    int k = 0;
    while (k < 995) {
        bool b = SVC.GDsample(a, q);
        
        if (b) {
            for (int j=0; j < 6; j++) {
                cout << a[j] << " ";
                File << a[j] << " ";
            }
            for (int j=0; j < 12; j++) {
                cout << q[j] << " ";
                File << q[j] << " ";
            }
            cout << "\n";
            File << "\n";

            k++;
            cout << k << endl;
        }
    }

    File.close();

	return 0;
}