#include<iostream>

using namespace std;

int inverse() {
    //int m[3][3]; //3x3 matrix
    float d = 0;

    double m[3][3] = { {2.1, 0.0, 0.0},
                            {0.0, 2.1, 0.0},
                            {0.0, 0.0, 2.001} };


    //finding determinant of the matrix
    for (int i = 0; i < 3; i++)
        d = d + (m[0][i] * (m[1][(i + 1) % 3] * m[2][(i + 2) % 3] - m[1][(i + 2) % 3] * m[2][(i + 1) % 3]));

    if (d > 0)         //Condition to check if the derterminat is zero or not if zero than inverse dont exists
    {
        cout << "\nInverse of the matrix is: \n";
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++)
                std::cout << ((m[(j + 1) % 3][(i + 1) % 3] * m[(j + 2) % 3][(i + 2) % 3]) - (m[(j + 1) % 3][(i + 2) % 3] * m[(j + 2) % 3][(i + 1) % 3])) / d << "\t"; //finding adjoint and dividing it by determinant
            std::cout << "\n";
        }
    }
    else std::cout << "Inverse does'nt exist for this matrix";
    return 0;
}
