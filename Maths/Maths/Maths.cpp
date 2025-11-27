#include <iostream>
#include <vector>
#include <Eigen/Dense>
//#include "quaternion.h"
#include "Timer.h"

using namespace std;
using namespace Eigen;

const double EPSILON = 1e-12;

static Matrix3d eulerAxisAngleToRotMat(Vector4d& eulerAxisAngle) {

}

static Matrix3d eulerAnglesToRotMat(Vector3d& eulerAngles) {

}

static Vector3d rotMatToEulerAngles(Matrix3d& rotMat) {

}

static Vector2d rotMatToEulerAxisAngle(Matrix3d& rotMat) {

}

static Quaterniond eulerAxisAngleToQuaternion(Vector4d& eulerAxisAngle) {
    
}

static Vector4d quaternionToEulerAxisAngle(Quaterniond& quaternion) {

}

static Vector3d eulerAxisAngleToRotVec(Vector4d& eulerAxisAngle) {

}

static Vector4d rotVecToEulerAxisAngle(Vector3d& rotVec) {

}

static Quaterniond quaternionProduct(Quaterniond& q1, Quaterniond& q2) {
    return q1 * q2;
}

static Matrix3d matrixScalarProduct(Matrix3d& m, double scalar) {
    return scalar * m;
}

static Matrix3d matrixSum(Matrix3d& m1, Matrix3d& m2) {
    return m1 + m2;
}

static Matrix3d vectorToCrossProductMatrix(Vector3d& vector) {
    Matrix3d crossProductMatrix;
    crossProductMatrix <<
        0, -vector[2], vector[1],
        vector[2], 0, -vector[0],
        -vector[1], vector[0], 0;
    return crossProductMatrix;
}

static Matrix3d transpose(Matrix3d& matrix) {
    return matrix.transpose();
}

static Matrix3d inverse(Matrix3d& matrix) {
    return matrix.inverse();
}

static Vector3d normalize(Vector3d& vector) {
    return vector / vector.norm();
}

static double norm(Vector3d& vector) {
    return vector.norm();
}

static double trace(Matrix3d& matrix) {
    return matrix.trace();
}

static double determinant(Matrix3d& matrix) {
    return matrix.determinant();
}

static double randomDouble(double min, double max) {
    double random = static_cast<double>(rand()) / RAND_MAX;

    double range = max - min;
    double scaled = random * range + min;

    if (abs(scaled) < EPSILON) return 0;
    if (rand() % 2 == 0) return scaled;
    else return -scaled;
}

static MatrixXd getRandomMatrix(int rows, int columns, double minVal, double maxVal) {
    MatrixXd matrix(rows, columns);
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < columns; j++) {
            matrix(i, j) = randomDouble(minVal, maxVal);
        }
    }
    return matrix;
}

static MatrixXd getMatrix() {
    int rows, columns;

    cout << "Rows: ";
    string r;
    getline(cin, r);
    rows = stoi(r);

    cout << "Columns: ";
    string c;
    getline(cin, c);
    columns = stoi(c);

    system("CLS");

    if (columns <= rows) {
        columns = rows + 1;
        cerr << "Columns added to get an expanded matrix." << endl;
    }

    MatrixXd matrix(rows, columns);
    matrix.setZero();
    cout << matrix << endl;

    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < columns; j++) {
            cout << "Value (" << i << ", " << j << "): ";
            string line;
            getline(cin, line);
            if (line.size() == 0) line = "0";
            double value = stod(line);
            matrix(i, j) = value;
            system("CLS");
            cout << matrix << endl;
        }
    }

    system("CLS");

    return matrix;
}

static string getTime(Timer* timer) {
    long mls = timer->GetMilliseconds();
    long mcs = timer->GetMicroseconds();
    long ns = timer->GetNanoseconds();

    if (mls != 0) return to_string(mls) + " milliseconds";
    else if (mcs != 0) return to_string(mcs) + " microseconds";
    else return to_string(ns) + " nanoseconds";
}

static string getType(const MatrixXd& matrix) {
    string type = "Compatible Determinate";
    int rows = matrix.rows();
    int cols = matrix.cols();
    int lastCol = cols - 1;

    for (int i = 0; i < rows; ++i) {
        bool allZero = true;

        for (int j = 0; j < cols - 1; ++j) {
            if (fabs(matrix(i, j)) > EPSILON) {
                allZero = false;
                break;
            }
        }

        if (allZero && fabs(matrix(i, lastCol)) > EPSILON) {
            return "Incompatible";
        }

        if (fabs(matrix(i, i)) < EPSILON && fabs(matrix(i, lastCol)) < EPSILON) {
            type = "Compatible Indeterminate";
        }
    }

    return type;
}

int main()
{
    Timer timer;
    srand(static_cast<unsigned int>(time(0)));

    
}