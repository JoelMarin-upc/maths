#include <iostream>
#include <vector>
#include <Eigen/Dense>
//#include "quaternion.h"
#include "Timer.h"

using namespace std;
using namespace Eigen;

enum MatrixType {
    COMPATIBLE_DETERMINATE,
    COMPATIBLE_INDETERMINATE,
    INCOMPATIBLE
};

const double EPSILON = 1e-12;

static Matrix3d eulerAxisAngleToRotMat(Vector4d& eulerAxisAngle) {
    double angle = eulerAxisAngle.w();
    Vector3d axis(eulerAxisAngle.x(), eulerAxisAngle.y(), eulerAxisAngle.z());
    Matrix3d I = Matrix3d::Identity();
    Matrix3d crossMatrix = vectorToCrossMatrix(axis);
    
    Matrix3d a = cos(angle) * I;
    Matrix3d b = (axis * axis.transpose()) * (1 - cos(angle));
    Matrix3d c = sin(angle) * crossMatrix;
    return a + b + c;
}

static Matrix3d eulerAnglesToRotMat(Vector3d& eulerAngles) {
    double yaw = eulerAngles.x();
    double pitch = eulerAngles.y();
    double roll = eulerAngles.z();
    Matrix3d rotMat;
    rotMat <<
        cos(pitch) * cos(yaw), cos(yaw) * sin(pitch) * sin(roll) - cos(roll) * sin(yaw), cos(roll) * cos(yaw) * sin(pitch) + sin(yaw) * sin(roll),
        cos(pitch) * sin(yaw), sin(yaw) * sin(pitch) * sin(roll) + cos(roll) * cos(yaw), cos(roll) * sin(yaw) * sin(pitch) - cos(yaw) * sin(roll),
        -sin(pitch), cos(pitch) * sin(roll), cos(pitch) * cos(roll);
    return rotMat;
}

static Vector3d rotMatToEulerAngles(Matrix3d& rotMat) {
    double pitch = asin(-rotMat(2, 0));
    double roll = atan2(rotMat(2, 1) / cos(pitch), rotMat(2, 2) / cos(pitch));
    double yaw = atan2(rotMat(1, 0) / cos(pitch), rotMat(0, 0) / cos(pitch));
    return Vector3d(yaw, pitch, roll);
}

static Vector4d rotMatToEulerAxisAngle(Matrix3d& rotMat) {
    double angle = acos((rotMat.trace() - 1) / 2);
    Matrix3d crossMatrix = (rotMat - rotMat.transpose()) / (2 * sin(angle));
    return Vector4d(crossMatrix(2, 1), crossMatrix(0, 2), crossMatrix(1, 0), angle);
}

static Quaterniond eulerAxisAngleToQuaternion(Vector4d& eulerAxisAngle) {
    double realPart = cos(eulerAxisAngle.w() / 2);
    Vector3d axis(eulerAxisAngle.x(), eulerAxisAngle.y(), eulerAxisAngle.z());
    Vector3d imaginaryPart = sin(eulerAxisAngle.w() / 2) * axis;
    return Quaterniond(realPart, imaginaryPart.x(), imaginaryPart.y(), imaginaryPart.z());
}

static Vector4d quaternionToEulerAxisAngle(Quaterniond& quaternion) {
    double angle = 2 * acos(quaternion.w());
    Vector3d quatVector(quaternion.x(), quaternion.y(), quaternion.z());
    Vector3d axis = (1 / sin(angle / 2)) * quatVector;
    return Vector4d(axis.x(), axis.y(), axis.z(), angle);
}

static Vector3d eulerAxisAngleToRotVec(Vector4d& eulerAxisAngle) {
    Vector3d axis(eulerAxisAngle.x(), eulerAxisAngle.y(), eulerAxisAngle.z());
    return eulerAxisAngle.w() * axis;
}

static Vector4d rotVecToEulerAxisAngle(Vector3d& rotVec) {
    double angle = rotVec.norm();
    Vector3d axis = rotVec / rotVec.norm();
    return Vector4d(axis.x(), axis.y(), axis.z(), angle);
}

static Vector3d quaternionRotation(Vector3d& rotationAxis, Quaterniond& rotation) {
    Quaterniond v(0, rotationAxis.x(), rotationAxis.y(), rotationAxis.z());
    Quaterniond conj = rotation.conjugate();
    Quaterniond res1 = quaternionProduct(rotation, v);
    Quaterniond res2 = quaternionProduct(res1, conj);
    return Vector3d(res2.x(), res2.y(), res2.z());
}

static Quaterniond quaternionProduct(Quaterniond& q1, Quaterniond& q2) {
    //return q1 * q2;
    Matrix4d m1;
    m1 << 
        q1.w(), -q1.x(), -q1.y(), -q1.z(),
        q1.x(), q1.w(), -q1.z(), q1.y(),
        q1.y(), q1.z(), q1.w(), -q1.x(),
        q1.z(), -q1.y(), q1.x(), q1.w();
    MatrixXd m2(4, 1);
    m2 << q2.w(), q2.x(), q2.y(), q2.z();
    MatrixXd res = m1 * m2;
    return Quaterniond(res(0, 0), res(1, 0), res(2, 0), res(3, 0));
}

static Matrix3d vectorToCrossMatrix(Vector3d& vector) {
    Matrix3d crossProductMatrix;
    crossProductMatrix <<
        0, -vector(2), vector(1),
        vector(2), 0, -vector(0),
        -vector(1), vector(0), 0;
    return crossProductMatrix;
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

static MatrixType getType(const MatrixXd& matrix) {
    MatrixType type = COMPATIBLE_DETERMINATE;
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
            return INCOMPATIBLE;
        }

        if (fabs(matrix(i, i)) < EPSILON && fabs(matrix(i, lastCol)) < EPSILON) {
            type = COMPATIBLE_INDETERMINATE;
        }
    }

    return type;
}

int main()
{
    Timer timer;
    srand(static_cast<unsigned int>(time(0)));

    
}