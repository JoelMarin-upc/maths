#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include "matplotlibcpp.h"
#include "Timer.h"

using namespace std;
using namespace Eigen;
namespace plt = matplotlibcpp;

enum MatrixType {
    COMPATIBLE_DETERMINATE,
    COMPATIBLE_INDETERMINATE,
    INCOMPATIBLE
};

const double EPSILON = 1e-12;
const double PI = 3.14159265359;

static double randomDouble(double min, double max) {
    double random = static_cast<double>(rand()) / RAND_MAX;

    double range = max - min;
    double scaled = random * range + min;

    if (abs(scaled) < EPSILON) return 0;
    if (rand() % 2 == 0) return scaled;
    else return -scaled;
}

static Quaterniond QuaternionProduct(Quaterniond& q1, Quaterniond& q2) {
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

static Matrix3d VectorToCrossMatrix(Vector3d& vector) {
    Matrix3d crossProductMatrix;
    crossProductMatrix <<
        0, -vector(2), vector(1),
        vector(2), 0, -vector(0),
        -vector(1), vector(0), 0;
    return crossProductMatrix;
}

static Matrix3d EulerAxisAngleToRotMat(Vector4d& eulerAxisAngle) {
    double angle = eulerAxisAngle.w();
    Vector3d axis(eulerAxisAngle.x(), eulerAxisAngle.y(), eulerAxisAngle.z());
    Matrix3d I = Matrix3d::Identity();
    Matrix3d crossMatrix = VectorToCrossMatrix(axis);
    
    Matrix3d a = cos(angle) * I;
    Matrix3d b = (axis * axis.transpose()) * (1 - cos(angle));
    Matrix3d c = sin(angle) * crossMatrix;
    return a + b + c;
}

static Matrix3d EulerAnglesToRotMat(Vector3d& eulerAngles) {
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

static Vector3d RotMatToEulerAngles(Matrix3d& rotMat) {
    double pitch = asin(-rotMat(2, 0));
    double roll = atan2(rotMat(2, 1) / cos(pitch), rotMat(2, 2) / cos(pitch));
    double yaw = atan2(rotMat(1, 0) / cos(pitch), rotMat(0, 0) / cos(pitch));
    return Vector3d(yaw, pitch, roll);
}

static Vector4d RotMatToEulerAxisAngle(Matrix3d& rotMat) {
    double angle = acos((rotMat.trace() - 1) / 2);
    Matrix3d crossMatrix = (rotMat - rotMat.transpose()) / (2 * sin(angle));
    return Vector4d(crossMatrix(2, 1), crossMatrix(0, 2), crossMatrix(1, 0), angle);
}

static Quaterniond EulerAxisAngleToQuaternion(Vector4d& eulerAxisAngle) {
    double realPart = cos(eulerAxisAngle.w() / 2);
    Vector3d axis(eulerAxisAngle.x(), eulerAxisAngle.y(), eulerAxisAngle.z());
    Vector3d imaginaryPart = sin(eulerAxisAngle.w() / 2) * axis;
    return Quaterniond(realPart, imaginaryPart.x(), imaginaryPart.y(), imaginaryPart.z());
}

static Vector4d QuaternionToEulerAxisAngle(Quaterniond& quaternion) {
    double angle = 2 * acos(quaternion.w());
    Vector3d quatVector(quaternion.x(), quaternion.y(), quaternion.z());
    Vector3d axis = (1 / sin(angle / 2)) * quatVector;
    return Vector4d(axis.x(), axis.y(), axis.z(), angle);
}

static Vector3d EulerAxisAngleToRotVec(Vector4d& eulerAxisAngle) {
    Vector3d axis(eulerAxisAngle.x(), eulerAxisAngle.y(), eulerAxisAngle.z());
    return eulerAxisAngle.w() * axis;
}

static Vector4d RotVecToEulerAxisAngle(Vector3d& rotVec) {
    double angle = rotVec.norm();
    Vector3d axis = rotVec / rotVec.norm();
    return Vector4d(axis.x(), axis.y(), axis.z(), angle);
}

static Vector3d QuaternionRotation(Vector3d& rotationAxis, Quaterniond& rotation) {
    Quaterniond v(0, rotationAxis.x(), rotationAxis.y(), rotationAxis.z());
    Quaterniond conj = rotation.conjugate();
    Quaterniond res1 = QuaternionProduct(rotation, v);
    Quaterniond res2 = QuaternionProduct(res1, conj);
    return Vector3d(res2.x(), res2.y(), res2.z());
}

struct RotationSet {
    Eigen::Matrix3d rotationMatrix;
    Eigen::Vector3d eulerAngles;
    Eigen::Vector4d eulerAxisAngle;
    Eigen::Quaterniond quaternion;
    Eigen::Vector3d rotationVector;
};

RotationSet ConvertRotation(
    Eigen::Matrix3d* rotMat = nullptr,
    Eigen::Vector3d* eulerAngles = nullptr,
    Eigen::Vector4d* eulerAxisAngle = nullptr,
    Eigen::Quaterniond* quat = nullptr,
    Eigen::Vector3d* rotVec = nullptr
) {
    RotationSet result;

    if (rotMat) {
        result.rotationMatrix = *rotMat;
    }
    else if (eulerAngles) {
        result.rotationMatrix = EulerAnglesToRotMat(*eulerAngles);
    }
    else if (eulerAxisAngle) {
        result.rotationMatrix = EulerAxisAngleToRotMat(*eulerAxisAngle);
    }
    else if (quat) {
        result.rotationMatrix = quat->toRotationMatrix();
    }
    else if (rotVec) {
        Vector4d axisAnlge = RotVecToEulerAxisAngle(*rotVec);
        result.rotationMatrix = EulerAxisAngleToRotMat(axisAnlge);
    }
    else {
        throw std::runtime_error("No rotation input provided");
    }

    result.eulerAngles = RotMatToEulerAngles(result.rotationMatrix);

    result.eulerAxisAngle = RotMatToEulerAxisAngle(result.rotationMatrix);

    result.quaternion = EulerAxisAngleToQuaternion(result.eulerAxisAngle);

    result.rotationVector = EulerAxisAngleToRotVec(result.eulerAxisAngle);

    return result;
}

static vector<Vector2d> PlotValues() {
    vector<Vector2d> plottedValues;
    for (int i = 0; i < 100; i++) {
        double angle = 6 * PI / 100 * i;
        Vector3d axis = Vector3d(randomDouble(-10, 10), randomDouble(-10, 10), randomDouble(-10, 10));
        axis.normalize();
        Vector4d eulerAxisAngle = Vector4d(axis.x(), axis.y(), axis.z(), angle);
        Matrix3d rotationMatrix = EulerAxisAngleToRotMat(eulerAxisAngle);
        Vector2d plottedValue = Vector2d(angle, rotationMatrix.trace());
        plottedValues.push_back(plottedValue);
    }
    return plottedValues;
}

static void DrawValues(vector<Vector2d> plottedValues) {
    vector<double> x;
    vector<double> y;
    for (int i = 0; i < plottedValues.size(); i++)
    {
        x.push_back(plottedValues[i].x());
        y.push_back(plottedValues[i].y());
    }
    plt::plot(x, y);
    plt::show();
    //plt::detail::_interpreter::kill();
}

static bool IsRotationMatrix(Matrix3d matrix) {
    bool isRotationMatrix = false;

    isRotationMatrix = matrix.determinant() - 1 < EPSILON;
    isRotationMatrix = ((matrix.transpose() - matrix.inverse()).array().abs() < EPSILON).all();
    return isRotationMatrix;
}

static Vector4d GetEulerAxisAngle() {
    system("CLS");

    Vector4d vec;
    vec.setZero();
    cout << vec << endl;

    vector<string> components = { "X", "Y", "Z", "Angle"};
    for (int i = 0; i < 4; i++) {
        cout << components[i] << ": ";
        string line;
        getline(cin, line);
        if (line.size() == 0) line = "0";
        double value = stod(line);
        vec(i) = value;
        system("CLS");
        cout << vec << endl;
    }

    system("CLS");

    Vector3d normalizedAxis = Vector3d(vec.x(), vec.y(), vec.z());
    normalizedAxis.normalize();

    vec = Vector4d(normalizedAxis.x(), normalizedAxis.y(), normalizedAxis.z(), vec.w());

    return vec;
}

static Vector3d GetEulerAngles() {
    system("CLS");

    Vector3d vec;
    vec.setZero();
    cout << vec << endl;

    vector<string> components = { "Yaw", "Pitch", "Roll" };
    for (int i = 0; i < 3; i++) {
        cout << components[i] << ": ";
        string line;
        getline(cin, line);
        if (line.size() == 0) line = "0";
        double value = stod(line);
        vec(i) = value;
        system("CLS");
        cout << vec << endl;
    }

    system("CLS");

    return vec;
}

static Matrix3d GetRotationMatrix() {
    system("CLS");

    Matrix3d matrix;
    matrix.setZero();
    cout << matrix << endl;

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
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

    Eigen::JacobiSVD<Eigen::Matrix3d> svd(matrix, Eigen::ComputeFullU | Eigen::ComputeFullV);
    matrix = svd.matrixU() * svd.matrixV().transpose();

    return matrix;
}

static Vector3d GetRotationVector() {
    system("CLS");

    Vector3d vec;
    vec.setZero();
    cout << vec << endl;

    vector<string> components = { "X", "Y", "Z" };
    for (int i = 0; i < 3; i++) {
        cout << components[i] << ": ";
        string line;
        getline(cin, line);
        if (line.size() == 0) line = "0";
        double value = stod(line);
        vec(i) = value;
        system("CLS");
        cout << vec << endl;
    }

    system("CLS");

    return vec;
}

static Quaterniond GetQuaternion() {
    system("CLS");

    Vector4d vec;
    vec.setZero();
    cout << vec << endl;

    vector<string> components = { "q0", "q1", "q2", "q3"};
    for (int i = 0; i < 4; i++) {
        cout << components[i] << ": ";
        string line;
        getline(cin, line);
        if (line.size() == 0) line = "0";
        double value = stod(line);
        vec(i) = value;
        system("CLS");
        cout << vec << endl;
    }

    system("CLS");

    return Quaterniond(vec[0], vec[1], vec[2], vec[3]);
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
    srand(static_cast<unsigned int>(time(0)));

    cout << "[1] Euler Axis Angle -> Rotation Matrix" << endl;
    cout << "[2] Euler Angles -> Rotation Matrix" << endl;
    cout << "[3] Rotation Matrix -> Euler Angles" << endl;
    cout << "[4] Rotation Matrix -> Euler Axis Angle" << endl;
    cout << "[5] Euler Axis Angle -> Quaternion" << endl;
    cout << "[6] Quaternion -> Euler Axis Angle" << endl;
    cout << "[7] Euler Axis Angle -> Rotation Vector" << endl;
    cout << "[8] Rotation Vector -> Euler Axis Angle" << endl;
    cout << "[9] Quaternion Rotation" << endl;
    cout << "[10] Plot Matrices (angle: 0->6pi)" << endl;
    cout << "[11] Convert rotation" << endl;
    cout << "Choose an option: ";
    string input;
    getline(cin, input);
    cout << endl << endl;

    if (input == "1") {
        auto v = GetEulerAxisAngle();
        auto m = EulerAxisAngleToRotMat(v);
        cout << m << endl;
        if (IsRotationMatrix(m)) cout << "The matrix is a rotation: det(A) = 1, A(transposed) = A(inverse)" << endl;
    }
    else if (input == "2") {
        auto v = GetEulerAngles();
        auto m = EulerAnglesToRotMat(v);
        cout << m << endl;
        if (IsRotationMatrix(m)) cout << "The matrix is a rotation: det(A) = 1, A(transposed) = A(inverse)" << endl;
    }
    else if (input == "3") {
        auto m = GetRotationMatrix();
        auto v = RotMatToEulerAngles(m);
        cout << v << endl;
    }
    else if (input == "4") {
        auto m = GetRotationMatrix();
        auto v = RotMatToEulerAxisAngle(m);
        cout << v << endl;
    }
    else if (input == "5") {
        auto v = GetEulerAxisAngle();
        auto q = EulerAxisAngleToQuaternion(v);
        cout << q.coeffs() << endl;
    }
    else if (input == "6") {
        auto m = GetQuaternion();
        auto v = QuaternionToEulerAxisAngle(m);
        cout << v << endl;
    }
    else if (input == "7") {
        auto v1 = GetEulerAxisAngle();
        auto v2 = EulerAxisAngleToRotVec(v1);
        cout << v2 << endl;
    }
    else if (input == "8") {
        auto v1 = GetRotationVector();
        auto v2 = RotVecToEulerAxisAngle(v1);
        cout << v2 << endl;
    }
    else if (input == "9") {
        auto v1 = GetRotationVector();
        auto q = GetQuaternion();
        auto v2 = QuaternionRotation(v1, q);
        cout << v2 << endl;
    }
    else if (input == "10") {
        auto vals = PlotValues();
        DrawValues(vals);
    }
    else if (input == "11") {
        cout << "[1] Rotation matrix" << endl;
        cout << "[2] Euler Angles" << endl;
        cout << "[3] Euler Axis Angle" << endl;
        cout << "[4] Quaternion" << endl;
        cout << "[5] Rotation Vector" << endl;
        cout << "Chose an input option: ";
        string input;
        getline(cin, input);
        cout << endl << endl;

        RotationSet allRotations;

        if (input == "1") {
            auto m = GetRotationMatrix();
            allRotations = ConvertRotation(&m);
        }
        else if (input == "2") {
            auto v = GetEulerAngles();
            allRotations = ConvertRotation(nullptr, &v);
        }
        else if (input == "3") {
            auto v = GetEulerAxisAngle();
            allRotations = ConvertRotation(nullptr, nullptr, &v);
        }
        else if (input == "4") {
            auto q = GetQuaternion();
            allRotations = ConvertRotation(nullptr, nullptr, nullptr, &q);
        }
        else if (input == "5") {
            auto v = GetRotationVector();
            allRotations = ConvertRotation(nullptr, nullptr, nullptr, nullptr, &v);
        }
        else cout << "Not a valid option.";

        std::cout << "Rotation Matrix:\n" << allRotations.rotationMatrix << "\n\n";
        std::cout << "Euler Angles:\n" << allRotations.eulerAngles << "\n\n";
        std::cout << "Euler Axis Angle:\n" << allRotations.eulerAxisAngle << "\n\n";
        std::cout << "Quaternion:\n" << allRotations.quaternion.coeffs() << "\n\n";
        std::cout << "Rotation Vector:\n" << allRotations.rotationVector << "\n\n";
    }
    else cout << "Not a valid option.";
}