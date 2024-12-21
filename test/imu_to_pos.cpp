#include <iostream>
#include <array>
#include <cmath>
#include <vector>
#include <fstream>
#include <sstream>
#include <string>

// Define a struct for a Quaternion
struct Quaternion {
    double w, x, y, z;

    // Compute the conjugate (inverse for unit quaternion)
    Quaternion conjugate() const {
        return {w, -x, -y, -z};
    }

    // Normalize the quaternion
    void normalize() {
        double norm = std::sqrt(w * w + x * x + y * y + z * z);
        if (norm > 0.0) {
            w /= norm;
            x /= norm;
            y /= norm;
            z /= norm;
        }
    }

    // Convert quaternion to rotation matrix
    std::array<std::array<double, 3>, 3> toRotationMatrix() const {
        std::array<std::array<double, 3>, 3> rotationMatrix;

        rotationMatrix[0][0] = 1 - 2 * (y * y + z * z);
        rotationMatrix[0][1] = 2 * (x * y - z * w);
        rotationMatrix[0][2] = 2 * (x * z + y * w);

        rotationMatrix[1][0] = 2 * (x * y + z * w);
        rotationMatrix[1][1] = 1 - 2 * (x * x + z * z);
        rotationMatrix[1][2] = 2 * (y * z - x * w);

        rotationMatrix[2][0] = 2 * (x * z - y * w);
        rotationMatrix[2][1] = 2 * (y * z + x * w);
        rotationMatrix[2][2] = 1 - 2 * (x * x + y * y);

        return rotationMatrix;
    }

    // 四元数与四元数相乘
    Quaternion operator*(const Quaternion& other) const {
        return {
            w * other.w - x * other.x - y * other.y - z * other.z,
            w * other.x + x * other.w + y * other.z - z * other.y,
            w * other.y - x * other.z + y * other.w + z * other.x,
            w * other.z + x * other.y - y * other.x + z * other.w
        };
    }

    // 将四元数作用在一个向量上 (旋转操作)
    std::array<double, 3> rotateVector(const std::array<double, 3>& vec) const {
        Quaternion vecQuat{0, vec[0], vec[1], vec[2]};
        Quaternion result = (*this * vecQuat) * this->conjugate();
        return {result.x, result.y, result.z};
    }
};

// Transform acceleration using the rotation matrix
std::array<double, 3> transformAcceleration(
    const std::array<double, 3>& acceleration, 
    const std::array<std::array<double, 3>, 3>& rotationMatrix) {
    std::array<double, 3> transformedAcceleration = {0.0, 0.0, 0.0};

    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            transformedAcceleration[i] += rotationMatrix[i][j] * acceleration[j];
        }
    }

    return transformedAcceleration;
}

// 計算總位移的函數
double calculateDisplacement(double x, double y, double z) {
    return std::sqrt(x * x + y * y + z * z);
}


int main() {
    // Read quaternion and acceleration from CSV file
    std::ifstream inputFile("output.csv");
    if (!inputFile.is_open()) {
        std::cerr << "Failed to open file" << std::endl;
        return 1;
    }

    std::string line;
    double timeStep = 0.01; // Time step in seconds
    std::array<double, 3> cumulativeDisplacement = {0.0, 0.0, 0.0};
    std::array<double, 3> velocity = {0.0, 0.0, 0.0};

    while (std::getline(inputFile, line)) {
        std::istringstream iss(line);
        Quaternion q;

        // Body Frame 下测得的加速度
        std::array<double, 3> accel;

        char delimiter;
        if (iss >> q.w >> delimiter >> q.x >> delimiter >> q.y >> delimiter >> q.z >> delimiter
                >> accel[0] >> delimiter >> accel[1] >> delimiter >> accel[2]) {
            // Normalize quaternion
            q.normalize();
            //Quaternion qConjugate = q.conjugate();
            //auto rotationMatrix = qConjugate.toRotationMatrix();
            //auto transformedAccel = transformAcceleration(accel, rotationMatrix);

            // 转换到 Global Frame
            std::array<double, 3> transformedAccel = q.conjugate().rotateVector(accel);

            // Calculate step displacement and update cumulative displacement
            for (int i = 0; i < 3; ++i) {
                velocity[i] += transformedAccel[i] * timeStep;
                double stepDisplacement = velocity[i]*timeStep + (transformedAccel[i] * timeStep * timeStep) / 2.0;
                cumulativeDisplacement[i] += stepDisplacement;
            }
        }
    }

    inputFile.close();

    double total_Displacement = calculateDisplacement(cumulativeDisplacement[0],cumulativeDisplacement[1],cumulativeDisplacement[2]);

    // Output cumulative displacement
    std::cout << "Cumulative Displacement: ["
              << cumulativeDisplacement[0] << ", "
              << cumulativeDisplacement[1] << ", "
              << cumulativeDisplacement[2] << "]" << std::endl;

    std::cout << "Total Displacement: "
              << total_Displacement << std::endl;

    return 0;
}
