#include "RobotKinematics.h"

RobotKinematics::RobotKinematics(QObject *parent) : QObject(parent)
{
    // Инициализация DH-параметров из таблицы (a, d, alpha)
    dhParams = {
        {0.0, 0.213, M_PI_2},     // Joint 0
        {-0.8, 0.193, 0.0},       // Joint 1
        {-0.59, -0.16, 0.0},      // Joint 2
        {0.0, 0.25, M_PI_2},      // Joint 3
        {0.0, 0.28, -M_PI_2},     // Joint 4
        {0.0, 0.25, 0.0}          // Joint 5
    };
}

QVector3D RobotKinematics::calculatePosition(const QList<double> &jointAnglesDeg) {
    if (jointAnglesDeg.size() != 6) { // Проверка количества углов
        qWarning() << "Expected 6 joint angles!";
        return QVector3D(0, 0, 0);
    }
#ifdef USE_AVX
    alignas(32) Matrix4x4 T = {{
        {1, 0, 0, 0},
        {0, 1, 0, 0},
        {0, 0, 1, 0},
        {0, 0, 0, 1}
    }};
#else
    Matrix4x4 T = {{
        {1, 0, 0, 0},
        {0, 1, 0, 0},
        {0, 0, 1, 0},
        {0, 0, 0, 1}
    }};
#endif

    for (int i = 0; i < 6; ++i) {
        double theta = degToRad(jointAnglesDeg[i]);
        const DHParams &p = dhParams[i];
#ifdef USE_AVX
        alignas(32) Matrix4x4 Ti = {{
            {cos(theta), -sin(theta)*cos(p.alpha), sin(theta)*sin(p.alpha), p.a * cos(theta)},
            {sin(theta), cos(theta)*cos(p.alpha), -cos(theta)*sin(p.alpha), p.a * sin(theta)},
            {0, sin(p.alpha), cos(p.alpha), p.d},
            {0, 0, 0, 1}
        }};
#else
        Matrix4x4 Ti = {{
            {cos(theta), -sin(theta)*cos(p.alpha), sin(theta)*sin(p.alpha), p.a * cos(theta)},
            {sin(theta), cos(theta)*cos(p.alpha), -cos(theta)*sin(p.alpha), p.a * sin(theta)},
            {0, sin(p.alpha), cos(p.alpha), p.d},
            {0, 0, 0, 1}
        }};
#endif
        roundMatrix(Ti);

        multiplyMatrices(T, std::move(Ti));
    }

    return QVector3D(T[0][3], T[1][3], T[2][3]);
}

constexpr double RobotKinematics::degToRad(double deg) const {
    return deg * M_PI / 180.0;
}

#ifdef USE_AVX
void RobotKinematics::multiplyMatrices(Matrix4x4& fMatrix, Matrix4x4&& sMatrix) {
    alignas(32) std::array<std::array<double, 4>, 4> temp = {0.0};

    for (int i = 0; i < 4; ++i) {
        __m256d rowA = _mm256_load_pd(fMatrix[i].data());
        for (int j = 0; j < 4; ++j) {
            __m256d colB = _mm256_set_pd(sMatrix[3][j], sMatrix[2][j], sMatrix[1][j], sMatrix[0][j]);

            __m256d result = _mm256_mul_pd(rowA, colB);
            alignas(32) std::array<double, 4> sum = {0};
            _mm256_store_pd(sum.data(), result);
            temp[i][j] = sum[0] + sum[1] + sum[2] + sum[3];
        }
    }
    // Обновляем fMatrix
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            fMatrix[i][j] = temp[i][j];
        }
    }
}
#else
void RobotKinematics::multiplyMatrices(Matrix4x4& fMatrix, Matrix4x4&& sMatrix) {
    Matrix4x4 result = {0};

    for (size_t i = 0; i < 4; ++i) {
        for (size_t j = 0; j < 4; ++j) {
            result[i][j] = fMatrix[i][0] * sMatrix[0][j] +
                           fMatrix[i][1] * sMatrix[1][j] +
                           fMatrix[i][2] * sMatrix[2][j] +
                           fMatrix[i][3] * sMatrix[3][j];
        }
    }
    fMatrix = result;
}
#endif

constexpr void RobotKinematics::roundMatrix(Matrix4x4& matrix, double threshold) {
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            matrix[i][j] = roundSmallValues(matrix[i][j], threshold);
        }
    }
}
constexpr double RobotKinematics::roundSmallValues(double value, double threshold) {
    if (std::fabs(value) < threshold) {
        return 0.0;
    }
    return value; // Если нужно сократить количество чисел после запятой std::round(value * std::pow(10, precision)) / std::pow(10, precision).
}
