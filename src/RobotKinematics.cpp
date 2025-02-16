#include "RobotKinematics.h"
// Защита от отсутствия <immintrin.h> на неподдерживаемых платформах
#if defined(__x86_64__) || defined(_M_X64) || defined(__i386__) || defined(_M_IX86)
    #if AVX_SUP == 1
        #include <immintrin.h> // Для AVX
    #endif
#endif

// По-хорошему должно выноситься в отдельный файл Tools/Utils, но используется только в данном файле, поэтому применяется static
static constexpr double abs_constexpr(double x) {
    return x < 0 ? -x : x;
}
static constexpr double PI = 3.14159265358979323846;

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
#if AVX_SUP == 1
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
#if AVX_SUP == 1
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
    return deg * PI / 180.0;
}

void RobotKinematics::multiplyMatrices(Matrix4x4& fMatrix, Matrix4x4&& sMatrix) {
#if AVX_SUP == 1
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

#else

    Matrix4x4 result;

    for (size_t i = 0; i < 4; ++i) {
        for (size_t j = 0; j < 4; ++j) {
            result[i][j] = fMatrix[i][0] * sMatrix[0][j] +
                           fMatrix[i][1] * sMatrix[1][j] +
                           fMatrix[i][2] * sMatrix[2][j] +
                           fMatrix[i][3] * sMatrix[3][j];
        }
    }
    fMatrix = result;

#endif
}

void RobotKinematics::roundMatrix(Matrix4x4& matrix) {
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            matrix[i][j] = roundValue(matrix[i][j]);
        }
    }
}
constexpr double RobotKinematics::roundValue(double value) const {
    return (abs_constexpr(value) < threshold) ? 0.0 : value; // Если нужно сократить количество чисел после запятой: std::round(value * std::pow(10, precision)) / std::pow(10, precision).;
}
