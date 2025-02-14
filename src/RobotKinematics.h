#include <QObject>
#include <QVector3D>
#include <QDebug>
#include <cmath>
#include <array>
// #include <Eigen/Dense> // Можно использовать для удобства и оптимизации, но в тз об этом ничего не сказано.

// Проверяем поддержку AVX на этапе компиляции (если компилятор поддерживает)
#ifdef __AVX__
#define USE_AVX 1
#include <immintrin.h> // Для AVX/AVX2/SSE
// #include <cpuid.h> // Для AVX/AVX2/SSE и для отключения через if constexpr (C++17)
#else
#define USE_AVX 0
#endif

using Matrix4x4 = std::array<std::array<double, 4>, 4>;

class RobotKinematics : public QObject {
public:
    struct DHParams { // Структура для хранения параметров
        double a;
        double d;
        double alpha;
    };
public: // Публичные методы для работы с классом
    explicit RobotKinematics(QObject *parent = nullptr);
    Q_INVOKABLE QVector3D calculatePosition(const QList<double> &jointAnglesDeg);

private: // Приватные методы для рассчёта матрицы T
    Q_OBJECT

    constexpr double degToRad(double deg) const; // перевод градусов в радианы
#ifdef USE_AVX //1
    void multiplyMatrices(Matrix4x4& fMatrix, Matrix4x4&& sMatrix); // перемножение матриц AVX
#else
    void multiplyMatrices(Matrix4x4& fMatrix, Matrix4x4&& sMatrix); // перемножение матриц без AVX
#endif
    constexpr void roundMatrix(Matrix4x4& matrix, double threshold = 1e-10); // Функция для округления всей матрицы
    constexpr double roundSmallValues(double value, double threshold = 1e-10); // Функция для округления значения

private: // Переменные
    QList<DHParams> dhParams;
};
