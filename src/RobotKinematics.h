#include <QObject>
#include <QVector3D>
#include <QDebug>
#include <array>

#include <qmath.h>

using Matrix4x4 = std::array<std::array<double, 4>, 4>;

class RobotKinematics : public QObject {
public:
    struct DHParams { // Структура для хранения DH-параметров
        double a;
        double d;
        double alpha;
    };

public: // Публичные методы для работы с классом
    explicit RobotKinematics(QObject *parent = nullptr);
    Q_INVOKABLE QVector3D calculatePosition(const QList<double> &jointAnglesDeg);

private: // Приватные методы для вычислений (Скрытая логика)
    Q_OBJECT

    constexpr double degToRad(double deg) const; // перевод градусов в радианы
    inline void multiplyMatrices(Matrix4x4& fMatrix, Matrix4x4&& sMatrix); // Перемножение матриц
    inline void roundMatrix(Matrix4x4& matrix); // Функция для округления всей матрицы
    constexpr double roundValue(double value) const; // Функция для округления числа

private: // Переменные
    QList<DHParams> dhParams;
    constexpr static double threshold = 1e-10;
};
