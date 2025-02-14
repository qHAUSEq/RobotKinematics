#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QQmlContext>
#include "RobotKinematics.h"

int main(int argc, char *argv[]) {
    QGuiApplication app(argc, argv);

    qmlRegisterType<RobotKinematics>("Robot", 1, 0, "RobotKinematics");

    QQmlApplicationEngine engine;
    engine.load(QUrl(QStringLiteral("qrc:/qml/main.qml")));

    return app.exec();
}
