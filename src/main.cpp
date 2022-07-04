#include <QGuiApplication>
#include <QApplication>

#include "joint_state_widget.h"

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);
    JointStateWidget widget(argc, argv);

    widget.show(); // default QT generation
    return app.exec(); // default QT generation.
}
