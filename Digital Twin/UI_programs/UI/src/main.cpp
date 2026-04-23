#include <QApplication>
#include "dashboard_window.h"

int main(int argc, char *argv[]) {
    QApplication a(argc, argv);
    DashboardWindow w;
    w.show();
    return a.exec();
}