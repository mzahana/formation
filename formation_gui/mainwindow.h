    #ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

// Dronecode Includes
# include <dronecode_sdk/dronecode_sdk.h>
#include <dronecode_sdk/action.h>
#include <dronecode_sdk/mission.h>
#include <dronecode_sdk/telemetry.h>

// Other includes
#include <functional>
#include <future>
#include <iostream>
#include <memory>

using namespace dronecode_sdk;

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

    void initialize_connection();

private slots:
    void on_set_robots_count_clicked();

    //void on_status_msgs_output_textChanged();

private:
    Ui::MainWindow *ui;

    // Dronecede object
    DronecodeSDK dc;

    // Connection URL
    std::string connection_url;

    // number of robots
    uint8_t robot_count;

    // vector of systems
    std::vector<System *>  systems;

    // vector of actions
    std::vector<Action *> actions;

    // vector of telemetry
    std::vector<Telemetry *> telemetry;
};

#endif // MAINWINDOW_H
