#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QString>
#include <QMessageBox>
#include <QScrollBar>

using namespace std::chrono; // for seconds(), milliseconds()
using namespace std::this_thread; // for sleep_for()

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    // initialized number of robots
    robot_count = 5;

    //initialize url
    connection_url = "udp://0.0.0.0:14540";

    // initialize vectors
    systems.resize(robot_count);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_set_robots_count_clicked()
{
    /*
     * Updates robot_count member
     * Affects the expected number of connected vehicles
     *
     */

    robot_count = ui->robot_count_input->value();
    ui->status_msgs_output->append("Robot count set to: "+QString::number(robot_count));
    //QMessageBox::information(this, tr("Robot count"), "Number of robots is update: "+QString::number(robot_count));
}

void MainWindow::initialize_connection()
{
    // add connection
    ConnectionResult connection_result = dc.add_any_connection(connection_url);
    QString stat_str = connection_result_str(connection_result) ;
    ui->status_msgs_output->append("Connection status: "+stat_str);


    sleep_for(seconds(1));

    dc.register_on_discover([this](uint64_t uuid)
    {
        //QMessageBox msgbox;
        //msgbox.setText("Discovered system: " + QString::number(uuid));
        //msgbox.exec();
        this->ui->status_msgs_output->append("Discovered system: " + QString::number(uuid));
        System &system = dc.system(uuid);
        systems.at(system.get_system_id()-1) = &system;
        uint8_t id = systems.at(0)->get_system_id();
        std::cout << "System ID: " << unsigned(id) << std::endl;
    }
    );
}


/*
void MainWindow::on_status_msgs_output_textChanged()
{
    QScrollBar *sb = ui->status_msgs_output->verticalScrollBar();
    sb->setValue(sb->maximum());
    //ui->status_msgs_output->setVerticalScrollBar()

}
*/
