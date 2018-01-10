/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include <math.h>
#include "../include/ros_gui/main_window.hpp"
#include "../include/ros_gui/log_window.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ros_gui {

using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
	: QMainWindow(parent)
	, qnode(argc,argv)
    , controller(argc, argv)
{

    ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
    QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application

    ReadSettings();
    setWindowIcon(QIcon(":/images/icon.png"));
    QObject::connect(&controller, SIGNAL(rosShutdown()), this, SLOT(close()));

    /*********************
    ** Logging
    **********************/
    //QObject::connect(&qnode, SIGNAL(updateData()), this, SLOT(updateSerialData()));
    QObject::connect(&controller, SIGNAL(updateList()), this, SLOT(updateList()));

    /*********************
    ** Auto Start
    **********************/
    //if ( ui.checkbox_remember_settings->isChecked() ) {
    //    on_button_connect_clicked(true);
    //}

    sw = new SettingsWindow(this);
    lw = new LogWindow(this);
    fw = new FieldWindow(this);

}

MainWindow::~MainWindow() {}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/

void MainWindow::showNoMasterMessage() {
	QMessageBox msgBox;
	msgBox.setText("Couldn't find the ros master.");
	msgBox.exec();
    close();
}

/*
 * These triggers whenever the button is clicked, regardless of whether it
 * is already checked or not.
 */

void MainWindow::on_button_send_clicked()
{

    double b = 0;

    //listenerNode.freeze = false;

    if(ui.m1_orientation->isChecked()) { b+=1; }
    if(ui.m2_orientation->isChecked()) { b+=2; }
    if(ui.m3_orientation->isChecked()) { b+=4; }

    sw->device->clearSerialData(0);

    if(ui.mode_pwm->isChecked()) {

        char a[4];

        a[0] = (trunc(42.5*ui.m1_set->value()));
        a[1] = (trunc(42.5*ui.m2_set->value()));
        a[2] = (trunc(42.5*ui.m3_set->value()));

        a[3] = b;

        sc.serialSend("13A20040B09872", 2, a, 4, sw->device->getFd());

    }
    else {

        char a[7];

        int m1,m2,m3;

        m1 = (trunc(10*ui.m1_set->value()));
        m2 = (trunc(10*ui.m2_set->value()));
        m3 = (trunc(10*ui.m3_set->value()));

        a[0] = (char) ((0xff00 & m1) >> 8);
        a[1] = (char) (0xff & m1);

        a[2] = (char) ((0xff00 & m2) >> 8);
        a[3] = (char) (0xff & m2);

        a[4] = (char) ((0xff00 & m3) >> 8);
        a[5] = (char) (0xff & m3);

        a[6] = b;

        //sw->device->clearSerialData(0);

        sc.serialSend("13A20040B09872", 3, a, 7, sw->device->getFd());

    }

}

void MainWindow::on_button_send_pid_gain_clicked() {

    char a[3];

    a[0] = 100*ui.m1_set_kp->value();
    a[1] = 100*ui.m1_set_kd->value();
    a[2] = 100*ui.m1_set_ki->value();

    sw->device->clearSerialData(0);

    sc.serialSend("13A20040B09872", 1, a, 3, sw->device->getFd());


}

void MainWindow::on_button_stop_clicked()
{
    if(ui.mode_automatic->isChecked())
        ui.mode_pwm->toggle();

    char a[4] = {0,0,0,0};

    //listenerNode.freeze = true;

    sw->device->clearSerialData(0);

    sc.serialSend("13A20040B09872", 2, a, 4, sw->device->getFd());



}

void MainWindow::on_button_connect_clicked(bool check ) {

    //char fool;

    if ( !qnode.init() ||!controller.init(sw->device) ) {
        showNoMasterMessage();
    } else {
        ui.button_connect->setEnabled(false);
        ui.button_send->setEnabled(true);
        ui.button_send_pid_gain->setEnabled(true);
        ui.button_stop->setEnabled(true);
        ui.mode_pid->setEnabled(true);
        ui.mode_pwm->setEnabled(true);
        ui.mode_automatic->setEnabled(true);
    }

    sw->device->clearSerialData(0);

    //sc.serialSend("13A20040B09872", 1, &fool, 0, sw->device->getFd());

}

/*****************************************************************************
** Implemenation: Mimic Logic
*****************************************************************************/

void MainWindow::on_m1_set_valueChanged ( double i ){
    if(ui.checkBox_mimic->isChecked()){
        ui.m2_set->setValue(i);
        ui.m3_set->setValue(i);
    }
}

void MainWindow::on_m2_set_valueChanged ( double i ){
    if(ui.checkBox_mimic->isChecked()){
        ui.m1_set->setValue(i);
        ui.m3_set->setValue(i);
    }
}

void MainWindow::on_m3_set_valueChanged ( double i ){
    if(ui.checkBox_mimic->isChecked()){
        ui.m1_set->setValue(i);
        ui.m2_set->setValue(i);
    }
}

void MainWindow::on_m1_set_kp_valueChanged ( double i ){
    if(ui.checkBox_mimic->isChecked()){
        ui.m2_set_kp->setValue(i);
        ui.m3_set_kp->setValue(i);
    }
}

void MainWindow::on_m2_set_kp_valueChanged ( double i ){
    if(ui.checkBox_mimic->isChecked()){
        ui.m1_set_kp->setValue(i);
        ui.m3_set_kp->setValue(i);
    }
}

void MainWindow::on_m3_set_kp_valueChanged ( double i ){
    if(ui.checkBox_mimic->isChecked()){
        ui.m1_set_kp->setValue(i);
        ui.m2_set_kp->setValue(i);
    }
}

void MainWindow::on_m1_set_ki_valueChanged ( double i ){
    if(ui.checkBox_mimic->isChecked()){
        ui.m2_set_ki->setValue(i);
        ui.m3_set_ki->setValue(i);
    }
}

void MainWindow::on_m2_set_ki_valueChanged ( double i ){
    if(ui.checkBox_mimic->isChecked()){
        ui.m1_set_ki->setValue(i);
        ui.m3_set_ki->setValue(i);
    }
}

void MainWindow::on_m3_set_ki_valueChanged ( double i ){
    if(ui.checkBox_mimic->isChecked()){
        ui.m1_set_ki->setValue(i);
        ui.m2_set_ki->setValue(i);
    }
}

void MainWindow::on_m1_set_kd_valueChanged ( double i ){
    if(ui.checkBox_mimic->isChecked()){
        ui.m2_set_kd->setValue(i);
        ui.m3_set_kd->setValue(i);
    }
}

void MainWindow::on_m2_set_kd_valueChanged ( double i ){
    if(ui.checkBox_mimic->isChecked()){
        ui.m1_set_kd->setValue(i);
        ui.m3_set_kd->setValue(i);
    }
}

void MainWindow::on_m3_set_kd_valueChanged ( double i ){
    if(ui.checkBox_mimic->isChecked()){
        ui.m1_set_kd->setValue(i);
        ui.m2_set_kd->setValue(i);
    }
}

/*****************************************************************************
** Implemenation
*****************************************************************************/


void MainWindow::on_m1_orientation_toggled(bool b){
    if(b)
        ui.m1_orientation->setText("CounterClockWise");
    else
        ui.m1_orientation->setText("ClockWise");
}

void MainWindow::on_m2_orientation_toggled(bool b){
    if(b)
        ui.m2_orientation->setText("CounterClockWise");
    else
        ui.m2_orientation->setText("ClockWise");
}

void MainWindow::on_m3_orientation_toggled(bool b){
    if(b)
        ui.m3_orientation->setText("CounterClockWise");
    else
        ui.m3_orientation->setText("ClockWise");
}

void MainWindow::on_mode_pid_toggled(bool b){
    if(b) {
        ui.mode_description->setText("RPM");
        ui.m1_set->setMaximum(1000);
        ui.m2_set->setMaximum(1000);
        ui.m3_set->setMaximum(1000);

        ui.button_send->setEnabled(true);
        ui.button_send_pid_gain->setEnabled(true);
        ui.button_stop->setEnabled(true);
        controller.enable = false;

    }
}

void MainWindow::on_mode_pwm_toggled(bool b){
    if(b) {
        ui.mode_description->setText("Voltage");
        ui.m1_set->setMaximum(6);
        ui.m2_set->setMaximum(6);
        ui.m3_set->setMaximum(6);

        ui.button_send->setEnabled(true);
        ui.button_send_pid_gain->setEnabled(true);
        ui.button_stop->setEnabled(true);
        controller.enable = false;

    }
}

void MainWindow::on_mode_automatic_toggled(bool b){
    if(b) {
        char fool;
        ui.button_send->setEnabled(false);
        ui.button_send_pid_gain->setEnabled(false);
        //ui.button_stop->setEnabled(false);
        controller.enable = true;
        //listenerNode.freeze = false;
        sw->device->clearSerialData(0);
        //Enviando pacote em branco para iniciar a comunicação
        //sc.serialSend("13A20040B09872", 1, &fool, 0, sw->device->getFd());

    }

}

/*****************************************************************************
** Implemenation [Slots][manually connected]
*****************************************************************************/

/**
 * This function is signalled by the underlying model. When the model changes,
 * this will drop the cursor down to the last line in the QListview to ensure
 * the user can always see the latest log message.
 */

void MainWindow::updateList(){
    //std::stringstream msg;

    //msg << "Data received: ["<< (int) qnode.RX[0] << "]|[" << (int) qnode.RX[1] << "]|[" << (int) qnode.RX[2] << "]|[" << (int) qnode.RX[3] << "]";

    lw->updateTable(controller.pkg_data);

    //controller.pkg_data = listenerNode.pkg_data;
    //controller.dataReceived = true;


}

/*****************************************************************************
** Implementation [Menu]
*****************************************************************************/

void MainWindow::on_actionAbout_triggered() {
    QMessageBox::about(this, tr("About ..."),tr("<h2>ROS_GUI Interface v1.0</h2><p>Copyright LaR</p><p>This package implements the graphical user interface for the ROS communication.</p>"));
}


void MainWindow::on_actionSettings_triggered()
{
    sw->show();

    if(!sw->isActiveWindow())
    {
        //std::cout << "settingWindow not visible \n";
        sw->activateWindow();
    }
}

void MainWindow::on_actionLogs_triggered()
{
    lw->show();

    if(!lw->isActiveWindow())
    {
        lw->activateWindow();
    }
}

void MainWindow::on_actionField_triggered()
{
    fw->show();

    if(!fw->isActiveWindow())
    {
        //std::cout << "fieldWindow not visible \n";
        fw->activateWindow();
    }
}

/*****************************************************************************
** Implementation [Configuration]
*****************************************************************************/

void MainWindow::ReadSettings()
{
    QSettings settings("Qt-Ros Package", "ros_gui");
    restoreGeometry(settings.value("geometry").toByteArray());
    restoreState(settings.value("windowState").toByteArray());

//    ui.comboBox_motor1->addItem("a");
//    ui.comboBox_motor1->addItem("h");
//    ui.comboBox_motor2->addItem("a");
//    ui.comboBox_motor2->addItem("h");

    //ui.rb_freeze->setChecked(true);
    //listenerNode.freeze = true;
}

void MainWindow::WriteSettings()
{
    QSettings settings("Qt-Ros Package", "ros_gui");

    settings.setValue("geometry", saveGeometry());
    settings.setValue("windowState", saveState());
}

void MainWindow::closeEvent(QCloseEvent *event)
{
	WriteSettings();
	QMainWindow::closeEvent(event);
}

}  // namespace ros_gui

