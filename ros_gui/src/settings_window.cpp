#include "../include/ros_gui/settings_window.hpp"
#include "ui_settings_window.h"

#include <QtGui>
#include <QMessageBox>
#include <iostream>

using namespace Qt;

/*****************************************************************************
** Implementation [SettingsWindow]
*****************************************************************************/

SettingsWindow::SettingsWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::SettingsWindow)
{
    ui->setupUi(this);

    device = new SerialPort();

    ReadSettings();
}

SettingsWindow::~SettingsWindow()
{
    delete ui;
}

/*
void SettingsWindow::setSerial(const char *port){
    //QSettings settings("Qt-Ros Package", "ros_gui_settings_window");
    //QString serial_port = settings.value("serial_port",QString("/dev/ttyACM0")).toString();

    //device.fd = -12;
    //device.port = new char[12];

    //strcpy(device.port, port);

    //std::cout << "device = " << device.port << "\n";
}
*/

void SettingsWindow::connectSerial(){
    if(!device->connectSerial()){
        showMessage("Invalid port value. Couldn't open device");
    }
    else{
        ui->line_edit_serial_port->setEnabled(false);
        ui->button_open_port->setEnabled(false);
        ui->button_close_port->setEnabled(true);
        ui->checkbox_default_port->setEnabled(false);

        showMessage("Connected to the device");
    }
}

/*bool SettingsWindow::closeSerial(){
    //using namespace std;

    //if(device.fd == -1)
    //    return false;


    //return close(device.fd);
}
*/

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/

void SettingsWindow::on_button_close_port_clicked(){
    if(!device->disconnectSerial()){
        showMessage("Invalid port value. Couldn't close device");
        close();
    }
    else{
        showMessage("Closed device");
        //std::cout << "Closed device " << device.port << "\n";
        ui->button_open_port->setEnabled(true);
        ui->button_close_port->setEnabled(false);
        if(!ui->checkbox_default_port->isChecked())
            ui->line_edit_serial_port->setEnabled(true);

        ui->checkbox_default_port->setEnabled(true);
    }

}

void SettingsWindow::on_button_open_port_clicked(){
    char* port = new char[12];

    strcpy(port, ui->line_edit_serial_port->text().toStdString().c_str());

    //std::cout << "port = " << port << "\n";
    //std::cout << "device->getPort() = " << device->getPort() << "\n";

    //delete device;

    //device = new SerialPort();

    device->updateSerial();

    device->setPort(port);
    //std::cout << "//getPort()//device = " << device->getPort() << "\n";

    if(!device->connectSerial()){
        showMessage("Invalid port value. Couldn't open device");
    }
    else{
        ui->line_edit_serial_port->setEnabled(false);
        ui->button_open_port->setEnabled(false);
        ui->button_close_port->setEnabled(true);
        ui->checkbox_default_port->setEnabled(false);

        showMessage("Connected to the device");
    }

}

void SettingsWindow::on_checkbox_default_port_stateChanged(int state) {
    bool enabled;
    if ( state == 0 ) {
        enabled = true;
        ui->line_edit_serial_port->setText("/dev/");
    }
    else {
        enabled = false;
        ui->line_edit_serial_port->setText("/dev/ttyUSB0");
    }

    ui->line_edit_serial_port->setEnabled(enabled);

}

/*****************************************************************************
** Implementation [Configuration]
*****************************************************************************/

void SettingsWindow::ReadSettings() {
    QSettings settings("Qt-Ros Package", "ros_gui_settings_window");
    restoreGeometry(settings.value("geometry").toByteArray());
    restoreState(settings.value("windowState").toByteArray());

    QString serial_port = settings.value("serial_port",QString("/dev/ttyACM0")).toString();
    ui->line_edit_serial_port->setText(serial_port);

    bool default_port = settings.value("use_default_port", false).toBool();
    ui->checkbox_default_port->setChecked(default_port);
    if(default_port){
        //ui->line_edit_serial_port->setText("/dev/ttyACM0");
        ui->line_edit_serial_port->setEnabled(false);
    }

    device->setPort((char*)serial_port.toStdString().c_str());
    std::cout << "//getPort()//device = " << device->getPort() << "\n";

    if(!device->connectSerial()){
        showMessage("Invalid port value. Couldn't open device");
    }
    else{
        ui->line_edit_serial_port->setEnabled(false);
        ui->button_open_port->setEnabled(false);
        ui->button_close_port->setEnabled(true);
        ui->checkbox_default_port->setEnabled(false);

        showMessage("Connected to the device");
    }
}

void SettingsWindow::WriteSettings() {
    QSettings settings("Qt-Ros Package", "ros_gui_settings_window");
    settings.setValue("geometry", saveGeometry());
    settings.setValue("windowState", saveState());
    settings.setValue("use_default_port",QVariant(ui->checkbox_default_port->isChecked()));
    settings.setValue("serial_port",ui->line_edit_serial_port->text());
}

void SettingsWindow::closeEvent(QCloseEvent *event)
{
    WriteSettings();
    QMainWindow::closeEvent(event);
}

void SettingsWindow::showMessage(QString message){
    QMessageBox msgBox;
    msgBox.setText(message);
    msgBox.exec();
}
