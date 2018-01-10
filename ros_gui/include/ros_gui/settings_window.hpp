#ifndef SETTINGS_WINDOW_HPP
#define SETTINGS_WINDOW_HPP

#include <QMainWindow>
#include "ui_settings_window.h"

#include <sstream>
#include <iostream>
#include <fstream>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>

#include <fcntl.h>
#include <unistd.h>
#include <sys/types.h>
#include <errno.h>      // Error number definitions
#include <termios.h>    // POSIX terminal control definitions

#include "serial_port.hpp"

//struct serial { 
//	char *port;
//	int fd;
//};

namespace Ui {
class SettingsWindow;
}

class SettingsWindow : public QMainWindow
{
    Q_OBJECT
    
public:
	//serial device;

	SerialPort *device;

    explicit SettingsWindow(QWidget *parent = 0);
    ~SettingsWindow();
    
	void connectSerial();
	//void setSerial(const char *port);
	//bool closeSerial();

	void ReadSettings();
	void WriteSettings();
	void closeEvent(QCloseEvent *event);
	void showMessage(QString message);


public Q_SLOTS:
	/******************************************
	** Auto-connections (connectSlotsByName())
	*******************************************/
	void on_checkbox_default_port_stateChanged(int state);
	void on_button_close_port_clicked();
	void on_button_open_port_clicked();

private:
    Ui::SettingsWindow *ui;
};

#endif // SETTINGS_WINDOW_HPP
