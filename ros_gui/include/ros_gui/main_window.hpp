/**
 * @file /include/ros_gui/main_window.hpp
 *
 * @brief Qt based gui for ros_gui.
 *
 * @date November 2010
 **/
#ifndef ros_gui_MAIN_WINDOW_H
#define ros_gui_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "settings_window.hpp"
#include "log_window.hpp"
#include "field_window.hpp"
#include "qnode.hpp"
#include "serialCommunication.hpp"
#include "controller.hpp"

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace ros_gui {

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow {
Q_OBJECT


public:
	MainWindow(int argc, char** argv, QWidget *parent = 0);
	~MainWindow();

	void ReadSettings(); // Load up qt program settings at startup
	void WriteSettings(); // Save qt program settings when closing

	void closeEvent(QCloseEvent *event); // Overloaded function
	void showNoMasterMessage();

public Q_SLOTS:
	/******************************************
	** Auto-connections (connectSlotsByName())
	*******************************************/
	void on_actionAbout_triggered();
	void on_actionSettings_triggered();
	void on_actionLogs_triggered();
    void on_actionField_triggered();


	void on_button_connect_clicked(bool check);
    void on_button_send_pid_gain_clicked();
	void on_button_send_clicked();
    void on_button_stop_clicked();

    void on_m1_set_valueChanged ( double i );
    void on_m2_set_valueChanged ( double i );
    void on_m3_set_valueChanged ( double i );

    void on_m1_set_kp_valueChanged ( double i );
    void on_m1_set_ki_valueChanged ( double i );
    void on_m1_set_kd_valueChanged ( double i );
    void on_m2_set_kp_valueChanged ( double i );
    void on_m2_set_ki_valueChanged ( double i );
    void on_m2_set_kd_valueChanged ( double i );
    void on_m3_set_kp_valueChanged ( double i );
    void on_m3_set_ki_valueChanged ( double i );
    void on_m3_set_kd_valueChanged ( double i );

    void on_m1_orientation_toggled(bool b);
    void on_m2_orientation_toggled(bool b);
    void on_m3_orientation_toggled(bool b);

    void on_mode_pwm_toggled(bool b);
    void on_mode_pid_toggled(bool b);
    void on_mode_automatic_toggled(bool b);

	//void on_checkbox_use_environment_stateChanged(int state);

    /******************************************
    ** Manual connections
    *******************************************/
    //void updateLoggingView(); // no idea why this can't connect automatically
    void updateList();

private:
	Ui::MainWindowDesign ui;
	SettingsWindow *sw;
	LogWindow *lw;
	FieldWindow *fw;

	QNode qnode;
    //SerialListener listenerNode;

    SerialCommunication sc;
    Controller controller;

};

}  // namespace ros_gui

#endif // ros_gui_MAIN_WINDOW_H
