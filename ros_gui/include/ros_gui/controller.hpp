/**
 * @file /include/ros_gui/controller.hpp
 *
 * @brief Controller central!
 *
 * @date March 2015
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ros_gui_SERIAL_CONTROLLER_HPP_
#define ros_gui_SERIAL_CONTROLLER_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <string.h>
#include <QThread>
#include <QStringListModel>
#include "serial_port.hpp"
#include "serialCommunication.hpp"

//including messages
#include <ros_gui/RX_data.h>

#define RX_BUFFER_SIZE 48
#define PI 3.14159265

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ros_gui {

/*****************************************************************************
** Class
*****************************************************************************/

class Controller : public QThread {
    Q_OBJECT
public:
	//SerialPort *port;
    char rx_buffer[RX_BUFFER_SIZE];

	//ros_gui::RX_data pkg_data;
	//bool freeze;

    RX_data pkg_data;
	bool dataReceived;
    bool enable;

	Controller(int argc, char** argv );
	virtual ~Controller();
	bool init(SerialPort* device);
	void run();
    bool readSerial();
    void publisher(int dt);
    void optmum_predictor();
    void smith_predictor();
    void trajetoria(int time);
    double DiffAngle(double a1, double a2);
    void TrajectoryControl(double x, double y, double theta);
    double Dist(double x, double y);
    double Rad(double xw);

    /*********************
    ** Logging
    **********************/
    enum LogLevel {
             Debug,
             Info,
             Warn,
             Error,
             Fatal
     };

    QStringListModel* loggingModel() { return &logging_model; }
    void log( const LogLevel &level, const std::string &msg);


Q_SIGNALS:
    void loggingUpdated();
    void rosShutdown();
//	void updateData();
    void updateList();

private:
    int init_argc;
    char** init_argv;
	SerialPort *port;
    SerialCommunication sC;
    QStringListModel logging_model;

    static const float Kdlqr[3][6];
    static const float Ag[6][6];
    static const float Bg[6][3];
    static const float Ad[3][3];
    static const float Bd[3][3];
    static const float Kd[3];
    static const float Gd[3];
    static const float alpha;

    float TrajX[100];             //vetor contendo a trajetória em X
    float TrajY[100];             //vetor contendo a trajetória em Y
    float TrajTheta[100];
    int Index;
    int TrajPoints;
    int TrajOK;

    float xr,yr,theta;

    float Vref[3];
    float dU[3], U[3];
    float vel[3], old_vel[3];
    float velrad[3];
    float x[3], old_x[3];
    float xFiltrado[3];
    float velm[3], new_velm[3];
    float difvel[3];

    char PWM[4];


};

}  // namespace ros_gui

#endif /* ros_gui_CONTROLLER_HPP_ */

