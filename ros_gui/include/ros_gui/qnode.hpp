/**
 * @file /include/ros_gui/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ros_gui_QNODE_HPP_
#define ros_gui_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
//#include <string.h>
#include <QThread>
#include <QStringListModel>
#include "serial_port.hpp"

//including messages
#include <ros_gui/RX_data.h>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ros_gui {

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT
public:
	//SerialPort port;
	//char rx_buffer[RX_BUFFER_SIZE];
	int RX[4];
	ros_gui::RX_data pkg_data;

	QNode(int argc, char** argv );
	virtual ~QNode();
	bool init();
	//bool init(SerialPort* device);
	bool init(const std::string &master_url, const std::string &host_url);
	void run();

    //void rxDataCallback(const ros_gui::RX_data::ConstPtr& data);
	

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

	//QStringListModel* loggingModel() { return &logging_model; }
	//void log( const LogLevel &level, const std::string &msg);

Q_SIGNALS:
	void loggingUpdated();
    void rosShutdown();

	void updateData();
	void updateList();

private:
	int init_argc;
	char** init_argv;
	ros::Publisher chatter_publisher, info_pub, rx_pub;
	ros::Subscriber rx_sub;
    QStringListModel logging_model;
};

	//int errorCount, readCount, ATcountE, countSend;
	//char buf[BUFF_SIZE];
	


}  // namespace ros_gui

#endif /* ros_gui_QNODE_HPP_ */
