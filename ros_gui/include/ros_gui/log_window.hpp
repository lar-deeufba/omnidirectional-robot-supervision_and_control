#ifndef LOG_WINDOW_HPP
#define LOG_WINDOW_HPP

#include <ros_gui/RX_data.h>
#include <QMainWindow>

namespace Ui {
class LogWindow;
}

class LogWindow : public QMainWindow
{
    Q_OBJECT
    
public:
    explicit LogWindow(QWidget *parent = 0);
    ~LogWindow();
    
	void ReadSettings();
	void WriteSettings();
	void closeEvent(QCloseEvent *event);
	
    void updateTable(ros_gui::RX_data pkg_data);

	void exportData(int row);
	
public Q_SLOTS:
	/******************************************
	** Auto-connections (connectSlotsByName())
	*******************************************/
	//void mousePressEvent ( QMouseEvent * event );
	//void on_tableWidget_cellClicked(int, int);
	//void on_tableWidget_cellClicked( QMouseEvent*);
	void contextMenu(QPoint pos);
	
private:
    Ui::LogWindow *ui;
};

#endif // LOG_WINDOW_HPP
