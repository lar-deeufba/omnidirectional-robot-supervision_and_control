#include "../include/ros_gui/log_window.hpp"
#include "../include/ros_gui/main_window.hpp"
#include "ui_log_window.h"

#include <QtGui>
#include <QtCore>
#include <QMouseEvent>
#include <QFile>
#include <QFileDialog>
#include <QMessageBox>
#include <iostream>

using namespace Qt;

/*****************************************************************************
** Implementation [LogWindow]
*****************************************************************************/

LogWindow::LogWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::LogWindow)
{
    ui->setupUi(this);

    ReadSettings();

    ui->tableWidget->setRowCount(33);
    ui->tableWidget->setColumnCount(0);

    ui->tableWidget->setVerticalHeaderLabels(QStringList() << "Time" << "m1_velocity" << "m2_velocity" << "m3_velocity"
                                             << "m1_current" << "m2_current" << "m3_current"
                                             << "x_acelleration" << "y_acelleration" << "angular_velocity" << "compass"
                                             << "m1_dutycycle" << "m2_dutycycle" << "m3_dutycycle"
                                             << "m1_setpoint" << "m2_setpoint" << "m3_setpoint" << "m1_dU" << "m2_dU"
                                             << "m3_dU" << "m1_U" << "m2_U" << "m3_U"  << "V" << "Vn" << "W" << "elapsed_time"
                                             << "Xr" << "Yr" << "Teta"
                                             << "Vref" << "Vnref" << "Wref");

    //ui->tableWidget->insertRow(0);
    //QTableWidgetItem *itm = new QTableWidgetItem("time");
    //ui->tableWidget->setVerticalHeaderItem(0,itm);

    //ui->tableWidget->insertRow(ui->tableWidget->rowCount());
    //itm = new QTableWidgetItem("data");
    //ui->tableWidget->setVerticalHeaderItem(ui->tableWidget->rowCount()-1,itm);

    ui->tableWidget->setContextMenuPolicy(Qt::CustomContextMenu);
    QObject::connect(ui->tableWidget, SIGNAL(customContextMenuRequested(QPoint)), this, SLOT(contextMenu(QPoint)));
}

LogWindow::~LogWindow()
{
    delete ui;
}


void LogWindow::updateTable(ros_gui::RX_data pkg_data)
{
    ui->tableWidget->insertColumn(ui->tableWidget->columnCount());

    QTime tempo = QTime::currentTime();

    QTableWidgetItem *itm;
    itm = new QTableWidgetItem(tempo.toString()+"."+QString::number(tempo.msec()));
    ui->tableWidget->setItem(0,ui->tableWidget->columnCount()-1,itm);


    ui->tableWidget->setItem(1,ui->tableWidget->columnCount()-1,new QTableWidgetItem(QString::number(pkg_data.m1_velocity/10)));
    ui->tableWidget->setItem(2,ui->tableWidget->columnCount()-1,new QTableWidgetItem(QString::number(pkg_data.m2_velocity/10)));
    ui->tableWidget->setItem(3,ui->tableWidget->columnCount()-1,new QTableWidgetItem(QString::number(pkg_data.m3_velocity/10)));
    ui->tableWidget->setItem(4,ui->tableWidget->columnCount()-1,new QTableWidgetItem(QString::number(pkg_data.m1_current)));
    ui->tableWidget->setItem(5,ui->tableWidget->columnCount()-1,new QTableWidgetItem(QString::number(pkg_data.m2_current)));
    ui->tableWidget->setItem(6,ui->tableWidget->columnCount()-1,new QTableWidgetItem(QString::number(pkg_data.m3_current)));
    ui->tableWidget->setItem(7,ui->tableWidget->columnCount()-1,new QTableWidgetItem(QString::number(pkg_data.x_acelleration)));
    ui->tableWidget->setItem(8,ui->tableWidget->columnCount()-1,new QTableWidgetItem(QString::number(pkg_data.y_acelleration)));
    ui->tableWidget->setItem(9,ui->tableWidget->columnCount()-1,new QTableWidgetItem(QString::number(pkg_data.angular_velocity)));
    ui->tableWidget->setItem(10,ui->tableWidget->columnCount()-1,new QTableWidgetItem(QString::number(pkg_data.compass)));
    ui->tableWidget->setItem(11,ui->tableWidget->columnCount()-1,new QTableWidgetItem(QString::number(pkg_data.m1_dutycycle)));
    ui->tableWidget->setItem(12,ui->tableWidget->columnCount()-1,new QTableWidgetItem(QString::number(pkg_data.m2_dutycycle)));
    ui->tableWidget->setItem(13,ui->tableWidget->columnCount()-1,new QTableWidgetItem(QString::number(pkg_data.m3_dutycycle)));
    ui->tableWidget->setItem(14,ui->tableWidget->columnCount()-1,new QTableWidgetItem(QString::number(pkg_data.m1_setpoint)));
    ui->tableWidget->setItem(15,ui->tableWidget->columnCount()-1,new QTableWidgetItem(QString::number(pkg_data.m2_setpoint)));
    ui->tableWidget->setItem(16,ui->tableWidget->columnCount()-1,new QTableWidgetItem(QString::number(pkg_data.m3_setpoint)));
	ui->tableWidget->setItem(17,ui->tableWidget->columnCount()-1,new QTableWidgetItem(QString::number(pkg_data.m1_inc_control_signal)));
	ui->tableWidget->setItem(18,ui->tableWidget->columnCount()-1,new QTableWidgetItem(QString::number(pkg_data.m2_inc_control_signal)));
	ui->tableWidget->setItem(19,ui->tableWidget->columnCount()-1,new QTableWidgetItem(QString::number(pkg_data.m3_inc_control_signal)));
	ui->tableWidget->setItem(20,ui->tableWidget->columnCount()-1,new QTableWidgetItem(QString::number(pkg_data.m1_control_signal)));
	ui->tableWidget->setItem(21,ui->tableWidget->columnCount()-1,new QTableWidgetItem(QString::number(pkg_data.m2_control_signal)));
	ui->tableWidget->setItem(22,ui->tableWidget->columnCount()-1,new QTableWidgetItem(QString::number(pkg_data.m3_control_signal)));
	ui->tableWidget->setItem(23,ui->tableWidget->columnCount()-1,new QTableWidgetItem(QString::number(pkg_data.V)));
	ui->tableWidget->setItem(24,ui->tableWidget->columnCount()-1,new QTableWidgetItem(QString::number(pkg_data.Vn)));
	ui->tableWidget->setItem(25,ui->tableWidget->columnCount()-1,new QTableWidgetItem(QString::number(pkg_data.W)));
    ui->tableWidget->setItem(26,ui->tableWidget->columnCount()-1,new QTableWidgetItem(QString::number(pkg_data.delta_time)));
    ui->tableWidget->setItem(27,ui->tableWidget->columnCount()-1,new QTableWidgetItem(QString::number(pkg_data.X_robot)));
    ui->tableWidget->setItem(28,ui->tableWidget->columnCount()-1,new QTableWidgetItem(QString::number(pkg_data.Y_robot)));
    ui->tableWidget->setItem(29,ui->tableWidget->columnCount()-1,new QTableWidgetItem(QString::number(pkg_data.Theta_robot)));
    ui->tableWidget->setItem(30,ui->tableWidget->columnCount()-1,new QTableWidgetItem(QString::number(pkg_data.Vref)));
    ui->tableWidget->setItem(31,ui->tableWidget->columnCount()-1,new QTableWidgetItem(QString::number(pkg_data.Vnref)));
    ui->tableWidget->setItem(32,ui->tableWidget->columnCount()-1,new QTableWidgetItem(QString::number(pkg_data.Wref)));
//        itm = new QTableWidgetItem(aux);
//        itm->setFlags(Qt::ItemIsEnabled);
//        ui->tableWidget->setItem(i,ui->tableWidget->columnCount()-1,itm);



    ui->tableWidget->scrollToItem(ui->tableWidget->item(0,ui->tableWidget->columnCount()-1));
}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/


/*****************************************************************************
** Implemenation [Slots][manually connected]
*****************************************************************************/

void LogWindow::contextMenu(QPoint pos)
{
    QMenu *menu = new QMenu;

    QAction *exportTableAction = menu->addAction("Export Table");
    QAction *exportRowAction = menu->addAction("Export row");
    QAction *clearTableAction = menu->addAction("Clear all..");

    QAction *action = menu->exec(ui->tableWidget->viewport()->mapToGlobal(pos));

    if(action == exportTableAction){
        exportData(-1);
    }
    else if (action == exportRowAction)
    {
        QTableWidgetItem *aux = ui->tableWidget->itemAt(pos);
        if(aux == NULL)
        {
            QMessageBox msgBox;
            msgBox.setText("Invalid row selected!");
            msgBox.exec();
        }
        else
        {
            exportData(aux->row());
        }
    }
    else if(action == clearTableAction)
    {
        //ui->tableWidget->clearContents();
        for(int c = ui->tableWidget->columnCount()-1; c >= 0; c--)
            ui->tableWidget->removeColumn(c);
    }
}

void LogWindow::exportData(int row)
{
    std::cout << "Save data triggered in row " << row << "\n";

    QString fileName = QFileDialog::getSaveFileName(this,tr("Save File"),"~/Área\ de\ Trabalho/", "All Files (*.*);;Text Files (*.csv)");

    if(fileName.at(fileName.length()-4) != '.')
        fileName.append(".csv");

    QFile myF(fileName);
    if (myF.open(QFile::WriteOnly | QFile::Truncate))
    {
            QTextStream data(&myF);
            QStringList strList;

            if(row == -1)
            {
                for(int i = 0; i<ui->tableWidget->rowCount();i++){
                    strList <<ui->tableWidget->verticalHeaderItem(i)->data(Qt::DisplayRole).toString();// + ",";
                }

                data << strList.join(",") << "\n";

                for( int c = 0; c < ui->tableWidget->columnCount(); c++ )
                {
                   strList.clear();

                   for(int j = 0; j < ui->tableWidget->rowCount(); j++)
                   {

                        QTableWidgetItem* item = ui->tableWidget->item(j,c);        //Load items

                        if (!item || item->text().isEmpty())                        //Test if there is something at item(r,c)
                        {
                             ui->tableWidget->setItem(j,c,new QTableWidgetItem("0"));//IF there is nothing write 0
                        }

                        strList << ui->tableWidget->item( j, c )->text();//+ ",";

                   }

                   data << strList.join( "," ) << "\n";

                }
            }
            else
            {
               strList <<ui->tableWidget->verticalHeaderItem(row)->data(Qt::DisplayRole).toString();

               data << strList.join(",") << "\n";


               for( int c = 0; c < ui->tableWidget->columnCount(); c++ )
               {
                  strList.clear();
                  QTableWidgetItem* item = ui->tableWidget->item(row,c);        //Load items

                  if (!item || item->text().isEmpty())                        //Test if there is something at item(r,c)
                  {
                       ui->tableWidget->setItem(row,c,new QTableWidgetItem("0"));//IF there is nothing write 0
                  }

                  strList << ui->tableWidget->item( row, c )->text();
                  data << strList.join( "," ) << "\n";
               }
            }

            statusBar()->showMessage(tr("File saved successfully."), 3000);
            myF.close();
    }
    else
        std::cout << "Couldn't open the specified file.\n";

}

/*****************************************************************************
** Implementation [Configuration]
*****************************************************************************/

void LogWindow::ReadSettings() {
    QSettings settings("Qt-Ros Package", "ros_gui_log_window");
    restoreGeometry(settings.value("geometry").toByteArray());
    restoreState(settings.value("windowState").toByteArray());
}

void LogWindow::WriteSettings() {
    QSettings settings("Qt-Ros Package", "ros_gui_log_window");
    settings.setValue("geometry", saveGeometry());
    settings.setValue("windowState", saveState());
}

void LogWindow::closeEvent(QCloseEvent *event)
{
    WriteSettings();
    QMainWindow::closeEvent(event);
}
