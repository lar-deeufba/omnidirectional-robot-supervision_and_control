#include "../include/ros_gui/field_window.hpp"
#include "ui_field_window.h"

#include <QtGui>
#include <QMessageBox>
#include <iostream>

using namespace Qt;

/*****************************************************************************
** Implementation [FieldWindow]
*****************************************************************************/

FieldWindow::FieldWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::FieldWindow)
{
    ui->setupUi(this);

    drawField();

    ReadSettings();
}

FieldWindow::~FieldWindow()
{
    delete ui;
}


/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/



/*****************************************************************************
** Implementation [Configuration]
*****************************************************************************/

void FieldWindow::ReadSettings(){
    QSettings settings("Qt-Ros Package", "ros_gui_field_window");
    restoreGeometry(settings.value("geometry").toByteArray());
    restoreState(settings.value("windowState").toByteArray());


    float widthOffset = (field->getBoundaryWidth()-field->getWidth())/2;
    float depthOffset = (field->getBoundaryDepth()-field->getDepth())/2;
    ui->lineEdit->setText("widthOffset = " +QString::number(widthOffset));
    ui->lineEdit_2->setText("depthOffset = " +QString::number(depthOffset));


    //ui->line_width->setText(QString::number(gs->width()));
    //ui->line_height->setText(QString::number(gs->height()));

    ui->line_width->setText("topLeft().x = " + QString::number(gs->sceneRect().topLeft().x()));
    ui->line_height->setText("topLeftt().y = " +QString::number(gs->sceneRect().topLeft().y()));

    //ui->lineEdit->setText("bottomLeft().x = " +QString::number(gs->sceneRect().bottomLeft().x()));
    //ui->lineEdit_2->setText("bottomLeft().y = " +QString::number(gs->sceneRect().bottomLeft().y()));

    //ui->line_width->setText(QString::number((field->getBoundaryWidth()/2)));
    //ui->line_height->setText(QString::number((field->getBoundaryDepth()/2)));

    //ui->lineEdit->setText(QString::number(-1*(field->getBoundaryWidth()/2)));
    //ui->lineEdit_2->setText(QString::number((-1*(field->getBoundaryDepth()/2))));


    ui->lineEdit->hide();
    ui->lineEdit_2->hide();
    ui->line_width->hide();
    ui->line_height->hide();
    ui->label->hide();
    ui->label_2->hide();

}

void FieldWindow::WriteSettings()
{
    QSettings settings("Qt-Ros Package", "ros_gui_field_window");
    settings.setValue("geometry", saveGeometry());
    settings.setValue("windowState", saveState());
}

void FieldWindow::closeEvent(QCloseEvent *event){
    WriteSettings();
    QMainWindow::closeEvent(event);
}

void FieldWindow::drawField(){
    field = new Field();

    QPen blackPen(Qt::black);
    blackPen.setWidthF(0.5);
    QPen whitePen(Qt::white);
    whitePen.setWidthF(0.5);
    QPen yellowPen(Qt::yellow);
    yellowPen.setWidthF(0.5);
    QPen bluePen(Qt::blue);
    bluePen.setWidthF(0.5);

    gs = new QGraphicsScene(this);



/*
    //gs->setSceneRect(0,0, field->getBoundaryWidth(),field->getBoundaryDepth()/2);

    //external lines
    //gs->addLine(0,0,0,field->getBoundaryDepth()-1,QPen(QColor(0,0,0),0.5));
    //gs->addLine(0,field->getBoundaryDepth()-1,field->getBoundaryWidth()-1,field->getBoundaryDepth()-1,QPen(QColor(0,0,0),0.5));
    //gs->addLine(field->getBoundaryWidth()-1,field->getBoundaryDepth()-1,field->getBoundaryWidth()-1,0,QPen(QColor(0,0,0),0.5));
    //gs->addLine(field->getBoundaryWidth()-1,0,0,0,QPen(QColor(0,0,0),0.5));

    //field lines
    float widthOffset = (field->getBoundaryWidth()-field->getWidth())/2;
    float depthOffset = (field->getBoundaryDepth()-field->getDepth())/2;
    float middleX = (field->getBoundaryWidth()/2) - 1;
    float middleY = (field->getBoundaryDepth()/2) - 1;

    gs->addLine(widthOffset,depthOffset,widthOffset,field->getBoundaryDepth()-1-depthOffset,QPen(QColor(255,255,255),0.5));
    gs->addLine(widthOffset,field->getBoundaryDepth()-1-depthOffset,field->getBoundaryWidth()-1-widthOffset,field->getBoundaryDepth()-1-depthOffset,QPen(QColor(255,255,255),0.5));
    gs->addLine(field->getBoundaryWidth()-1-widthOffset,field->getBoundaryDepth()-1-depthOffset,field->getBoundaryWidth()-1-widthOffset,depthOffset,QPen(QColor(255,255,255),0.5));
    gs->addLine(field->getBoundaryWidth()-1-widthOffset,depthOffset,widthOffset,depthOffset,QPen(QColor(255,255,255),0.5));

    gs->addLine(middleX, depthOffset,middleX, field->getBoundaryDepth()-1-depthOffset,QPen(QColor(255,255,255),0.5));

    gs->addEllipse(middleX-field->getCircleRadius()/2,middleY-field->getCircleRadius()/2,field->getCircleRadius(),field->getCircleRadius(),QPen(QColor(255,255,255),0.5));
    gs->addEllipse(middleX-field->getCircleRadius()/10,middleY-field->getCircleRadius()/10,field->getCircleRadius()/5,field->getCircleRadius()/5,QPen(QColor(255,255,255),0.5));

    //goal lines
    float goalDepthOffset = (field->getBoundaryDepth()-field->getGoalDepth())/2 -1;
    //float goalWidthOffset = (field->getBoundaryWidth()-field->getGoalWidth())/2 -1;

    gs->addLine(widthOffset,goalDepthOffset,0,goalDepthOffset,QPen(QColor(0,0,255),0.5));
    gs->addLine(0,goalDepthOffset,0,goalDepthOffset+field->getGoalDepth(),QPen(QColor(0,0,255),0.5));
    gs->addLine(0,goalDepthOffset+field->getGoalDepth(),widthOffset,goalDepthOffset+field->getGoalDepth(),QPen(QColor(0,0,255),0.5));

    gs->addLine(field->getBoundaryWidth()-1-widthOffset,goalDepthOffset,field->getBoundaryWidth()-1,goalDepthOffset,QPen(QColor(255,255,0),0.5));
    gs->addLine(field->getBoundaryWidth()-1,goalDepthOffset,field->getBoundaryWidth()-1,goalDepthOffset+field->getGoalDepth(),QPen(QColor(255,255,0),0.5));
    gs->addLine(field->getBoundaryWidth()-1,goalDepthOffset+field->getGoalDepth(),field->getBoundaryWidth()-1-widthOffset,goalDepthOffset+field->getGoalDepth(),QPen(QColor(255,255,0),0.5));

    //area lines
    float areaDepthOffset = (field->getBoundaryDepth()-field->getAreaDepth())/2 -1;
    //float areaWidthOffset = (field->getBoundaryWidth()-field->getAreaWidth())/2 -1;

    gs->addLine(widthOffset,areaDepthOffset,widthOffset+field->getAreaWidth(),areaDepthOffset,QPen(QColor(255,255,255),0.5));
    gs->addLine(widthOffset+field->getAreaWidth(),areaDepthOffset,widthOffset+field->getAreaWidth(),areaDepthOffset+field->getAreaDepth(),QPen(QColor(255,255,255),0.5));
    gs->addLine(widthOffset+field->getAreaWidth(),areaDepthOffset+field->getAreaDepth(),widthOffset,areaDepthOffset+field->getAreaDepth(),QPen(QColor(255,255,255),0.5));

    gs->addLine(field->getBoundaryWidth()-1-widthOffset,areaDepthOffset,field->getBoundaryWidth()-1-widthOffset-field->getAreaWidth(),areaDepthOffset,QPen(QColor(255,255,255),0.5));
    gs->addLine(field->getBoundaryWidth()-1-widthOffset-field->getAreaWidth(),areaDepthOffset,field->getBoundaryWidth()-1-widthOffset-field->getAreaWidth(),areaDepthOffset+field->getAreaDepth(),QPen(QColor(255,255,255),0.5));
    gs->addLine(field->getBoundaryWidth()-1-widthOffset-field->getAreaWidth(),areaDepthOffset+field->getAreaDepth(),field->getBoundaryWidth()-1-widthOffset,areaDepthOffset+field->getAreaDepth(),QPen(QColor(255,255,255),0.5));

    //goalkeeper area lines
    float gkAreaDepthOffset = (field->getBoundaryDepth()-field->getKeeperAreaDepth())/2 -1;
    //float gkAeaWidthOffset = (field->getBoundaryWidth()-field->getKeeperAreaWidth())/2 -1;

    gs->addLine(widthOffset,gkAreaDepthOffset,widthOffset+field->getKeeperAreaWidth(),gkAreaDepthOffset,QPen(QColor(255,255,255),0.5));
    gs->addLine(widthOffset+field->getKeeperAreaWidth(),gkAreaDepthOffset,widthOffset+field->getKeeperAreaWidth(),gkAreaDepthOffset+field->getKeeperAreaDepth(),QPen(QColor(255,255,255),0.5));
    gs->addLine(widthOffset+field->getKeeperAreaWidth(),gkAreaDepthOffset+field->getKeeperAreaDepth(),widthOffset,gkAreaDepthOffset+field->getKeeperAreaDepth(),QPen(QColor(255,255,255),0.5));

    gs->addLine(field->getBoundaryWidth()-1-widthOffset,gkAreaDepthOffset,field->getBoundaryWidth()-1-widthOffset-field->getKeeperAreaWidth(),gkAreaDepthOffset,QPen(QColor(255,255,255),0.5));
    gs->addLine(field->getBoundaryWidth()-1-widthOffset-field->getKeeperAreaWidth(),gkAreaDepthOffset,field->getBoundaryWidth()-1-widthOffset-field->getKeeperAreaWidth(),gkAreaDepthOffset+field->getKeeperAreaDepth(),QPen(QColor(255,255,255),0.5));
    gs->addLine(field->getBoundaryWidth()-1-widthOffset-field->getKeeperAreaWidth(),gkAreaDepthOffset+field->getKeeperAreaDepth(),field->getBoundaryWidth()-1-widthOffset,gkAreaDepthOffset+field->getKeeperAreaDepth(),QPen(QColor(255,255,255),0.5));

*/
    gs->setSceneRect(-1*(field->getBoundaryWidth()/2),-1*(field->getBoundaryDepth()/2),field->getBoundaryWidth(),field->getBoundaryDepth());

    //external lines
    QLineF topLine(gs->sceneRect().topLeft(),gs->sceneRect().topRight());
    QLineF leftLine(gs->sceneRect().topLeft(),gs->sceneRect().bottomLeft());
    QLineF rightLine(gs->sceneRect().topRight(),gs->sceneRect().bottomRight());
    QLineF bottomLine(gs->sceneRect().bottomLeft(),gs->sceneRect().bottomRight());

    gs->addLine(topLine,blackPen);
    gs->addLine(leftLine,blackPen);
    gs->addLine(rightLine,blackPen);
    gs->addLine(bottomLine,blackPen);

    //field lines
    float widthOffset = (field->getBoundaryWidth()-field->getWidth())/2;
    float depthOffset = (field->getBoundaryDepth()-field->getDepth())/2;
    float middleX = 0;
    float middleY = 0;

    QPointF fieldTopLeft(gs->sceneRect().topLeft().x()+widthOffset,gs->sceneRect().topLeft().y()+depthOffset);
    QPointF fieldBottomLeft(gs->sceneRect().bottomLeft().x()+widthOffset,gs->sceneRect().bottomLeft().y()-depthOffset);
    QPointF fieldTopRight(gs->sceneRect().topRight().x()-widthOffset,gs->sceneRect().topRight().y()+depthOffset);
    QPointF fieldBottomRight(gs->sceneRect().bottomRight().x()-widthOffset,gs->sceneRect().bottomRight().y()-depthOffset);

    QLineF fieldTopLine(fieldTopLeft,fieldTopRight);
    QLineF fieldLeftLine(fieldTopLeft,fieldBottomLeft);
    QLineF fieldRightLine(fieldTopRight,fieldBottomRight);
    QLineF fieldBottomLine(fieldBottomLeft, fieldBottomRight);

    gs->addLine(fieldTopLine,whitePen);
    gs->addLine(fieldLeftLine,whitePen);
    gs->addLine(fieldRightLine,whitePen);
    gs->addLine(fieldBottomLine,whitePen);

    gs->addLine(middleX, gs->sceneRect().topLeft().y()+depthOffset, middleX, gs->sceneRect().bottomLeft().y()-depthOffset,whitePen);

    gs->addEllipse(middleX-field->getCircleRadius()/2,middleY-field->getCircleRadius()/2,field->getCircleRadius(),field->getCircleRadius(),whitePen);
    gs->addEllipse(middleX-field->getCircleRadius()/10,middleY-field->getCircleRadius()/10,field->getCircleRadius()/5,field->getCircleRadius()/5,whitePen);

    //goal lines
    float goalDepthOffset = (field->getBoundaryDepth()-field->getGoalDepth())/2;
    //float goalWidthOffset = (field->getBoundaryWidth()-field->getGoalWidth())/2 -1;

    QLineF leftGoalTopLine(gs->sceneRect().topLeft().x()+widthOffset,gs->sceneRect().topLeft().y()+goalDepthOffset,gs->sceneRect().topLeft().x(),gs->sceneRect().topLeft().y()+goalDepthOffset);
    QLineF leftGoalLeftLine(gs->sceneRect().topLeft().x(),gs->sceneRect().topLeft().y()+goalDepthOffset,gs->sceneRect().bottomLeft().x(),gs->sceneRect().topLeft().y()+(goalDepthOffset+field->getGoalDepth()));
    QLineF leftGoalBottomLine(gs->sceneRect().bottomLeft().x(),gs->sceneRect().topLeft().y()+(goalDepthOffset+field->getGoalDepth()),gs->sceneRect().bottomLeft().x()+widthOffset, gs->sceneRect().topLeft().y()+goalDepthOffset+field->getGoalDepth());

    QLineF rightGoalTopLine(gs->sceneRect().topRight().x()-widthOffset,gs->sceneRect().topRight().y()+goalDepthOffset,gs->sceneRect().topRight().x(),gs->sceneRect().topRight().y()+goalDepthOffset);
    QLineF rightGoalRightLine(gs->sceneRect().topRight().x(),gs->sceneRect().topRight().y()+goalDepthOffset,gs->sceneRect().bottomRight().x(),gs->sceneRect().topRight().y()+(goalDepthOffset+field->getGoalDepth()));
    QLineF rightGoalBottomLine(gs->sceneRect().bottomRight().x(),gs->sceneRect().topRight().y()+(goalDepthOffset+field->getGoalDepth()),gs->sceneRect().bottomRight().x()-widthOffset, gs->sceneRect().topRight().y()+goalDepthOffset+field->getGoalDepth());

    gs->addLine(leftGoalTopLine, yellowPen);
    gs->addLine(leftGoalLeftLine, yellowPen);
    gs->addLine(leftGoalBottomLine, yellowPen);

    gs->addLine(rightGoalTopLine,bluePen);
    gs->addLine(rightGoalRightLine, bluePen);
    gs->addLine(rightGoalBottomLine, bluePen);

    //area lines
    float areaDepthOffset = (field->getBoundaryDepth()-field->getAreaDepth())/2 ;
    //float areaWidthOffset = (field->getBoundaryWidth()-field->getAreaWidth())/2 ;

    QLineF leftAreaTopLine(gs->sceneRect().topLeft().x()+widthOffset,gs->sceneRect().topLeft().y()+areaDepthOffset,gs->sceneRect().topLeft().x()+widthOffset+field->getAreaWidth(),gs->sceneRect().topLeft().y()+areaDepthOffset);
    QLineF leftAreaLeftLine(gs->sceneRect().topLeft().x()+widthOffset+field->getAreaWidth(),gs->sceneRect().topLeft().y()+areaDepthOffset,gs->sceneRect().bottomLeft().x()+widthOffset+field->getAreaWidth(),gs->sceneRect().topLeft().y()+(areaDepthOffset+field->getAreaDepth()));
    QLineF leftAreaBottomLine(gs->sceneRect().bottomLeft().x()+widthOffset+field->getAreaWidth(),gs->sceneRect().topLeft().y()+(areaDepthOffset+field->getAreaDepth()),gs->sceneRect().bottomLeft().x()+widthOffset, gs->sceneRect().topLeft().y()+areaDepthOffset+field->getAreaDepth());

    QLineF rightAreaTopLine(gs->sceneRect().topRight().x()-widthOffset,gs->sceneRect().topRight().y()+areaDepthOffset,gs->sceneRect().topRight().x()-widthOffset-field->getAreaWidth(),gs->sceneRect().topRight().y()+areaDepthOffset);
    QLineF rightAreaLeftLine(gs->sceneRect().topRight().x()-widthOffset-field->getAreaWidth(),gs->sceneRect().topRight().y()+areaDepthOffset,gs->sceneRect().bottomRight().x()-widthOffset-field->getAreaWidth(),gs->sceneRect().topRight().y()+(areaDepthOffset+field->getAreaDepth()));
    QLineF rightAreaBottomLine(gs->sceneRect().bottomRight().x()-widthOffset-field->getAreaWidth(),gs->sceneRect().topRight().y()+(areaDepthOffset+field->getAreaDepth()),gs->sceneRect().bottomRight().x()-widthOffset, gs->sceneRect().topRight().y()+areaDepthOffset+field->getAreaDepth());

    gs->addLine(leftAreaTopLine,whitePen);
    gs->addLine(leftAreaLeftLine,whitePen);
    gs->addLine(leftAreaBottomLine, whitePen);

    gs->addLine(rightAreaTopLine, whitePen);
    gs->addLine(rightAreaLeftLine, whitePen);
    gs->addLine(rightAreaBottomLine, whitePen);


    //goalkeeper area lines
    float gkAreaDepthOffset = (field->getBoundaryDepth()-field->getKeeperAreaDepth())/2 ;
    //float gkAeaWidthOffset = (field->getBoundaryWidth()-field->getKeeperAreaWidth())/2 -1;

    QLineF leftGKAreaTopLine(gs->sceneRect().topLeft().x()+widthOffset,gs->sceneRect().topLeft().y()+gkAreaDepthOffset,gs->sceneRect().topLeft().x()+widthOffset+field->getKeeperAreaWidth(),gs->sceneRect().topLeft().y()+gkAreaDepthOffset);
    QLineF leftGKAreaLeftLine(gs->sceneRect().topLeft().x()+widthOffset+field->getKeeperAreaWidth(),gs->sceneRect().topLeft().y()+gkAreaDepthOffset,gs->sceneRect().bottomLeft().x()+widthOffset+field->getKeeperAreaWidth(),gs->sceneRect().topLeft().y()+(gkAreaDepthOffset+field->getKeeperAreaDepth()));
    QLineF leftGKAreaBottomLine(gs->sceneRect().bottomLeft().x()+widthOffset+field->getKeeperAreaWidth(),gs->sceneRect().topLeft().y()+(gkAreaDepthOffset+field->getKeeperAreaDepth()),gs->sceneRect().bottomLeft().x()+widthOffset, gs->sceneRect().topLeft().y()+gkAreaDepthOffset+field->getKeeperAreaDepth());

    QLineF rightGKAreaTopLine(gs->sceneRect().topRight().x()-widthOffset,gs->sceneRect().topRight().y()+gkAreaDepthOffset,gs->sceneRect().topRight().x()-widthOffset-field->getKeeperAreaWidth(),gs->sceneRect().topRight().y()+gkAreaDepthOffset);
    QLineF rightGKAreaLeftLine(gs->sceneRect().topRight().x()-widthOffset-field->getKeeperAreaWidth(),gs->sceneRect().topRight().y()+gkAreaDepthOffset,gs->sceneRect().bottomRight().x()-widthOffset-field->getKeeperAreaWidth(),gs->sceneRect().topRight().y()+(gkAreaDepthOffset+field->getKeeperAreaDepth()));
    QLineF rightGKAreaBottomLine(gs->sceneRect().bottomRight().x()-widthOffset-field->getKeeperAreaWidth(),gs->sceneRect().topRight().y()+(gkAreaDepthOffset+field->getKeeperAreaDepth()),gs->sceneRect().bottomRight().x()-widthOffset, gs->sceneRect().topRight().y()+gkAreaDepthOffset+field->getKeeperAreaDepth());

    gs->addLine(leftGKAreaTopLine, whitePen);
    gs->addLine(leftGKAreaLeftLine, whitePen);
    gs->addLine(leftGKAreaBottomLine, whitePen);

    gs->addLine(rightGKAreaTopLine, whitePen);
    gs->addLine(rightGKAreaLeftLine, whitePen);
    gs->addLine(rightGKAreaBottomLine, whitePen);


    //corner lines
    //drawCircle(widthOffset, depthOffset,10);

    //gs->addLine(0,middleY,field->getBoundaryWidth()-1,middleY,QPen(QColor(255,255,255),0.8));

    // a grid foreground
    //gs->setForegroundBrush(QBrush(Qt::lightGray, Qt::CrossPattern));

    gs->setBackgroundBrush(QColor(0,200,75));//Qt::green);

    ui->graphicsView->setScene(gs);

    ui->graphicsView->scale(2.0,2.0);
}

void FieldWindow::drawCircle(float h, float k, float r){
    float xi;
    float y = 0;

    printf("(x, y) = (%f,%f) | r = %f\n",h,k,r);

    for (xi = h;xi <= h+r;xi+=0.1)
    {
        printf("xi = %f\n",xi);
        float r2 = r*r;
        float x2 = (xi-h);
        x2 = x2*x2;
        double aux = r2-x2;
        y = sqrt(aux);
        y = y+k;

        printf("(xi, y) = (%f,%f)\n",xi,y);

        gs->addLine(h,k,xi,y,QPen(QColor(255,0,0),0.2));
    }
}

void FieldWindow::mousePressEvent(QGraphicsSceneMouseEvent *event)
{
    QMessageBox *msgBox = new QMessageBox();
    msgBox->setWindowTitle("Hello");
    msgBox->setText("You Clicked the Mouse Button");
    msgBox->show();
}
