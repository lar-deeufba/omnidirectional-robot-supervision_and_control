#ifndef FIELD_WINDOW_HPP
#define FIELD_WINDOW_HPP

#include <QtGui/QMainWindow>
#include <QtGui/QGraphicsScene>
#include <math.h>

#include "field.hpp"

namespace Ui {
class FieldWindow;
}

class FieldWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit FieldWindow(QWidget *parent = 0);
    ~FieldWindow();

	void drawField();
	void drawCircle(float h, float k, float r);

	void ReadSettings(); // Load up qt program settings at startup
	void WriteSettings(); // Save qt program settings when closing
	void closeEvent(QCloseEvent *event); // Overloaded function

//protected:
	void mousePressEvent(QGraphicsSceneMouseEvent *event);

private:
    Ui::FieldWindow *ui;
	QGraphicsScene *gs;

	Field *field;

};

#endif // FIELD_WINDOW_HPP
