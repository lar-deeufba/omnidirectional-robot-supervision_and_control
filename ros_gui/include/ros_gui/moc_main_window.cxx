/****************************************************************************
** Meta object code from reading C++ file 'main_window.hpp'
**
** Created by: The Qt Meta Object Compiler version 63 (Qt 4.8.6)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "main_window.hpp"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'main_window.hpp' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.6. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_ros_gui__MainWindow[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
      14,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: signature, parameters, type, tag, flags
      21,   20,   20,   20, 0x0a,
      48,   20,   20,   20, 0x0a,
      78,   20,   20,   20, 0x0a,
     104,   20,   20,   20, 0x0a,
     131,   20,   20,   20, 0x0a,
     165,  159,   20,   20, 0x0a,
     197,   20,   20,   20, 0x0a,
     222,   20,   20,   20, 0x0a,
     247,   20,   20,   20, 0x0a,
     275,  273,   20,   20, 0x0a,
     319,  273,   20,   20, 0x0a,
     358,  356,   20,   20, 0x0a,
     385,   20,   20,   20, 0x0a,
     404,   20,   20,   20, 0x0a,

       0        // eod
};

static const char qt_meta_stringdata_ros_gui__MainWindow[] = {
    "ros_gui::MainWindow\0\0on_actionAbout_triggered()\0"
    "on_actionSettings_triggered()\0"
    "on_actionLogs_triggered()\0"
    "on_actionField_triggered()\0"
    "on_actionMyrviz_triggered()\0check\0"
    "on_button_connect_clicked(bool)\0"
    "on_button_send_clicked()\0"
    "on_button_stop_clicked()\0"
    "on_button_clear_clicked()\0i\0"
    "on_comboBox_motor1_currentIndexChanged(int)\0"
    "on_spinBox_speedM1_valueChanged(int)\0"
    "b\0on_rb_freeze_toggled(bool)\0"
    "updateSerialData()\0updateList()\0"
};

void ros_gui::MainWindow::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        MainWindow *_t = static_cast<MainWindow *>(_o);
        switch (_id) {
        case 0: _t->on_actionAbout_triggered(); break;
        case 1: _t->on_actionSettings_triggered(); break;
        case 2: _t->on_actionLogs_triggered(); break;
        case 3: _t->on_actionField_triggered(); break;
        case 4: _t->on_actionMyrviz_triggered(); break;
        case 5: _t->on_button_connect_clicked((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 6: _t->on_button_send_clicked(); break;
        case 7: _t->on_button_stop_clicked(); break;
        case 8: _t->on_button_clear_clicked(); break;
        case 9: _t->on_comboBox_motor1_currentIndexChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 10: _t->on_spinBox_speedM1_valueChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 11: _t->on_rb_freeze_toggled((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 12: _t->updateSerialData(); break;
        case 13: _t->updateList(); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData ros_gui::MainWindow::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject ros_gui::MainWindow::staticMetaObject = {
    { &QMainWindow::staticMetaObject, qt_meta_stringdata_ros_gui__MainWindow,
      qt_meta_data_ros_gui__MainWindow, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &ros_gui::MainWindow::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *ros_gui::MainWindow::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *ros_gui::MainWindow::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_ros_gui__MainWindow))
        return static_cast<void*>(const_cast< MainWindow*>(this));
    return QMainWindow::qt_metacast(_clname);
}

int ros_gui::MainWindow::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QMainWindow::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 14)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 14;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
