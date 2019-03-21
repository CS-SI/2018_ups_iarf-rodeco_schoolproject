/****************************************************************************
** Meta object code from reading C++ file 'qglviewer.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.5.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../../../cgal/GraphicsView/include/CGAL/Qt/qglviewer.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'qglviewer.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.5.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_CGAL__QGLViewer_t {
    QByteArrayData data[122];
    char stringdata0[1723];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_CGAL__QGLViewer_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_CGAL__QGLViewer_t qt_meta_stringdata_CGAL__QGLViewer = {
    {
QT_MOC_LITERAL(0, 0, 15), // "CGAL::QGLViewer"
QT_MOC_LITERAL(1, 16, 17), // "viewerInitialized"
QT_MOC_LITERAL(2, 34, 0), // ""
QT_MOC_LITERAL(3, 35, 10), // "drawNeeded"
QT_MOC_LITERAL(4, 46, 12), // "drawFinished"
QT_MOC_LITERAL(5, 59, 9), // "automatic"
QT_MOC_LITERAL(6, 69, 13), // "animateNeeded"
QT_MOC_LITERAL(7, 83, 12), // "helpRequired"
QT_MOC_LITERAL(8, 96, 18), // "axisIsDrawnChanged"
QT_MOC_LITERAL(9, 115, 5), // "drawn"
QT_MOC_LITERAL(10, 121, 18), // "gridIsDrawnChanged"
QT_MOC_LITERAL(11, 140, 21), // "FPSIsDisplayedChanged"
QT_MOC_LITERAL(12, 162, 9), // "displayed"
QT_MOC_LITERAL(13, 172, 20), // "textIsEnabledChanged"
QT_MOC_LITERAL(14, 193, 7), // "enabled"
QT_MOC_LITERAL(15, 201, 21), // "cameraIsEditedChanged"
QT_MOC_LITERAL(16, 223, 6), // "edited"
QT_MOC_LITERAL(17, 230, 13), // "pointSelected"
QT_MOC_LITERAL(18, 244, 18), // "const QMouseEvent*"
QT_MOC_LITERAL(19, 263, 1), // "e"
QT_MOC_LITERAL(20, 265, 19), // "mouseGrabberChanged"
QT_MOC_LITERAL(21, 285, 24), // "qglviewer::MouseGrabber*"
QT_MOC_LITERAL(22, 310, 12), // "mouseGrabber"
QT_MOC_LITERAL(23, 323, 14), // "needNewContext"
QT_MOC_LITERAL(24, 338, 14), // "setAxisIsDrawn"
QT_MOC_LITERAL(25, 353, 4), // "draw"
QT_MOC_LITERAL(26, 358, 14), // "setGridIsDrawn"
QT_MOC_LITERAL(27, 373, 17), // "setFPSIsDisplayed"
QT_MOC_LITERAL(28, 391, 7), // "display"
QT_MOC_LITERAL(29, 399, 16), // "setTextIsEnabled"
QT_MOC_LITERAL(30, 416, 6), // "enable"
QT_MOC_LITERAL(31, 423, 17), // "setCameraIsEdited"
QT_MOC_LITERAL(32, 441, 4), // "edit"
QT_MOC_LITERAL(33, 446, 17), // "toggleAxisIsDrawn"
QT_MOC_LITERAL(34, 464, 17), // "toggleGridIsDrawn"
QT_MOC_LITERAL(35, 482, 20), // "toggleFPSIsDisplayed"
QT_MOC_LITERAL(36, 503, 19), // "toggleTextIsEnabled"
QT_MOC_LITERAL(37, 523, 20), // "toggleCameraIsEdited"
QT_MOC_LITERAL(38, 544, 18), // "setBackgroundColor"
QT_MOC_LITERAL(39, 563, 5), // "color"
QT_MOC_LITERAL(40, 569, 18), // "setForegroundColor"
QT_MOC_LITERAL(41, 588, 14), // "setSceneRadius"
QT_MOC_LITERAL(42, 603, 6), // "radius"
QT_MOC_LITERAL(43, 610, 14), // "setSceneCenter"
QT_MOC_LITERAL(44, 625, 14), // "qglviewer::Vec"
QT_MOC_LITERAL(45, 640, 6), // "center"
QT_MOC_LITERAL(46, 647, 19), // "setSceneBoundingBox"
QT_MOC_LITERAL(47, 667, 3), // "min"
QT_MOC_LITERAL(48, 671, 3), // "max"
QT_MOC_LITERAL(49, 675, 15), // "showEntireScene"
QT_MOC_LITERAL(50, 691, 9), // "setCamera"
QT_MOC_LITERAL(51, 701, 23), // "qglviewer::Camera*const"
QT_MOC_LITERAL(52, 725, 6), // "camera"
QT_MOC_LITERAL(53, 732, 19), // "setManipulatedFrame"
QT_MOC_LITERAL(54, 752, 28), // "qglviewer::ManipulatedFrame*"
QT_MOC_LITERAL(55, 781, 5), // "frame"
QT_MOC_LITERAL(56, 787, 15), // "setMouseGrabber"
QT_MOC_LITERAL(57, 803, 13), // "setFullScreen"
QT_MOC_LITERAL(58, 817, 10), // "fullScreen"
QT_MOC_LITERAL(59, 828, 16), // "toggleFullScreen"
QT_MOC_LITERAL(60, 845, 16), // "toggleCameraMode"
QT_MOC_LITERAL(61, 862, 19), // "copyBufferToTexture"
QT_MOC_LITERAL(62, 882, 5), // "GLint"
QT_MOC_LITERAL(63, 888, 6), // "GLenum"
QT_MOC_LITERAL(64, 895, 18), // "setAnimationPeriod"
QT_MOC_LITERAL(65, 914, 6), // "period"
QT_MOC_LITERAL(66, 921, 14), // "startAnimation"
QT_MOC_LITERAL(67, 936, 13), // "stopAnimation"
QT_MOC_LITERAL(68, 950, 7), // "animate"
QT_MOC_LITERAL(69, 958, 15), // "toggleAnimation"
QT_MOC_LITERAL(70, 974, 4), // "help"
QT_MOC_LITERAL(71, 979, 14), // "aboutQGLViewer"
QT_MOC_LITERAL(72, 994, 6), // "select"
QT_MOC_LITERAL(73, 1001, 5), // "event"
QT_MOC_LITERAL(74, 1007, 5), // "point"
QT_MOC_LITERAL(75, 1013, 19), // "setSelectBufferSize"
QT_MOC_LITERAL(76, 1033, 4), // "size"
QT_MOC_LITERAL(77, 1038, 20), // "setSelectRegionWidth"
QT_MOC_LITERAL(78, 1059, 5), // "width"
QT_MOC_LITERAL(79, 1065, 21), // "setSelectRegionHeight"
QT_MOC_LITERAL(80, 1087, 6), // "height"
QT_MOC_LITERAL(81, 1094, 15), // "setSelectedName"
QT_MOC_LITERAL(82, 1110, 2), // "id"
QT_MOC_LITERAL(83, 1113, 11), // "setShortcut"
QT_MOC_LITERAL(84, 1125, 25), // "qglviewer::KeyboardAction"
QT_MOC_LITERAL(85, 1151, 6), // "action"
QT_MOC_LITERAL(86, 1158, 3), // "key"
QT_MOC_LITERAL(87, 1162, 17), // "setKeyDescription"
QT_MOC_LITERAL(88, 1180, 11), // "description"
QT_MOC_LITERAL(89, 1192, 14), // "clearShortcuts"
QT_MOC_LITERAL(90, 1207, 10), // "setPathKey"
QT_MOC_LITERAL(91, 1218, 5), // "index"
QT_MOC_LITERAL(92, 1224, 28), // "setPlayPathKeyboardModifiers"
QT_MOC_LITERAL(93, 1253, 23), // "::Qt::KeyboardModifiers"
QT_MOC_LITERAL(94, 1277, 9), // "modifiers"
QT_MOC_LITERAL(95, 1287, 31), // "setAddKeyFrameKeyboardModifiers"
QT_MOC_LITERAL(96, 1319, 15), // "setMouseBinding"
QT_MOC_LITERAL(97, 1335, 17), // "::Qt::MouseButton"
QT_MOC_LITERAL(98, 1353, 7), // "buttons"
QT_MOC_LITERAL(99, 1361, 23), // "qglviewer::MouseHandler"
QT_MOC_LITERAL(100, 1385, 7), // "handler"
QT_MOC_LITERAL(101, 1393, 22), // "qglviewer::MouseAction"
QT_MOC_LITERAL(102, 1416, 14), // "withConstraint"
QT_MOC_LITERAL(103, 1431, 6), // "button"
QT_MOC_LITERAL(104, 1438, 22), // "qglviewer::ClickAction"
QT_MOC_LITERAL(105, 1461, 11), // "doubleClick"
QT_MOC_LITERAL(106, 1473, 18), // "::Qt::MouseButtons"
QT_MOC_LITERAL(107, 1492, 13), // "buttonsBefore"
QT_MOC_LITERAL(108, 1506, 15), // "setWheelBinding"
QT_MOC_LITERAL(109, 1522, 26), // "setMouseBindingDescription"
QT_MOC_LITERAL(110, 1549, 9), // "::Qt::Key"
QT_MOC_LITERAL(111, 1559, 18), // "clearMouseBindings"
QT_MOC_LITERAL(112, 1578, 18), // "initFromDOMElement"
QT_MOC_LITERAL(113, 1597, 11), // "QDomElement"
QT_MOC_LITERAL(114, 1609, 7), // "element"
QT_MOC_LITERAL(115, 1617, 15), // "saveStateToFile"
QT_MOC_LITERAL(116, 1633, 20), // "restoreStateFromFile"
QT_MOC_LITERAL(117, 1654, 16), // "setStateFileName"
QT_MOC_LITERAL(118, 1671, 4), // "name"
QT_MOC_LITERAL(119, 1676, 16), // "resetVisualHints"
QT_MOC_LITERAL(120, 1693, 17), // "delayedFullScreen"
QT_MOC_LITERAL(121, 1711, 11) // "hideMessage"

    },
    "CGAL::QGLViewer\0viewerInitialized\0\0"
    "drawNeeded\0drawFinished\0automatic\0"
    "animateNeeded\0helpRequired\0"
    "axisIsDrawnChanged\0drawn\0gridIsDrawnChanged\0"
    "FPSIsDisplayedChanged\0displayed\0"
    "textIsEnabledChanged\0enabled\0"
    "cameraIsEditedChanged\0edited\0pointSelected\0"
    "const QMouseEvent*\0e\0mouseGrabberChanged\0"
    "qglviewer::MouseGrabber*\0mouseGrabber\0"
    "needNewContext\0setAxisIsDrawn\0draw\0"
    "setGridIsDrawn\0setFPSIsDisplayed\0"
    "display\0setTextIsEnabled\0enable\0"
    "setCameraIsEdited\0edit\0toggleAxisIsDrawn\0"
    "toggleGridIsDrawn\0toggleFPSIsDisplayed\0"
    "toggleTextIsEnabled\0toggleCameraIsEdited\0"
    "setBackgroundColor\0color\0setForegroundColor\0"
    "setSceneRadius\0radius\0setSceneCenter\0"
    "qglviewer::Vec\0center\0setSceneBoundingBox\0"
    "min\0max\0showEntireScene\0setCamera\0"
    "qglviewer::Camera*const\0camera\0"
    "setManipulatedFrame\0qglviewer::ManipulatedFrame*\0"
    "frame\0setMouseGrabber\0setFullScreen\0"
    "fullScreen\0toggleFullScreen\0"
    "toggleCameraMode\0copyBufferToTexture\0"
    "GLint\0GLenum\0setAnimationPeriod\0period\0"
    "startAnimation\0stopAnimation\0animate\0"
    "toggleAnimation\0help\0aboutQGLViewer\0"
    "select\0event\0point\0setSelectBufferSize\0"
    "size\0setSelectRegionWidth\0width\0"
    "setSelectRegionHeight\0height\0"
    "setSelectedName\0id\0setShortcut\0"
    "qglviewer::KeyboardAction\0action\0key\0"
    "setKeyDescription\0description\0"
    "clearShortcuts\0setPathKey\0index\0"
    "setPlayPathKeyboardModifiers\0"
    "::Qt::KeyboardModifiers\0modifiers\0"
    "setAddKeyFrameKeyboardModifiers\0"
    "setMouseBinding\0::Qt::MouseButton\0"
    "buttons\0qglviewer::MouseHandler\0handler\0"
    "qglviewer::MouseAction\0withConstraint\0"
    "button\0qglviewer::ClickAction\0doubleClick\0"
    "::Qt::MouseButtons\0buttonsBefore\0"
    "setWheelBinding\0setMouseBindingDescription\0"
    "::Qt::Key\0clearMouseBindings\0"
    "initFromDOMElement\0QDomElement\0element\0"
    "saveStateToFile\0restoreStateFromFile\0"
    "setStateFileName\0name\0resetVisualHints\0"
    "delayedFullScreen\0hideMessage"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_CGAL__QGLViewer[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
      91,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
      13,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    0,  469,    2, 0x06 /* Public */,
       3,    0,  470,    2, 0x06 /* Public */,
       4,    1,  471,    2, 0x06 /* Public */,
       6,    0,  474,    2, 0x06 /* Public */,
       7,    0,  475,    2, 0x06 /* Public */,
       8,    1,  476,    2, 0x06 /* Public */,
      10,    1,  479,    2, 0x06 /* Public */,
      11,    1,  482,    2, 0x06 /* Public */,
      13,    1,  485,    2, 0x06 /* Public */,
      15,    1,  488,    2, 0x06 /* Public */,
      17,    1,  491,    2, 0x06 /* Public */,
      20,    1,  494,    2, 0x06 /* Public */,
      23,    0,  497,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
      24,    1,  498,    2, 0x0a /* Public */,
      24,    0,  501,    2, 0x2a /* Public | MethodCloned */,
      26,    1,  502,    2, 0x0a /* Public */,
      26,    0,  505,    2, 0x2a /* Public | MethodCloned */,
      27,    1,  506,    2, 0x0a /* Public */,
      27,    0,  509,    2, 0x2a /* Public | MethodCloned */,
      29,    1,  510,    2, 0x0a /* Public */,
      29,    0,  513,    2, 0x2a /* Public | MethodCloned */,
      31,    1,  514,    2, 0x0a /* Public */,
      31,    0,  517,    2, 0x2a /* Public | MethodCloned */,
      33,    0,  518,    2, 0x0a /* Public */,
      34,    0,  519,    2, 0x0a /* Public */,
      35,    0,  520,    2, 0x0a /* Public */,
      36,    0,  521,    2, 0x0a /* Public */,
      37,    0,  522,    2, 0x0a /* Public */,
      38,    1,  523,    2, 0x0a /* Public */,
      40,    1,  526,    2, 0x0a /* Public */,
      41,    1,  529,    2, 0x0a /* Public */,
      43,    1,  532,    2, 0x0a /* Public */,
      46,    2,  535,    2, 0x0a /* Public */,
      49,    0,  540,    2, 0x0a /* Public */,
      50,    1,  541,    2, 0x0a /* Public */,
      53,    1,  544,    2, 0x0a /* Public */,
      56,    1,  547,    2, 0x0a /* Public */,
      57,    1,  550,    2, 0x0a /* Public */,
      57,    0,  553,    2, 0x2a /* Public | MethodCloned */,
      59,    0,  554,    2, 0x0a /* Public */,
      60,    0,  555,    2, 0x0a /* Public */,
      61,    2,  556,    2, 0x0a /* Public */,
      61,    1,  561,    2, 0x2a /* Public | MethodCloned */,
      64,    1,  564,    2, 0x0a /* Public */,
      66,    0,  567,    2, 0x0a /* Public */,
      67,    0,  568,    2, 0x0a /* Public */,
      68,    0,  569,    2, 0x0a /* Public */,
      69,    0,  570,    2, 0x0a /* Public */,
      70,    0,  571,    2, 0x0a /* Public */,
      71,    0,  572,    2, 0x0a /* Public */,
      72,    1,  573,    2, 0x0a /* Public */,
      72,    1,  576,    2, 0x0a /* Public */,
      75,    1,  579,    2, 0x0a /* Public */,
      77,    1,  582,    2, 0x0a /* Public */,
      79,    1,  585,    2, 0x0a /* Public */,
      81,    1,  588,    2, 0x0a /* Public */,
      83,    2,  591,    2, 0x0a /* Public */,
      87,    2,  596,    2, 0x0a /* Public */,
      89,    0,  601,    2, 0x0a /* Public */,
      90,    2,  602,    2, 0x0a /* Public */,
      90,    1,  607,    2, 0x2a /* Public | MethodCloned */,
      92,    1,  610,    2, 0x0a /* Public */,
      95,    1,  613,    2, 0x0a /* Public */,
      96,    5,  616,    2, 0x0a /* Public */,
      96,    4,  627,    2, 0x2a /* Public | MethodCloned */,
      96,    5,  636,    2, 0x0a /* Public */,
      96,    4,  647,    2, 0x2a /* Public | MethodCloned */,
      96,    3,  656,    2, 0x2a /* Public | MethodCloned */,
     108,    4,  663,    2, 0x0a /* Public */,
     108,    3,  672,    2, 0x2a /* Public | MethodCloned */,
     109,    5,  679,    2, 0x0a /* Public */,
     109,    4,  690,    2, 0x2a /* Public | MethodCloned */,
     109,    3,  699,    2, 0x2a /* Public | MethodCloned */,
      96,    6,  706,    2, 0x0a /* Public */,
      96,    5,  719,    2, 0x2a /* Public | MethodCloned */,
      96,    6,  730,    2, 0x0a /* Public */,
      96,    5,  743,    2, 0x2a /* Public | MethodCloned */,
      96,    4,  754,    2, 0x2a /* Public | MethodCloned */,
     108,    5,  763,    2, 0x0a /* Public */,
     108,    4,  774,    2, 0x2a /* Public | MethodCloned */,
     109,    6,  783,    2, 0x0a /* Public */,
     109,    5,  796,    2, 0x2a /* Public | MethodCloned */,
     109,    4,  807,    2, 0x2a /* Public | MethodCloned */,
     111,    0,  816,    2, 0x0a /* Public */,
     112,    1,  817,    2, 0x0a /* Public */,
     115,    0,  820,    2, 0x0a /* Public */,
     116,    0,  821,    2, 0x0a /* Public */,
     117,    1,  822,    2, 0x0a /* Public */,
     119,    0,  825,    2, 0x0a /* Public */,
     120,    0,  826,    2, 0x08 /* Private */,
     121,    0,  827,    2, 0x08 /* Private */,

 // signals: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Bool,    5,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Bool,    9,
    QMetaType::Void, QMetaType::Bool,    9,
    QMetaType::Void, QMetaType::Bool,   12,
    QMetaType::Void, QMetaType::Bool,   14,
    QMetaType::Void, QMetaType::Bool,   16,
    QMetaType::Void, 0x80000000 | 18,   19,
    QMetaType::Void, 0x80000000 | 21,   22,
    QMetaType::Void,

 // slots: parameters
    QMetaType::Void, QMetaType::Bool,   25,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Bool,   25,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Bool,   28,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Bool,   30,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Bool,   32,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::QColor,   39,
    QMetaType::Void, QMetaType::QColor,   39,
    QMetaType::Void, QMetaType::QReal,   42,
    QMetaType::Void, 0x80000000 | 44,   45,
    QMetaType::Void, 0x80000000 | 44, 0x80000000 | 44,   47,   48,
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 51,   52,
    QMetaType::Void, 0x80000000 | 54,   55,
    QMetaType::Void, 0x80000000 | 21,   22,
    QMetaType::Void, QMetaType::Bool,   58,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 62, 0x80000000 | 63,    2,    2,
    QMetaType::Void, 0x80000000 | 62,    2,
    QMetaType::Void, QMetaType::Int,   65,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 18,   73,
    QMetaType::Void, QMetaType::QPoint,   74,
    QMetaType::Void, QMetaType::Int,   76,
    QMetaType::Void, QMetaType::Int,   78,
    QMetaType::Void, QMetaType::Int,   80,
    QMetaType::Void, QMetaType::Int,   82,
    QMetaType::Void, 0x80000000 | 84, QMetaType::UInt,   85,   86,
    QMetaType::Void, QMetaType::UInt, QMetaType::QString,   86,   88,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Int, QMetaType::UInt,   86,   91,
    QMetaType::Void, QMetaType::Int,   86,
    QMetaType::Void, 0x80000000 | 93,   94,
    QMetaType::Void, 0x80000000 | 93,   94,
    QMetaType::Void, 0x80000000 | 93, 0x80000000 | 97, 0x80000000 | 99, 0x80000000 | 101, QMetaType::Bool,   94,   98,  100,   85,  102,
    QMetaType::Void, 0x80000000 | 93, 0x80000000 | 97, 0x80000000 | 99, 0x80000000 | 101,   94,   98,  100,   85,
    QMetaType::Void, 0x80000000 | 93, 0x80000000 | 97, 0x80000000 | 104, QMetaType::Bool, 0x80000000 | 106,   94,  103,   85,  105,  107,
    QMetaType::Void, 0x80000000 | 93, 0x80000000 | 97, 0x80000000 | 104, QMetaType::Bool,   94,  103,   85,  105,
    QMetaType::Void, 0x80000000 | 93, 0x80000000 | 97, 0x80000000 | 104,   94,  103,   85,
    QMetaType::Void, 0x80000000 | 93, 0x80000000 | 99, 0x80000000 | 101, QMetaType::Bool,   94,  100,   85,  102,
    QMetaType::Void, 0x80000000 | 93, 0x80000000 | 99, 0x80000000 | 101,   94,  100,   85,
    QMetaType::Void, 0x80000000 | 93, 0x80000000 | 97, QMetaType::QString, QMetaType::Bool, 0x80000000 | 106,   94,  103,   88,  105,  107,
    QMetaType::Void, 0x80000000 | 93, 0x80000000 | 97, QMetaType::QString, QMetaType::Bool,   94,  103,   88,  105,
    QMetaType::Void, 0x80000000 | 93, 0x80000000 | 97, QMetaType::QString,   94,  103,   88,
    QMetaType::Void, 0x80000000 | 110, 0x80000000 | 93, 0x80000000 | 97, 0x80000000 | 99, 0x80000000 | 101, QMetaType::Bool,   86,   94,   98,  100,   85,  102,
    QMetaType::Void, 0x80000000 | 110, 0x80000000 | 93, 0x80000000 | 97, 0x80000000 | 99, 0x80000000 | 101,   86,   94,   98,  100,   85,
    QMetaType::Void, 0x80000000 | 110, 0x80000000 | 93, 0x80000000 | 97, 0x80000000 | 104, QMetaType::Bool, 0x80000000 | 106,   86,   94,  103,   85,  105,  107,
    QMetaType::Void, 0x80000000 | 110, 0x80000000 | 93, 0x80000000 | 97, 0x80000000 | 104, QMetaType::Bool,   86,   94,  103,   85,  105,
    QMetaType::Void, 0x80000000 | 110, 0x80000000 | 93, 0x80000000 | 97, 0x80000000 | 104,   86,   94,  103,   85,
    QMetaType::Void, 0x80000000 | 110, 0x80000000 | 93, 0x80000000 | 99, 0x80000000 | 101, QMetaType::Bool,   86,   94,  100,   85,  102,
    QMetaType::Void, 0x80000000 | 110, 0x80000000 | 93, 0x80000000 | 99, 0x80000000 | 101,   86,   94,  100,   85,
    QMetaType::Void, 0x80000000 | 110, 0x80000000 | 93, 0x80000000 | 97, QMetaType::QString, QMetaType::Bool, 0x80000000 | 106,   86,   94,  103,   88,  105,  107,
    QMetaType::Void, 0x80000000 | 110, 0x80000000 | 93, 0x80000000 | 97, QMetaType::QString, QMetaType::Bool,   86,   94,  103,   88,  105,
    QMetaType::Void, 0x80000000 | 110, 0x80000000 | 93, 0x80000000 | 97, QMetaType::QString,   86,   94,  103,   88,
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 113,  114,
    QMetaType::Void,
    QMetaType::Bool,
    QMetaType::Void, QMetaType::QString,  118,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

void CGAL::QGLViewer::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        QGLViewer *_t = static_cast<QGLViewer *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->viewerInitialized(); break;
        case 1: _t->drawNeeded(); break;
        case 2: _t->drawFinished((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 3: _t->animateNeeded(); break;
        case 4: _t->helpRequired(); break;
        case 5: _t->axisIsDrawnChanged((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 6: _t->gridIsDrawnChanged((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 7: _t->FPSIsDisplayedChanged((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 8: _t->textIsEnabledChanged((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 9: _t->cameraIsEditedChanged((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 10: _t->pointSelected((*reinterpret_cast< const QMouseEvent*(*)>(_a[1]))); break;
        case 11: _t->mouseGrabberChanged((*reinterpret_cast< qglviewer::MouseGrabber*(*)>(_a[1]))); break;
        case 12: _t->needNewContext(); break;
        case 13: _t->setAxisIsDrawn((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 14: _t->setAxisIsDrawn(); break;
        case 15: _t->setGridIsDrawn((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 16: _t->setGridIsDrawn(); break;
        case 17: _t->setFPSIsDisplayed((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 18: _t->setFPSIsDisplayed(); break;
        case 19: _t->setTextIsEnabled((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 20: _t->setTextIsEnabled(); break;
        case 21: _t->setCameraIsEdited((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 22: _t->setCameraIsEdited(); break;
        case 23: _t->toggleAxisIsDrawn(); break;
        case 24: _t->toggleGridIsDrawn(); break;
        case 25: _t->toggleFPSIsDisplayed(); break;
        case 26: _t->toggleTextIsEnabled(); break;
        case 27: _t->toggleCameraIsEdited(); break;
        case 28: _t->setBackgroundColor((*reinterpret_cast< const QColor(*)>(_a[1]))); break;
        case 29: _t->setForegroundColor((*reinterpret_cast< const QColor(*)>(_a[1]))); break;
        case 30: _t->setSceneRadius((*reinterpret_cast< qreal(*)>(_a[1]))); break;
        case 31: _t->setSceneCenter((*reinterpret_cast< const qglviewer::Vec(*)>(_a[1]))); break;
        case 32: _t->setSceneBoundingBox((*reinterpret_cast< const qglviewer::Vec(*)>(_a[1])),(*reinterpret_cast< const qglviewer::Vec(*)>(_a[2]))); break;
        case 33: _t->showEntireScene(); break;
        case 34: _t->setCamera((*reinterpret_cast< qglviewer::Camera*const(*)>(_a[1]))); break;
        case 35: _t->setManipulatedFrame((*reinterpret_cast< qglviewer::ManipulatedFrame*(*)>(_a[1]))); break;
        case 36: _t->setMouseGrabber((*reinterpret_cast< qglviewer::MouseGrabber*(*)>(_a[1]))); break;
        case 37: _t->setFullScreen((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 38: _t->setFullScreen(); break;
        case 39: _t->toggleFullScreen(); break;
        case 40: _t->toggleCameraMode(); break;
        case 41: _t->copyBufferToTexture((*reinterpret_cast< GLint(*)>(_a[1])),(*reinterpret_cast< GLenum(*)>(_a[2]))); break;
        case 42: _t->copyBufferToTexture((*reinterpret_cast< GLint(*)>(_a[1]))); break;
        case 43: _t->setAnimationPeriod((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 44: _t->startAnimation(); break;
        case 45: _t->stopAnimation(); break;
        case 46: _t->animate(); break;
        case 47: _t->toggleAnimation(); break;
        case 48: _t->help(); break;
        case 49: _t->aboutQGLViewer(); break;
        case 50: _t->select((*reinterpret_cast< const QMouseEvent*(*)>(_a[1]))); break;
        case 51: _t->select((*reinterpret_cast< const QPoint(*)>(_a[1]))); break;
        case 52: _t->setSelectBufferSize((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 53: _t->setSelectRegionWidth((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 54: _t->setSelectRegionHeight((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 55: _t->setSelectedName((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 56: _t->setShortcut((*reinterpret_cast< qglviewer::KeyboardAction(*)>(_a[1])),(*reinterpret_cast< uint(*)>(_a[2]))); break;
        case 57: _t->setKeyDescription((*reinterpret_cast< uint(*)>(_a[1])),(*reinterpret_cast< QString(*)>(_a[2]))); break;
        case 58: _t->clearShortcuts(); break;
        case 59: _t->setPathKey((*reinterpret_cast< int(*)>(_a[1])),(*reinterpret_cast< uint(*)>(_a[2]))); break;
        case 60: _t->setPathKey((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 61: _t->setPlayPathKeyboardModifiers((*reinterpret_cast< ::Qt::KeyboardModifiers(*)>(_a[1]))); break;
        case 62: _t->setAddKeyFrameKeyboardModifiers((*reinterpret_cast< ::Qt::KeyboardModifiers(*)>(_a[1]))); break;
        case 63: _t->setMouseBinding((*reinterpret_cast< ::Qt::KeyboardModifiers(*)>(_a[1])),(*reinterpret_cast< ::Qt::MouseButton(*)>(_a[2])),(*reinterpret_cast< qglviewer::MouseHandler(*)>(_a[3])),(*reinterpret_cast< qglviewer::MouseAction(*)>(_a[4])),(*reinterpret_cast< bool(*)>(_a[5]))); break;
        case 64: _t->setMouseBinding((*reinterpret_cast< ::Qt::KeyboardModifiers(*)>(_a[1])),(*reinterpret_cast< ::Qt::MouseButton(*)>(_a[2])),(*reinterpret_cast< qglviewer::MouseHandler(*)>(_a[3])),(*reinterpret_cast< qglviewer::MouseAction(*)>(_a[4]))); break;
        case 65: _t->setMouseBinding((*reinterpret_cast< ::Qt::KeyboardModifiers(*)>(_a[1])),(*reinterpret_cast< ::Qt::MouseButton(*)>(_a[2])),(*reinterpret_cast< qglviewer::ClickAction(*)>(_a[3])),(*reinterpret_cast< bool(*)>(_a[4])),(*reinterpret_cast< ::Qt::MouseButtons(*)>(_a[5]))); break;
        case 66: _t->setMouseBinding((*reinterpret_cast< ::Qt::KeyboardModifiers(*)>(_a[1])),(*reinterpret_cast< ::Qt::MouseButton(*)>(_a[2])),(*reinterpret_cast< qglviewer::ClickAction(*)>(_a[3])),(*reinterpret_cast< bool(*)>(_a[4]))); break;
        case 67: _t->setMouseBinding((*reinterpret_cast< ::Qt::KeyboardModifiers(*)>(_a[1])),(*reinterpret_cast< ::Qt::MouseButton(*)>(_a[2])),(*reinterpret_cast< qglviewer::ClickAction(*)>(_a[3]))); break;
        case 68: _t->setWheelBinding((*reinterpret_cast< ::Qt::KeyboardModifiers(*)>(_a[1])),(*reinterpret_cast< qglviewer::MouseHandler(*)>(_a[2])),(*reinterpret_cast< qglviewer::MouseAction(*)>(_a[3])),(*reinterpret_cast< bool(*)>(_a[4]))); break;
        case 69: _t->setWheelBinding((*reinterpret_cast< ::Qt::KeyboardModifiers(*)>(_a[1])),(*reinterpret_cast< qglviewer::MouseHandler(*)>(_a[2])),(*reinterpret_cast< qglviewer::MouseAction(*)>(_a[3]))); break;
        case 70: _t->setMouseBindingDescription((*reinterpret_cast< ::Qt::KeyboardModifiers(*)>(_a[1])),(*reinterpret_cast< ::Qt::MouseButton(*)>(_a[2])),(*reinterpret_cast< QString(*)>(_a[3])),(*reinterpret_cast< bool(*)>(_a[4])),(*reinterpret_cast< ::Qt::MouseButtons(*)>(_a[5]))); break;
        case 71: _t->setMouseBindingDescription((*reinterpret_cast< ::Qt::KeyboardModifiers(*)>(_a[1])),(*reinterpret_cast< ::Qt::MouseButton(*)>(_a[2])),(*reinterpret_cast< QString(*)>(_a[3])),(*reinterpret_cast< bool(*)>(_a[4]))); break;
        case 72: _t->setMouseBindingDescription((*reinterpret_cast< ::Qt::KeyboardModifiers(*)>(_a[1])),(*reinterpret_cast< ::Qt::MouseButton(*)>(_a[2])),(*reinterpret_cast< QString(*)>(_a[3]))); break;
        case 73: _t->setMouseBinding((*reinterpret_cast< ::Qt::Key(*)>(_a[1])),(*reinterpret_cast< ::Qt::KeyboardModifiers(*)>(_a[2])),(*reinterpret_cast< ::Qt::MouseButton(*)>(_a[3])),(*reinterpret_cast< qglviewer::MouseHandler(*)>(_a[4])),(*reinterpret_cast< qglviewer::MouseAction(*)>(_a[5])),(*reinterpret_cast< bool(*)>(_a[6]))); break;
        case 74: _t->setMouseBinding((*reinterpret_cast< ::Qt::Key(*)>(_a[1])),(*reinterpret_cast< ::Qt::KeyboardModifiers(*)>(_a[2])),(*reinterpret_cast< ::Qt::MouseButton(*)>(_a[3])),(*reinterpret_cast< qglviewer::MouseHandler(*)>(_a[4])),(*reinterpret_cast< qglviewer::MouseAction(*)>(_a[5]))); break;
        case 75: _t->setMouseBinding((*reinterpret_cast< ::Qt::Key(*)>(_a[1])),(*reinterpret_cast< ::Qt::KeyboardModifiers(*)>(_a[2])),(*reinterpret_cast< ::Qt::MouseButton(*)>(_a[3])),(*reinterpret_cast< qglviewer::ClickAction(*)>(_a[4])),(*reinterpret_cast< bool(*)>(_a[5])),(*reinterpret_cast< ::Qt::MouseButtons(*)>(_a[6]))); break;
        case 76: _t->setMouseBinding((*reinterpret_cast< ::Qt::Key(*)>(_a[1])),(*reinterpret_cast< ::Qt::KeyboardModifiers(*)>(_a[2])),(*reinterpret_cast< ::Qt::MouseButton(*)>(_a[3])),(*reinterpret_cast< qglviewer::ClickAction(*)>(_a[4])),(*reinterpret_cast< bool(*)>(_a[5]))); break;
        case 77: _t->setMouseBinding((*reinterpret_cast< ::Qt::Key(*)>(_a[1])),(*reinterpret_cast< ::Qt::KeyboardModifiers(*)>(_a[2])),(*reinterpret_cast< ::Qt::MouseButton(*)>(_a[3])),(*reinterpret_cast< qglviewer::ClickAction(*)>(_a[4]))); break;
        case 78: _t->setWheelBinding((*reinterpret_cast< ::Qt::Key(*)>(_a[1])),(*reinterpret_cast< ::Qt::KeyboardModifiers(*)>(_a[2])),(*reinterpret_cast< qglviewer::MouseHandler(*)>(_a[3])),(*reinterpret_cast< qglviewer::MouseAction(*)>(_a[4])),(*reinterpret_cast< bool(*)>(_a[5]))); break;
        case 79: _t->setWheelBinding((*reinterpret_cast< ::Qt::Key(*)>(_a[1])),(*reinterpret_cast< ::Qt::KeyboardModifiers(*)>(_a[2])),(*reinterpret_cast< qglviewer::MouseHandler(*)>(_a[3])),(*reinterpret_cast< qglviewer::MouseAction(*)>(_a[4]))); break;
        case 80: _t->setMouseBindingDescription((*reinterpret_cast< ::Qt::Key(*)>(_a[1])),(*reinterpret_cast< ::Qt::KeyboardModifiers(*)>(_a[2])),(*reinterpret_cast< ::Qt::MouseButton(*)>(_a[3])),(*reinterpret_cast< QString(*)>(_a[4])),(*reinterpret_cast< bool(*)>(_a[5])),(*reinterpret_cast< ::Qt::MouseButtons(*)>(_a[6]))); break;
        case 81: _t->setMouseBindingDescription((*reinterpret_cast< ::Qt::Key(*)>(_a[1])),(*reinterpret_cast< ::Qt::KeyboardModifiers(*)>(_a[2])),(*reinterpret_cast< ::Qt::MouseButton(*)>(_a[3])),(*reinterpret_cast< QString(*)>(_a[4])),(*reinterpret_cast< bool(*)>(_a[5]))); break;
        case 82: _t->setMouseBindingDescription((*reinterpret_cast< ::Qt::Key(*)>(_a[1])),(*reinterpret_cast< ::Qt::KeyboardModifiers(*)>(_a[2])),(*reinterpret_cast< ::Qt::MouseButton(*)>(_a[3])),(*reinterpret_cast< QString(*)>(_a[4]))); break;
        case 83: _t->clearMouseBindings(); break;
        case 84: _t->initFromDOMElement((*reinterpret_cast< const QDomElement(*)>(_a[1]))); break;
        case 85: _t->saveStateToFile(); break;
        case 86: { bool _r = _t->restoreStateFromFile();
            if (_a[0]) *reinterpret_cast< bool*>(_a[0]) = _r; }  break;
        case 87: _t->setStateFileName((*reinterpret_cast< const QString(*)>(_a[1]))); break;
        case 88: _t->resetVisualHints(); break;
        case 89: _t->delayedFullScreen(); break;
        case 90: _t->hideMessage(); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        void **func = reinterpret_cast<void **>(_a[1]);
        {
            typedef void (QGLViewer::*_t)();
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&QGLViewer::viewerInitialized)) {
                *result = 0;
            }
        }
        {
            typedef void (QGLViewer::*_t)();
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&QGLViewer::drawNeeded)) {
                *result = 1;
            }
        }
        {
            typedef void (QGLViewer::*_t)(bool );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&QGLViewer::drawFinished)) {
                *result = 2;
            }
        }
        {
            typedef void (QGLViewer::*_t)();
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&QGLViewer::animateNeeded)) {
                *result = 3;
            }
        }
        {
            typedef void (QGLViewer::*_t)();
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&QGLViewer::helpRequired)) {
                *result = 4;
            }
        }
        {
            typedef void (QGLViewer::*_t)(bool );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&QGLViewer::axisIsDrawnChanged)) {
                *result = 5;
            }
        }
        {
            typedef void (QGLViewer::*_t)(bool );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&QGLViewer::gridIsDrawnChanged)) {
                *result = 6;
            }
        }
        {
            typedef void (QGLViewer::*_t)(bool );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&QGLViewer::FPSIsDisplayedChanged)) {
                *result = 7;
            }
        }
        {
            typedef void (QGLViewer::*_t)(bool );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&QGLViewer::textIsEnabledChanged)) {
                *result = 8;
            }
        }
        {
            typedef void (QGLViewer::*_t)(bool );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&QGLViewer::cameraIsEditedChanged)) {
                *result = 9;
            }
        }
        {
            typedef void (QGLViewer::*_t)(const QMouseEvent * );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&QGLViewer::pointSelected)) {
                *result = 10;
            }
        }
        {
            typedef void (QGLViewer::*_t)(qglviewer::MouseGrabber * );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&QGLViewer::mouseGrabberChanged)) {
                *result = 11;
            }
        }
        {
            typedef void (QGLViewer::*_t)();
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&QGLViewer::needNewContext)) {
                *result = 12;
            }
        }
    }
}

const QMetaObject CGAL::QGLViewer::staticMetaObject = {
    { &QOpenGLWidget::staticMetaObject, qt_meta_stringdata_CGAL__QGLViewer.data,
      qt_meta_data_CGAL__QGLViewer,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *CGAL::QGLViewer::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *CGAL::QGLViewer::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_CGAL__QGLViewer.stringdata0))
        return static_cast<void*>(const_cast< QGLViewer*>(this));
    if (!strcmp(_clname, "QOpenGLFunctions"))
        return static_cast< QOpenGLFunctions*>(const_cast< QGLViewer*>(this));
    return QOpenGLWidget::qt_metacast(_clname);
}

int CGAL::QGLViewer::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QOpenGLWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 91)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 91;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 91)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 91;
    }
    return _id;
}

// SIGNAL 0
void CGAL::QGLViewer::viewerInitialized()
{
    QMetaObject::activate(this, &staticMetaObject, 0, Q_NULLPTR);
}

// SIGNAL 1
void CGAL::QGLViewer::drawNeeded()
{
    QMetaObject::activate(this, &staticMetaObject, 1, Q_NULLPTR);
}

// SIGNAL 2
void CGAL::QGLViewer::drawFinished(bool _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}

// SIGNAL 3
void CGAL::QGLViewer::animateNeeded()
{
    QMetaObject::activate(this, &staticMetaObject, 3, Q_NULLPTR);
}

// SIGNAL 4
void CGAL::QGLViewer::helpRequired()
{
    QMetaObject::activate(this, &staticMetaObject, 4, Q_NULLPTR);
}

// SIGNAL 5
void CGAL::QGLViewer::axisIsDrawnChanged(bool _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 5, _a);
}

// SIGNAL 6
void CGAL::QGLViewer::gridIsDrawnChanged(bool _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 6, _a);
}

// SIGNAL 7
void CGAL::QGLViewer::FPSIsDisplayedChanged(bool _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 7, _a);
}

// SIGNAL 8
void CGAL::QGLViewer::textIsEnabledChanged(bool _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 8, _a);
}

// SIGNAL 9
void CGAL::QGLViewer::cameraIsEditedChanged(bool _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 9, _a);
}

// SIGNAL 10
void CGAL::QGLViewer::pointSelected(const QMouseEvent * _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 10, _a);
}

// SIGNAL 11
void CGAL::QGLViewer::mouseGrabberChanged(qglviewer::MouseGrabber * _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 11, _a);
}

// SIGNAL 12
void CGAL::QGLViewer::needNewContext()
{
    QMetaObject::activate(this, &staticMetaObject, 12, Q_NULLPTR);
}
QT_END_MOC_NAMESPACE
