#-------------------------------------------------
#
# Project created by QtCreator 2018-08-10T12:09:14
#
#-------------------------------------------------

QT       += core gui widgets

TARGET = formation_gui
TEMPLATE = app

# The following define makes your compiler emit warnings if you use
# any feature of Qt which has been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

CONFIG += c++11

SOURCES += \
        main.cpp \
        mainwindow.cpp

HEADERS += \
        mainwindow.h

FORMS += \
        mainwindow.ui

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target


win32:CONFIG(release, debug|release): LIBS += -L$$PWD/../../../../../../usr/local/lib/release/ -ldronecode_sdk
else:win32:CONFIG(debug, debug|release): LIBS += -L$$PWD/../../../../../../usr/local/lib/debug/ -ldronecode_sdk
else:unix: LIBS += -L$$PWD/../../../../../../usr/local/lib/ -ldronecode_sdk

INCLUDEPATH += $$PWD/../../../../../../usr/local/include
DEPENDPATH += $$PWD/../../../../../../usr/local/include

win32-g++:CONFIG(release, debug|release): PRE_TARGETDEPS += $$PWD/../../../../../../usr/local/lib/release/libdronecode_sdk.a
else:win32-g++:CONFIG(debug, debug|release): PRE_TARGETDEPS += $$PWD/../../../../../../usr/local/lib/debug/libdronecode_sdk.a
else:win32:!win32-g++:CONFIG(release, debug|release): PRE_TARGETDEPS += $$PWD/../../../../../../usr/local/lib/release/dronecode_sdk.lib
else:win32:!win32-g++:CONFIG(debug, debug|release): PRE_TARGETDEPS += $$PWD/../../../../../../usr/local/lib/debug/dronecode_sdk.lib
else:unix: PRE_TARGETDEPS += $$PWD/../../../../../../usr/local/lib/libdronecode_sdk.a

win32:CONFIG(release, debug|release): LIBS += -L$$PWD/../../../../../../usr/local/lib/release/ -ldronecode_sdk_action
else:win32:CONFIG(debug, debug|release): LIBS += -L$$PWD/../../../../../../usr/local/lib/debug/ -ldronecode_sdk_action
else:unix: LIBS += -L$$PWD/../../../../../../usr/local/lib/ -ldronecode_sdk_action

INCLUDEPATH += $$PWD/../../../../../../usr/local/include
DEPENDPATH += $$PWD/../../../../../../usr/local/include

win32:CONFIG(release, debug|release): LIBS += -L$$PWD/../../../../../../usr/local/lib/release/ -ldronecode_sdk_telemetry
else:win32:CONFIG(debug, debug|release): LIBS += -L$$PWD/../../../../../../usr/local/lib/debug/ -ldronecode_sdk_telemetry
else:unix: LIBS += -L$$PWD/../../../../../../usr/local/lib/ -ldronecode_sdk_telemetry

INCLUDEPATH += $$PWD/../../../../../../usr/local/include
DEPENDPATH += $$PWD/../../../../../../usr/local/include

# unix|win32: LIBS += -ldronecode_sdk_telemetry
