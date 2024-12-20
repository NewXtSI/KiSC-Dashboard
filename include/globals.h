#ifndef GLOBALS_H
#define GLOBALS_H

#define ESP32DEBUGGING
#include <ESP32Logger.h>

#define MOTOR_FLYWHEEL -2222

#define SERIAL_OUTPUT_ENABLED       1
#define SERIAL_DEFAULT_LOGLEVEL     Debug
// Untersteps: Enabled bei 0, disabled bei 1

#define DBG_ENABLED                 0           // 
#define DBG_DISABLED                1

#define SERIAL_DEBUG_KISCMESSAGES   DBG_DISABLED
#define SERIAL_DEBUG_GUI            DBG_ENABLED
#define SERIAL_DEBUG_CONTROLLER     DBG_ENABLED
#define SERIAL_DEBUG_CONTROLLER_CALC  DBG_ENABLED

#endif