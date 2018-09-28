#include "params.h"

#pragma once

#ifdef LOGLEVEL_DEBUG
#define LOGLEVEL_INFO
#define LOG_D(a) Serial.print(ARG_DEBUG"D"); Serial.println(a);
#else
#define LOG_D(a)
#endif

#ifdef LOGLEVEL_INFO
#define LOGLEVEL_WARN
#define LOG_I(a) Serial.print(ARG_DEBUG"I"); Serial.println(a);
#else
#define LOG_I(a)
#endif

#ifdef LOGLEVEL_WARN
#define LOGLEVEL_ERROR
#define LOG_W(a) Serial.print(ARG_DEBUG"W"); Serial.println(a);
#else
#define LOG_W(a)
#endif

#ifdef LOGLEVEL_ERROR
#define LOG_E(a) Serial.print(ARG_DEBUG"W"); Serial.println(a);
#else
#define LOG_E(a)
#endif
