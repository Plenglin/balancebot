#pragma once

#include "ioconstants.hpp"

#ifdef LOGLEVEL_DEBUG
#define DEBUG
#define LOGLEVEL_INFO
#define LOG_D(a) Serial.print(OUT_ARG_LOG"D"); Serial.println(a);
#else
#define LOG_D(a)
#endif

#ifdef LOGLEVEL_INFO
#define LOGLEVEL_WARN
#define LOG_I(a) Serial.print(OUT_ARG_LOG"I"); Serial.println(a);
#else
#define LOG_I(a)
#endif

#ifdef LOGLEVEL_WARN
#define LOGLEVEL_ERROR
#define LOG_W(a) Serial.print(OUT_ARG_LOG"W"); Serial.println(a);
#else
#define LOG_W(a)
#endif

#ifdef LOGLEVEL_ERROR
#define LOGLEVEL_FATAL
#define LOG_E(a) Serial.print(OUT_ARG_LOG"W"); Serial.println(a);
#else
#define LOG_E(a)
#endif

#ifdef LOGLEVEL_FATAL
#define LOG_F(a) Serial.print(OUT_ARG_LOG"F"); Serial.println(a);
#else
#define LOG_F(a)
#endif

