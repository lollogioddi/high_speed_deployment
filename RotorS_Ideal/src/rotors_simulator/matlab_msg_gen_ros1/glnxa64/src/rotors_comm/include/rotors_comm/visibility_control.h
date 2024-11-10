#ifndef ROTORS_COMM__VISIBILITY_CONTROL_H_
#define ROTORS_COMM__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define ROTORS_COMM_EXPORT __attribute__ ((dllexport))
    #define ROTORS_COMM_IMPORT __attribute__ ((dllimport))
  #else
    #define ROTORS_COMM_EXPORT __declspec(dllexport)
    #define ROTORS_COMM_IMPORT __declspec(dllimport)
  #endif
  #ifdef ROTORS_COMM_BUILDING_LIBRARY
    #define ROTORS_COMM_PUBLIC ROTORS_COMM_EXPORT
  #else
    #define ROTORS_COMM_PUBLIC ROTORS_COMM_IMPORT
  #endif
  #define ROTORS_COMM_PUBLIC_TYPE ROTORS_COMM_PUBLIC
  #define ROTORS_COMM_LOCAL
#else
  #define ROTORS_COMM_EXPORT __attribute__ ((visibility("default")))
  #define ROTORS_COMM_IMPORT
  #if __GNUC__ >= 4
    #define ROTORS_COMM_PUBLIC __attribute__ ((visibility("default")))
    #define ROTORS_COMM_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define ROTORS_COMM_PUBLIC
    #define ROTORS_COMM_LOCAL
  #endif
  #define ROTORS_COMM_PUBLIC_TYPE
#endif
#endif  // ROTORS_COMM__VISIBILITY_CONTROL_H_
// Generated 17-Jun-2024 17:35:18
// Copyright 2019-2020 The MathWorks, Inc.
