#ifndef QCAR2_INTERFACES__VISIBILITY_CONTROL_H_
#define QCAR2_INTERFACES__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define QCAR2_INTERFACES_EXPORT __attribute__ ((dllexport))
    #define QCAR2_INTERFACES_IMPORT __attribute__ ((dllimport))
  #else
    #define QCAR2_INTERFACES_EXPORT __declspec(dllexport)
    #define QCAR2_INTERFACES_IMPORT __declspec(dllimport)
  #endif
  #ifdef QCAR2_INTERFACES_BUILDING_LIBRARY
    #define QCAR2_INTERFACES_PUBLIC QCAR2_INTERFACES_EXPORT
  #else
    #define QCAR2_INTERFACES_PUBLIC QCAR2_INTERFACES_IMPORT
  #endif
  #define QCAR2_INTERFACES_PUBLIC_TYPE QCAR2_INTERFACES_PUBLIC
  #define QCAR2_INTERFACES_LOCAL
#else
  #define QCAR2_INTERFACES_EXPORT __attribute__ ((visibility("default")))
  #define QCAR2_INTERFACES_IMPORT
  #if __GNUC__ >= 4
    #define QCAR2_INTERFACES_PUBLIC __attribute__ ((visibility("default")))
    #define QCAR2_INTERFACES_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define QCAR2_INTERFACES_PUBLIC
    #define QCAR2_INTERFACES_LOCAL
  #endif
  #define QCAR2_INTERFACES_PUBLIC_TYPE
#endif
#endif  // QCAR2_INTERFACES__VISIBILITY_CONTROL_H_
// Generated 16-Dec-2025 14:57:37
 