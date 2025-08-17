#ifndef CROBOT_CONTROL__VISIBILITY_CONTROL_H_
#define CROBOT_CONTROL__VISIBILITY_CONTROL_H_

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define CROBOT_CONTROL_EXPORT __attribute__((dllexport))
#define CROBOT_CONTROL_IMPORT __attribute__((dllimport))
#else
#define CROBOT_CONTROL_EXPORT __declspec(dllexport)
#define CROBOT_CONTROL_IMPORT __declspec(dllimport)
#endif
#ifdef CROBOT_CONTROL_BUILDING_DLL
#define CROBOT_CONTROL_PUBLIC CROBOT_CONTROL_EXPORT
#else
#define CROBOT_CONTROL_PUBLIC CROBOT_CONTROL_IMPORT
#endif
#define CROBOT_CONTROL_PUBLIC_TYPE CROBOT_CONTROL_PUBLIC
#define CROBOT_CONTROL_LOCAL
#else
#define CROBOT_CONTROL_EXPORT __attribute__((visibility("default")))
#define CROBOT_CONTROL_IMPORT
#if __GNUC__ >= 4
#define CROBOT_CONTROL_PUBLIC __attribute__((visibility("default")))
#define CROBOT_CONTROL_LOCAL __attribute__((visibility("hidden")))
#else
#define CROBOT_CONTROL_PUBLIC
#define CROBOT_CONTROL_LOCAL
#endif
#define CROBOT_CONTROL_PUBLIC_TYPE
#endif

#endif