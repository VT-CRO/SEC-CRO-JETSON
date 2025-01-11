#ifndef CROBOT_NAVIGATION_CPP__VISIBILITY_CONTROL_H_
#define CROBOT_NAVIGATION_CPP__VISIBILITY_CONTROL_H_

#ifdef __cplusplus
extern "C"
{
#endif

    #if defined _WIN32 || defined __CYGWIN__
        #ifdef __GNUC__
            #define CROBOT_NAVIGATION_CPP_EXPORT __attribute__ ((dllexport))
            #define CROBOT_NAVIGATION_CPP_IMPORT __attribute__ ((dllimport))
        #else
            #define CROBOT_NAVIGATION_CPP_EXPORT __declspec(dllexport)
            #define CROBOT_NAVIGATION_CPP_IMPORT __declspec(dllimport)
        #endif
        #ifdef CROBOT_NAVIGATION_CPP_BUILDING_DLL
            #define CROBOT_NAVIGATION_CPP_PUBLIC CROBOT_NAVIGATION_CPP_EXPORT
        #else
            #define CROBOT_NAVIGATION_CPP_PUBLIC CROBOT_NAVIGATION_CPP_IMPORT
        #endif
        #define CROBOT_NAVIGATION_CPP_PUBLIC_TYPE CROBOT_NAVIGATION_CPP_PUBLIC
        #define CROBOT_NAVIGATION_CPP_LOCAL
    #else
        #define CROBOT_NAVIGATION_CPP_EXPORT __attribute__ ((visibility("default")))
        #define CROBOT_NAVIGATION_CPP_IMPORT
        #if __GNUC__ >= 4
            #define CROBOT_NAVIGATION_CPP_PUBLIC __attribute__ ((visibility("default")))
            #define CROBOT_NAVIGATION_CPP_LOCAL __attribute__ ((visibility("hidden")))
        #else
            #define CROBOT_NAVIGATION_CPP_PUBLIC
            #define CROBOT_NAVIGATION_CPP_LOCAL
        #endif
        #define CROBOT_NAVIGATION_CPP_PUBLIC_TYPE
    #endif

#ifdef __cplusplus
}
#endif