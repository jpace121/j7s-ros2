#ifndef BLINKT_INTERFACE__VISIBILITY_CONTROL_H_
#define BLINKT_INTERFACE__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define BLINKT_INTERFACE_EXPORT __attribute__((dllexport))
#define BLINKT_INTERFACE_IMPORT __attribute__((dllimport))
#else
#define BLINKT_INTERFACE_EXPORT __declspec(dllexport)
#define BLINKT_INTERFACE_IMPORT __declspec(dllimport)
#endif
#ifdef BLINKT_INTERFACE_BUILDING_LIBRARY
#define BLINKT_INTERFACE_PUBLIC BLINKT_INTERFACE_EXPORT
#else
#define BLINKT_INTERFACE_PUBLIC BLINKT_INTERFACE_IMPORT
#endif
#define BLINKT_INTERFACE_PUBLIC_TYPE BLINKT_INTERFACE_PUBLIC
#define BLINKT_INTERFACE_LOCAL
#else
#define BLINKT_INTERFACE_EXPORT __attribute__((visibility("default")))
#define BLINKT_INTERFACE_IMPORT
#if __GNUC__ >= 4
#define BLINKT_INTERFACE_PUBLIC __attribute__((visibility("default")))
#define BLINKT_INTERFACE_LOCAL __attribute__((visibility("hidden")))
#else
#define BLINKT_INTERFACE_PUBLIC
#define BLINKT_INTERFACE_LOCAL
#endif
#define BLINKT_INTERFACE_PUBLIC_TYPE
#endif

#endif  // BLINKT_INTERFACE__VISIBILITY_CONTROL_H_
