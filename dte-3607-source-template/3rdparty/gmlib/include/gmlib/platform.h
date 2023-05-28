#ifndef GM2_PLATFORM_H
#define GM2_PLATFORM_H


#ifdef _MSVC_LANG

// Alternate spelling of operator keywords
#include <ciso646>

#endif   // _MSVC_LANG




#undef EnumSwitchEONVFReturnGuard
#if __clang__
#define EnumSwitchEONVFReturnGuard {}
#elif _MSC_VER
#define EnumSwitchEONVFReturnGuard return {}
#elif __GNUC__
#define EnumSwitchEONVFReturnGuard return {}
#endif   //

#undef EnumSwitchEONVFReturnTypeGuard
#if __clang__
#define EnumSwitchEONVFReturnTypeGuard(Typename) {}
#elif _MSC_VER
#define EnumSwitchEONVFReturnTypeGuard(Typename) return Typename{}
#elif __GNUC__
#define EnumSwitchEONVFReturnTypeGuard(Typename) return Typename()
#endif   //




#endif   // GM2_PLATFORM_H
