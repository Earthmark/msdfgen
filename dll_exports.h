#pragma once

#ifdef MSDFGEN_DLL
#if defined(WIN32) || defined(_WIN32) || defined(__WIN32__) || defined(__NT__)
#define DLL_EXPORT extern "C" __declspec(dllexport)
#define DLL_API __cdecl
#else
#define DLL_EXPORT extern "C"
#define DLL_API
#endif
#else
#define DLL_EXPORT 
#define DLL_API
#endif
