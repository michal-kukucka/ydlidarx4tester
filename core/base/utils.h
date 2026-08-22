
#pragma once

#if defined(WIN32) || defined(_WIN32)
#ifdef ydlidar_IMPORTS
#define YDLIDAR_API __declspec(dllimport)
#else
#ifdef ydlidarStatic_IMPORTS
#define YDLIDAR_API
#else

#define YDLIDAR_API __declspec(dllexport)
#endif // YDLIDAR_STATIC_EXPORTS
#endif

#else
#define YDLIDAR_API
#endif // Windows
