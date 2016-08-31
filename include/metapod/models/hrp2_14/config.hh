// Copyright 2014
// If you read this file you should have read and been given a copy of the closed license concerning HRP2
//

// This file has been generated by the metapod robotbuilder library.
#ifndef METAPOD_HRP2_14_CONFIG_HH
# define METAPOD_HRP2_14_CONFIG_HH

// Handle portable symbol export.
// Defining manually which symbol should be exported is required
// under Windows whether MinGW or MSVC is used.
//
// The headers then have to be able to work in two different modes:
// - dllexport when one is building the library,
// - dllimport for clients using the library.
//
// On Linux, set the visibility accordingly. If C++ symbol visibility
// is handled by the compiler, see: http://gcc.gnu.org/wiki/Visibility
# if defined _WIN32 || defined __CYGWIN__
// On Microsoft Windows, use dllimport and dllexport to tag symbols.
#  define METAPOD_HRP2_14_DLLIMPORT __declspec(dllimport)
#  define METAPOD_HRP2_14_DLLEXPORT __declspec(dllexport)
#  define METAPOD_HRP2_14_DLLLOCAL
# else
// On Linux, for GCC >= 4, tag symbols using GCC extension.
#  if __GNUC__ >= 4
#   define METAPOD_HRP2_14_DLLIMPORT __attribute__ ((visibility("default")))
#   define METAPOD_HRP2_14_DLLEXPORT __attribute__ ((visibility("default")))
#   define METAPOD_HRP2_14_DLLLOCAL  __attribute__ ((visibility("hidden")))
#  else
// Otherwise (GCC < 4 or another compiler is used), export everything.
#   define METAPOD_HRP2_14_DLLIMPORT
#   define METAPOD_HRP2_14_DLLEXPORT
#   define METAPOD_HRP2_14_DLLLOCAL
#  endif // __GNUC__ >= 4
# endif // defined _WIN32 || defined __CYGWIN__

# ifdef METAPOD_HRP2_14_STATIC
// If one is using the library statically, get rid of
// extra information.
#  define METAPOD_HRP2_14_DLLAPI
#  define METAPOD_HRP2_14_LOCAL
# else
// Depending on whether one is building or using the
// library define DLLAPI to import or export.
#  ifdef metapod_hrp2_14_EXPORTS
#   define METAPOD_HRP2_14_DLLAPI METAPOD_HRP2_14_DLLEXPORT
#  else
#   define METAPOD_HRP2_14_DLLAPI METAPOD_HRP2_14_DLLIMPORT
#  endif // METAPOD_HRP2_14_EXPORTS
#  define METAPOD_HRP2_14_LOCAL METAPOD_HRP2_14_DLLLOCAL
# endif // METAPOD_HRP2_14_STATIC
#endif //! METAPOD_HRP2_14_CONFIG_HH

