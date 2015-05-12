/*  +---------------------------------------------------------------------------+
 *  |                                                                           |
 *  |               https://sites.google.com/site/bellonemauro/                 |
 *  |                                                                           |
 *  | Copyright (c) 2015, - All rights reserved.                                |
 *  | Authors: Mauro Bellone                                                    |
 *  | Released under BDS License.                                               |
 *  +---------------------------------------------------------------------------+ */

#ifndef __VREPREMOTEAPI_EXPORT_
#define __VREPREMOTEAPI_EXPORT_

#if defined(_MSC_VER) || defined(__CYGWIN__) || defined(__MINGW32__) || defined( __BCPLUSPLUS__)  || defined( __MWERKS__)
#  ifdef VREPREMOTEAPI
#    define VREPREMOTEAPI_EXPORT __declspec(dllexport)
#  else
#    define VREPREMOTEAPI_EXPORT __declspec(dllimport)
#  endif
#else
#  define VREPREMOTEAPI_EXPORT
#endif

#endif // __VREPREMOTEAPI_EXPORT_