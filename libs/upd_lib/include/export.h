/*  +---------------------------------------------------------------------------+
 *  |                                                                           |
 *  |               https://sites.google.com/site/bellonemauro/                 |
 *  |                                                                           |
 *  | Copyright (c) 2015, - All rights reserved.                                |
 *  | Authors: Mauro Bellone                                                    |
 *  | Released under BDS License.                                               |
 *  +---------------------------------------------------------------------------+ */

#ifndef __UPD_EXPORT_
#define __UPD_EXPORT_

#if defined(_MSC_VER) || defined(__CYGWIN__) || defined(__MINGW32__) || defined( __BCPLUSPLUS__)  || defined( __MWERKS__)
#  ifdef UPD_LIBRARY
#    define UPD_EXPORT __declspec(dllexport)
#  else
#    define UPD_EXPORT __declspec(dllimport)
#  endif
#else
#  define UPD_EXPORT
#endif

#endif // __UPD_EXPORT_