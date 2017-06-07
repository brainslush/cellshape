/*
 * extIncludes.h
 *
 *  Created on: Jun 22, 2016
 *      Author: siegbahn
 */

#ifndef SRC_EXTINCLUDES_H_
#define SRC_EXTINCLUDES_H_

#include <boost/geometry/geometries/geometries.hpp>

#include <boost/variant.hpp>

//#include <boost/geometry.hpp>
//#include <boost/geometry/geometries/linestring.hpp>
//#include <boost/geometry/geometries/point_xy.hpp>

#include <limits>
#include <cmath>
#include <utility>
#include <vector>
#include <set>
#include <string>
#include <typeinfo>

#include "ofMain.h"

#ifdef Success
#undef Success
#endif

#include <eigen3/Eigen/Eigen>

// system specific includes for getting systemtime
//#include <boost/throw_exception.hpp>
#if defined(BOOST_WINDOWS)
#include <windows.h>
#elif defined(__linux__) || defined(__linux) || defined(linux)

#include <sys/sysinfo.h>

#elif defined(macintosh) || defined(__APPLE__) || defined(__APPLE_CC__)
/* still missing */
#elif defined(__FreeBSD__) || defined(__NetBSD__) || defined(__OpenBSD__) || defined(__DragonFly__)
/* still missing */
#endif

#endif /* SRC_EXTINCLUDES_H_ */
