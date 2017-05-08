/*
 * extIncludes.h
 *
 *  Created on: Jun 22, 2016
 *      Author: siegbahn
 */

#ifndef SRC_EXTINCLUDES_H_
#define SRC_EXTINCLUDES_H_

#include <boost/geometry/geometries/geometries.hpp>
#include <boost/algorithm/clamp.hpp>
#include <boost/variant.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_smallint.hpp>
#include <boost/random/uniform_01.hpp>
#include <boost/random/uniform_real_distribution.hpp>
#include <boost/random/bernoulli_distribution.hpp>
#include <boost/random/normal_distribution.hpp>
#include <boost/random/lognormal_distribution.hpp>
#include <boost/random/exponential_distribution.hpp>
//#include <boost/geometry.hpp>
//#include <boost/geometry/geometries/linestring.hpp>
//#include <boost/geometry/geometries/point_xy.hpp>

#include <limits>
#include <cmath>
#include <utility>
#include <vector>
#include <set>
#include <string>

#include <Eigen/Eigen>

#include "ofColor.h"
//#include "ofPoint.h"
#include "ofxGeo.h"
#include "ofMain.h"

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
