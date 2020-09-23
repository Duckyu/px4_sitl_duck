/* -*- mode: C++ -*-
 *
 *  Conversions between Latitude/Longitude and UTM
 *              (Universal Transverse Mercator) coordinates.
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

#ifndef _UTM_H
#define _UTM_H

#include "UTM_lib/UTM.h"

/**  @file

     @brief Universal Transverse Mercator transforms.

     Functions to convert (spherical) latitude and longitude to and
     from (Euclidean) UTM coordinates.

     @author Chuck Gantz- chuck.gantz@globalstar.com
 */

#include <cmath>
#include <stdio.h>
#include <stdlib.h>

//#include "conversions.h"

namespace UTM
{
//	// Conversion header
//	const double M_PI = 3.14159265358979323846264338327;
//	const double INCHES_PER_FOOT = 12.0;
//	const double CM_PER_INCH = 2.54;
//	const double CM_PER_METER = 100.0;
//	const double METERS_PER_FOOT = INCHES_PER_FOOT * CM_PER_INCH / CM_PER_METER; // = 0.3048
//	const double MMETERS_PER_KM = 1000000.0;
//	const double MMETERS_PER_MILE = 1609344.0;
//	const double METERS_PER_MILE = MMETERS_PER_MILE / 1000.0;
//	const long   SECONDS_PER_MINUTE = 60;
//	const long   MINUTES_PER_HOUR = 60;
//	const long   SECONDS_PER_HOUR = SECONDS_PER_MINUTE * MINUTES_PER_HOUR;
//	const double RADIANS_PER_DEGREE = M_PI / 180.0;
//	const double DEGREES_PER_RADIAN = 180.0 / M_PI;

//	const double TWOPI = 2.0 * M_PI;
//	const double HALFPI = M_PI / 2.0;

#define MM_PI                3.14159265358979323846264338327
#define INCHES_PER_FOOT     12.0
#define CM_PER_INCH         2.54
#define CM_PER_METER        100.0
#define METERS_PER_FOOT     INCHES_PER_FOOT * CM_PER_INCH / CM_PER_METER // = 0.3048
#define MMETERS_PER_KM      1000000.0
#define MMETERS_PER_MILE    1609344.0
#define METERS_PER_MILE     MMETERS_PER_MILE / 1000.0
#define SECONDS_PER_MINUTE  60
#define MINUTES_PER_HOUR    60
#define SECONDS_PER_HOUR    SECONDS_PER_MINUTE * MINUTES_PER_HOUR
#define RADIANS_PER_DEGREE  MM_PI / 180.0
#define DEGREES_PER_RADIAN  180.0 / MM_PI

//#define TWOPI               2.0 * MM_PI
//#define HALFPI              MM_PI / 2.0


  static inline double mmps2mph(double mm)
  {
    return mm * SECONDS_PER_HOUR / MMETERS_PER_MILE;
  }

  static inline double kmph2mmps(double kmph)
  {
    return kmph * MMETERS_PER_KM / SECONDS_PER_HOUR;
  }

  static inline double mph2mmps(double mph)
  {
    return mph * MMETERS_PER_MILE / SECONDS_PER_HOUR;
  }

  static inline double mph2mps(double mph)
  {
    return mph * METERS_PER_MILE / SECONDS_PER_HOUR;
  }

  static inline double mps2mph(double mps)
  {
    return mps * SECONDS_PER_HOUR / METERS_PER_MILE;
  }

  static inline double feet2meters(double feet)
  {
    return feet * METERS_PER_FOOT;
  }

  static inline double meters2feet(double meters)
  {
    return meters / METERS_PER_FOOT;
  }

  //static inline double tv2secs(struct timeval *tv)
  //{
  //	return tv->tv_sec + (tv->tv_usec / 1000000.0);
  //}

  static inline double analog_volts(int data, double maxvolts, int nbits)
  {
    // clamp value to specified bit range
    int limit = (1 << nbits);
    data &= (limit - 1);
    return (maxvolts * data) / limit;
  }


  // Grid granularity for rounding UTM coordinates to generate MapXY.
  const double grid_size = 100000.0;    // 100 km grid

// WGS84 Parameters
#define WGS84_A		6378137.0		// major axis
#define WGS84_B		6356752.31424518	// minor axis
#define WGS84_F		0.0033528107		// ellipsoid flattening
#define WGS84_E		0.0818191908		// first eccentricity
#define WGS84_EP	0.0820944379		// second eccentricity

// UTM Parameters
#define UTM_K0		0.9996			// scale factor
#define UTM_FE		500000.0		// false easting
#define UTM_FN_N	0.0           // false northing, northern hemisphere
#define UTM_FN_S	10000000.0    // false northing, southern hemisphere
#define UTM_E2		(WGS84_E*WGS84_E)	// e^2
#define UTM_E4		(UTM_E2*UTM_E2)		// e^4
#define UTM_E6		(UTM_E4*UTM_E2)		// e^6
#define UTM_EP2		(UTM_E2/(1-UTM_E2))	// e'^2

static inline void UTM(double lat, double lon, double *x, double *y)
{
  int zone = LatLonToUTMXY(lat,lon,0,*x,*y);
//  printf("zone2 : %d \n", zone);
}


static inline void UTM2GPS(double x, double y, int zone, bool southhemi, double* lat, double* lon)
{
  UTMXYToLatLon(x, y, zone, southhemi, *lat, *lon);
}


static inline int UTM_ZONE(double lon)
{
  int zone = FLOOR((lon + 180.0) / 6) + 1;
  return zone;
}

static inline void UTM_INV(double x, double y,int zone, double *lat, double *lon)
{
  UTMXYToLatLon(x,y,zone,false,*lat,*lon);
}


static inline void LLtoUTM(const double Lat, const double Long,
                           double &UTMNorthing, double &UTMEasting,
                           char* UTMZone)
{

}

static inline void UTMtoLL(const double UTMNorthing, const double UTMEasting,
                           const char* UTMZone, double& Lat,  double& Long )
{

}
} // end namespace UTM

#endif // _UTM_H
