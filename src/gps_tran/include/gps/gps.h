/***********************************************************************************************
 * Copyright (c) 2018, Youlian Long, Robotic Lab, Sun Yat-sen University, Guangzhou, China.
 * You can contact the author with <longylian@foxmail.com>
 **********************************************************************************************/

#ifndef _GPS_H_
#define _GPS_H_

#include <ros/ros.h>
#include <cmath>
#include <iostream>

#define WGS84 1984 //Reference ellipsoid for World Geodetic System 1984
#ifdef WGS84
    #define SEMI_MAJOR_AXIS                 6378137.0               //SemimajorAxis: a (Unit is m)
    #define SEMI_MINOR_AXIS                 6356752.31424518        //SemiminorAxis: b (Unit is m)
    #define FLATTENING                      0.0034                  //Flattening: f=(a-b)/a (扁率)
    #define ONE_MINUS_FLATTENING            0.9966                  //(1-f)
    #define INVERSE_FLATTENING              298.257223563           //Inverse of Flattening: 1/f (扁率的倒数)
    #define ECCENTRICITY                    0.0818191908426215  //First Eccentricity: e=sqrt(f(2-f)) (第一偏心率)
    #define ECCENTRICITY_SQUA               0.0066943799901410  //Square of First Eccentricity: e^2=f(2-f) (第一偏心率的平方)
    #define ECCENTRICITY_P_SQUA             0.0067394967422760  //Square of Second Eccentricity: e_p^2=e^2/(1-e^2) (第二偏心率的平方)
    #define A_MULTIPLY_BY_E_SQUA    42697.672707179970  //a*e^2
    #define B_MULTIPLY_BY_E_P_SQUA  42841.311513313570  //b*e_p^2
#endif

/*****************************************************************
* struct type
*****************************************************************/
typedef struct
{
    double longitude;               // unit: deg
    double latitude;                // unit: deg
    double altitude;                // unit: m
}GpsDataType;



typedef struct
{
    double x_north;                 // unit: m
    double y_east;                  // unit: m
    double z_down;                  // unit: m
}NedDataType;



/*****************************************************************
* class type
*****************************************************************/
class GpsTran
{
public:
    GpsTran(double initial_longitude, double initial_latitude, double initial_altitude);
    GpsTran(const GpsDataType& initial_gps);
    bool resetInitialGps(double initial_longitude, double initial_latitude, double initial_altitude);
    bool resetInitialGps(const GpsDataType& initial_gps);
    ~GpsTran();

    bool fromGpsToNed(NedDataType& ned, const GpsDataType& gps);
    void fromNedToGps(GpsDataType& gps, const NedDataType& ned);
    
private:
    typedef struct
    {
        double x;                       // unit: m
        double y;                       // unit: m
        double z;                       // unit: m
    }EcefDataType;

    GpsDataType     initial_gps_;
    EcefDataType    initial_ecef_;

    double sin_initial_longitude_;
    double cos_initial_longitude_;
    double sin_initial_latitude_;
    double cos_initial_latitude_;

    bool valid_initial_gps_;

    bool fromGpsToEcef(EcefDataType& ecef, const GpsDataType&  gps);
    void fromEcefToNed(NedDataType&  ned,  const EcefDataType& ecef);
    void fromNedToEcef(EcefDataType& ecef, const NedDataType&  ned);
    void fromEcefToGps(GpsDataType&  gps,  const EcefDataType& ecef);
};


#endif 

