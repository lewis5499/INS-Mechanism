#ifndef COMMON_H
#define COMMON_H

#include <Windows.h>
#include <stdint.h>
#include <windef.h>
#include <Eigen/Geometry>
#include <Eigen/Core>
#include "types.h"

using Eigen::Matrix3d;
using Eigen::Quaterniond;
using Eigen::Vector3d;
using Eigen::Vector2d;
using Eigen::AngleAxisd;

/* WGS84 params --------------------------------------------------------------*/
constexpr auto WGS84_WIE = 7.2921151467E-5;		
constexpr auto WGS84_F = 0.0033528106647474805;		/* oblateness */
constexpr auto WGS84_RA = 6378137.0000000000;		
constexpr auto WGS84_RB = 6356752.3142451793;	
constexpr auto WGS84_GM = 398600441800000.00;
constexpr auto WGS84_E1 = 0.0066943799901413156;	/* note: SQUARE */
constexpr auto WGS84_E2 = 0.0067394967422764341;	/* note: SQUARE */

/* mathmatic functions -------------------------------------------------------*/
#define SQR(x)		((x)*(x))
#define SQRT(x)     ((x)<=0.0||(x)!=(x)?0.0:sqrt(x))
#define MAX(x,y)    ((x)>(y)?(x):(y))
#define MIN(x,y)    ((x)<(y)?(x):(y))
#define ROUND(x)    (int)floor((x)+0.5)

/* constants -----------------------------------------------------------------*/
#define PI				3.14159265358979323846
#define R2D				180.0/PI
#define D2R				PI/180.0
#define EPOCHBUFFLEN	80
#define THRESHOLD		1.0E-6
#define THRH_LATLON		1.0E-7*D2R  //rad
#define THRH_HEIGHT		2.0
#define THRH_VEL		1.0E-5		//1.0e-4
#define THRH_ATT		1.0E-7*D2R	//rad:roll pitch yaw

/* init Nav info -------------------------------------------------------------*/
// demo refs
#define SAMPLING_RATE0	200
#define STARTINGTIME0	91620.0 
#define LAT0			23.1373950708*D2R
#define LON0			113.3713651222*D2R
#define HEIGHT0			2.175
#define VEL0_N			0.0
#define VEL0_E			0.0
#define VEL0_D			0.0
#define ROLL0			0.0107951084511778*D2R
#define PITCH0			-2.14251290749072*D2R
#define	YAW0			-75.7498049314083*D2R	//during calculation we make the 'yaw' range between 0~360deg
// refs
#define SAMPLING_RATE	200
#define STARTINGTIME	274119.9986884708
#define LAT				30.5278036113*D2R
#define LON				114.3557909526*D2R
#define HEIGHT			20.994
#define VEL_N			0.0
#define VEL_E			0.0
#define VEL_D			0.0
#define ROLL			0.34771930*D2R			//0.344658429202574*RAD
#define PITCH			0.67560824*D2R			//0.670957738195724*RAD		/* ->rough alignment */
#define	YAW				357.96993977*D2R		//-2.928974229324932*RAD
#define STILLSPAN1START	274119.9986884708
#define STILLSPAN1END	274419.5037986656
#define STILLSPAN2START	274670.0038890597
#define STILLSPAN2END	274737.2089136200
#define STILLSPAN3START	274989.0040040533
#define STILLSPAN3END	275054.8040281707 
#define STILLSPAN4START	275310.5041182469
#define STILLSPAN4END	275370.7041401501
#define STILLSPAN5START	275618.6042278114
#define STILLSPAN5END	275931.7743384507

/* Determine whether the time is within the time interval [ts, te] -----------*/
static inline bool isIn(const double &t, const double &ts, const double &te) {
	return (t >= ts - THRESHOLD) && (t <= te + THRESHOLD);
}

/* Get fields (little-endian) ------------------------------------------------*/
#define U1(p) (*((uint8_t *)(p)))
#define I1(p) (*((int8_t  *)(p)))
static uint16_t U2(uint8_t *p) { uint16_t u; memcpy(&u, p, 2); return u; }
static uint32_t U4(uint8_t *p) { uint32_t u; memcpy(&u, p, 4); return u; }
static int32_t  I4(uint8_t *p) { int32_t  i; memcpy(&i, p, 4); return i; }
static float    R4(uint8_t *p) { float    r; memcpy(&r, p, 4); return r; }
static double   R8(uint8_t *p) { double   r; memcpy(&r, p, 8); return r; }

/* IMU Epoch data deep copy --------------------------------------------------*/
static inline void rawIMUdpcy(IMU& dst, const IMU& ori) {
	dst.time = ori.time;
	dst.dt = ori.dt;
	dst.odovel = ori.odovel;
	dst.dtheta = ori.dtheta;
	dst.dvel = ori.dvel;
}
static inline void navStatedpcy(NavState& dst, const NavState& ori) {
	dst.time = ori.time;
	dst.pos = ori.pos;
	dst.vel = ori.vel;
	dst.att.cbn = ori.att.cbn;
	dst.att.qbn = ori.att.qbn;
	dst.att.euler = ori.att.euler;
}
#endif // !CONSTS_H
