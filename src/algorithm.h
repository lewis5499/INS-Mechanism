#ifndef ALGO_H
#define ALGO_H

#include"inscmn.h"
#include"fileio.h"
#include"frame.h"
#include"algebra.h"

using namespace Algebra;
using namespace Frame;
using namespace Eigen;

/* demo: pureINS --------------------------------------------------------------------------------*/
int pureINSf_demo();
void stateInit_demo(NavState &State);

/* Post pureINS: based IMU binary file ----------------------------------------------------------*/
int pureINSf();
void stateInit(NavState &State);
void renewIMU_State(NavState &curState, IMU& curIMU, const IMU& preIMU);
bool zeroVelCorr(const NavState &preState, NavState &curState, const IMU &preIMU, const IMU &curIMU);
void insUpdate(const NavState &preState, NavState &curState, const IMU &preIMU, const IMU &curIMU);
void attUpdate(const NavState &preState, NavState &curState, const IMU &preIMU, const IMU &curIMU);
void velUpdate(const NavState &preState, NavState &curState, const IMU &preIMU, const IMU &curIMU);
void posUpdate(const NavState &preState, NavState &curState, const IMU &preIMU, const IMU &curIMU);

#endif // !ALGO_H

