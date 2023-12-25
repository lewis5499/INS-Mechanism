#ifndef FIO_H
#define FIO_H
#pragma warning (disable:4996)

#include<stdio.h>
#include"inscmn.h"
#include"types.h"

/* operations for files  ------------------------------------------------------*/
int bfileRopen(const char *filer, FILE **fp, IMU& firstIMU, const double t0);
int afileWopen(const char *filew, FILE **fp);
void outfNavState(FILE *fp, const NavState& curState);

/* find the first IMUepoch for calculation according to 't0' ------------------*/
int epochSYNC(FILE **fp, IMU& firstIMU, const double t0);

/* decode IMUepoch, begin from *fp->?  ----------------------------------------*/
int loadIMUepoch(FILE **fp, IMU& curIMU);

/* decode and output to 'txt': reference results for demo INS -----------------*/
void decodepureINSresb2txt();


#endif // !FIO_H



