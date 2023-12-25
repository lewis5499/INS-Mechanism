#ifndef TYPES_H
#define TYPES_H

#include <Eigen/Geometry>

using Eigen::Matrix3d;
using Eigen::Quaterniond;
using Eigen::Vector3d;

typedef struct IMU {
    double time;
    double dt;

    Vector3d dtheta;
    Vector3d dvel;

    double odovel;

	IMU() {
		time = odovel = 0.0;
		dt = 0.05;
		dtheta.setZero();
		dvel.setZero();
	}
} IMU;

typedef struct Attitude {
	Quaterniond qbn;
	Matrix3d cbn;
	Vector3d euler;

	Attitude() {
		qbn.setIdentity();
		cbn.setZero();
		euler.setZero();
	}
} Attitude;

typedef struct NavState {
	double time;
	Vector3d pos;
	Vector3d vel;
	Attitude att;

	NavState() {
		time = 0.0;
		pos.setZero();
		vel.setZero();
	}
} NavState;
#endif // TYPES_H
