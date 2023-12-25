#ifndef ROTATION_H
#define ROTATION_H

#include "inscmn.h"

namespace Algebra {

	/* DCM <-> Quaternion */
	Quaterniond dcm2quat(const Matrix3d &dcm);
	Matrix3d quat2dcm(const Quaterniond &quaternion);

	/* Quaternion <-> Euler Angle */
	Vector3d quat2euler(const Quaterniond &quaternion);
	Quaterniond euler2quat(const Vector3d &euler);

	/* Euler Angle <-> DCM */
	Matrix3d euler2dcm(const Vector3d &euler);
	Vector3d dcm2euler(const Matrix3d &dcm);

	/* Vector <-> Quaternion */
	Quaterniond vector2quat(const Vector3d &rotvec);
	Vector3d quat2vector(const Quaterniond &quaternion);

	/* Vector <-> skew symmetric */
	Matrix3d vector2skewsym(const Vector3d &vector);
	Vector3d skewsym2vector(const Matrix3d &mat);

}

#endif // ROTATION_H
