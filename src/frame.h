#ifndef EARTH_H
#define EARTH_H

#include "inscmn.h"

namespace Frame {

	/* Vector3: wie->e frame (projection) */
	Vector3d iewe();

	/* Vector3: wie->n frame (projection) */
	Vector3d iewn(double lat);
	Vector3d iewn(const Vector3d &origin, const Vector3d &local);

	/* Vector3: wen->n frame (projection) */
	Vector3d enwn(const Vector2d &rmrn, const Vector3d &blh, const Vector3d &vel);
	Vector3d enwn(const Vector3d &origin, const Vector3d &local, const Vector3d &vel);

	/* double: Normal gravity calculation */
	double NormalGravity(const Vector3d &blh);

	/* Vector2/double: calculating the radius of the meridian circle and the radius of the prime vertical circle */
	Vector2d RmRn(double lat);
	double Rn(double lat);

	/* Matrix3: n(Nav Frame)->e(ECEF) transformation */
	Matrix3d Cne(const Vector3d &blh);

	/* Quaternion: n(Nav Frame)->e(ECEF) transformation */
	Quaterniond Qne(const Vector3d &blh);

	/* Vector3: get lat, lon from Quaternion(n->e Frame) */
	Vector3d blh(const Quaterniond &qne, double height);

	/* Vector3: 'geodetic' coordinates -> 'ECEF' coordinates */
	Vector3d blh2ecef(const Vector3d &blh);

	/* Vector3: 'ECEF' coordinates -> 'geodetic' coordinates */
	Vector3d ecef2blh(const Vector3d &ecef);

	/* Matrix3: n-frame vector3(vel) -> geodetic coordinate vector3(vel) at 'blh' */
	Matrix3d dRne(const Vector3d &blh);

	/* Matrix3: geodetic coordinate vector3(vel) -> n-frame vector3(vel) at 'blh' */
	Matrix3d dRen(const Vector3d &blh);

	/* Vector3: local coordinates (expanded around the origin) -> geodetic coordinates */
	Vector3d ned2blh(const Vector3d &origin, const Vector3d &local);

	/* Vector3: geodetic coordinates -> local coordinates (expanded around the origin) */
	Vector3d blh2ned(const Vector3d &origin, const Vector3d &global);
}
#endif // EARTH_H
