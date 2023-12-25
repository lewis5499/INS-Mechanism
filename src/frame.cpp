#include "frame.h"

namespace Frame {

	Vector3d iewe() {
		return { 0, 0, WGS84_WIE };
	}

	Vector3d iewn(double lat) {
		return { WGS84_WIE * cos(lat), 0, -WGS84_WIE * sin(lat) };
	}

	Vector3d iewn(const Vector3d &origin, const Vector3d &local) {
		Vector3d global = ned2blh(origin, local);
		return iewn(global[0]);
	}

	Vector3d enwn(const Vector2d &rmrn, const Vector3d &blh, const Vector3d &vel) {
		return { vel[1] / (rmrn[1] + blh[2]), -vel[0] / (rmrn[0] + blh[2]), -vel[1] * tan(blh[0]) / (rmrn[1] + blh[2]) };
	}

	Vector3d enwn(const Vector3d& origin, const Vector3d &local, const Vector3d &vel) {
		Vector3d global = ned2blh(origin, local);
		Vector2d rmrn = RmRn(global[0]);
		return enwn(rmrn, global, vel);
	}

	double NormalGravity(const Vector3d &blh) {
		double sin2 = SQR(sin(blh[0]));
		return 9.7803267715 * (1 + 0.0052790414 * sin2 + 0.0000232718 * SQR(sin2)) +
			blh[2] * (0.0000000043977311 * sin2 - 0.0000030876910891) + 0.0000000000007211 * SQR(blh[2]);
	}

	Vector2d RmRn(double lat) {
		double tmp, sqrttmp;
		tmp = 1 - WGS84_E1 * SQR(sin(lat));
		sqrttmp = SQRT(tmp);
		return { WGS84_RA * (1 - WGS84_E1) / (sqrttmp * tmp), WGS84_RA / sqrttmp };
	}

	double Rn(double lat) {
		return WGS84_RA / sqrt(1.0 - WGS84_E1 * SQR(sin(lat)));
	}

	Matrix3d Cne(const Vector3d &blh) {
		Matrix3d dcm;
		double cosl, sinl, cosb, sinb;
		sinb = sin(blh[0]);	sinl = sin(blh[1]);
		cosb = cos(blh[0]);	cosl = cos(blh[1]);
		dcm(0, 0) = -sinb * cosl;	dcm(0, 1) = -sinl;		dcm(0, 2) = -cosb * cosl;
		dcm(1, 0) = -sinb * sinl;	dcm(1, 1) = cosl;		dcm(1, 2) = -cosb * sinl;
		dcm(2, 0) = cosb;			dcm(2, 1) = 0;			dcm(2, 2) = -sinb;
		return dcm;
	}

	Quaterniond Qne(const Vector3d &blh) {
		Quaterniond quat;
		double cosl, sinl, cosb, sinb;
		cosl = cos(blh[1] * 0.5);				sinl = sin(blh[1] * 0.5);
		cosb = cos(-PI * 0.25 - blh[0] * 0.5);	sinb = sin(-PI * 0.25 - blh[0] * 0.5);
		quat.w() = cosb * cosl;
		quat.x() = -sinb * sinl;
		quat.y() = sinb * cosl;
		quat.z() = cosb * sinl;
		return quat;
	}

	Vector3d blh(const Quaterniond &qne, double height) {
		return { -2 * atan(qne.y() / qne.w()) - PI * 0.5, 2 * atan2(qne.z(), qne.w()), height };
	}

	Vector3d blh2ecef(const Vector3d &blh) {
		double cosb, sinb, cosl, sinl, rnh, rn;
		cosb = cos(blh[0]);		sinb = sin(blh[0]);
		cosl = cos(blh[1]);		sinl = sin(blh[1]);
		rn = Rn(blh[0]);		rnh = rn + blh[2];
		return { rnh * cosb * cosl, rnh * cosb * sinl, (rnh - rn * WGS84_E1) * sinb };
	}

	Vector3d ecef2blh(const Vector3d &ecef) {
		double p, rn, lat, lon, h = 0, h0;
		p = sqrt(ecef[0] * ecef[0] + ecef[1] * ecef[1]);
		lat = atan(ecef[2] / (p * (1.0 - WGS84_E1)));
		lon = 2.0 * atan2(ecef[1], ecef[0] + p);
		do {
			h0 = h;
			rn = Rn(lat);
			h = p / cos(lat) - rn;
			lat = atan(ecef[2] / (p * (1.0 - WGS84_E1 * rn / (rn + h))));
		} while (fabs(h - h0) > 1.0e-4);
		return { lat, lon, h };
	}

	Matrix3d dRne(const Vector3d &blh) {
		Matrix3d mat = Matrix3d::Zero();
		Vector2d rmn = RmRn(blh[0]);
		mat(0, 0) = 1.0 / (rmn[0] + blh[2]);
		mat(1, 1) = 1.0 / ((rmn[1] + blh[2]) * cos(blh[0]));
		mat(2, 2) = -1;
		return mat;
	}

	Matrix3d dRen(const Vector3d &blh) {
		Matrix3d mat = Matrix3d::Zero();
		Vector2d rmn = RmRn(blh[0]);
		mat(0, 0) = rmn[0] + blh[2];
		mat(1, 1) = (rmn[1] + blh[2]) * cos(blh[0]);
		mat(2, 2) = -1;
		return mat;
	}

	Vector3d ned2blh(const Vector3d &origin, const Vector3d &local) {
		Vector3d ecef0 = blh2ecef(origin);
		Matrix3d cn0e = Cne(origin);
		Vector3d ecef1 = ecef0 + cn0e * local;
		Vector3d blh1 = ecef2blh(ecef1);
		return blh1;
	}

	Vector3d blh2ned(const Vector3d &origin, const Vector3d &global) {
		Vector3d ecef0 = blh2ecef(origin);
		Matrix3d cn0e = Cne(origin);
		Vector3d ecef1 = blh2ecef(global);
		return cn0e.transpose() * (ecef1 - ecef0);
	}

}

