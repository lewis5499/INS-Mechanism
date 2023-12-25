#include "algebra.h"

namespace Algebra {

	Quaterniond dcm2quat(const Matrix3d &dcm) {
		return Quaterniond(dcm);
	}

	Matrix3d quat2dcm(const Quaterniond &quaternion) {
		return quaternion.toRotationMatrix();
	}

	Vector3d quat2euler(const Quaterniond &quaternion) {
		return dcm2euler(quaternion.toRotationMatrix());
	}

	Quaterniond euler2quat(const Vector3d &euler) {
		return Quaterniond(AngleAxisd(euler[2], Vector3d::UnitZ()) *
			AngleAxisd(euler[1], Vector3d::UnitY()) *
			AngleAxisd(euler[0], Vector3d::UnitX()));
	}

	Matrix3d euler2dcm(const Vector3d &euler) {	// Roll-Pitch-Yaw --> C_b^n, ZYX rotation sequence
		return Matrix3d(AngleAxisd(euler[2], Vector3d::UnitZ()) *
			AngleAxisd(euler[1], Vector3d::UnitY()) *
			AngleAxisd(euler[0], Vector3d::UnitX()));
	}

	Vector3d dcm2euler(const Matrix3d &dcm) {	// ZYX rotation sequence, IMU located at the						
		Vector3d euler;							// front-right-down position, outputs Roll-Pitch-Yaw
		euler[1] = atan(-dcm(2, 0) / sqrt(dcm(2, 1) * dcm(2, 1) + dcm(2, 2) * dcm(2, 2)));
		if (dcm(2, 0) <= -0.999) {
			euler[0] = 0;
			euler[2] = atan2((dcm(1, 2) - dcm(0, 1)), (dcm(0, 2) + dcm(1, 1)));
			printf("[WARNING] Rotation::dcm2euler: Singular Euler Angle! Set the roll angle to 0!\n");
		}
		else if (dcm(2, 0) >= 0.999) {
			euler[0] = 0;
			euler[2] = PI + atan2((dcm(1, 2) + dcm(0, 1)), (dcm(0, 2) - dcm(1, 1)));
			printf("[WARNING] Rotation::dcm2euler: Singular Euler Angle! Set the roll angle to 0!\n");
		}
		else {
			euler[0] = atan2(dcm(2, 1), dcm(2, 2));
			euler[2] = atan2(dcm(1, 0), dcm(0, 0));
		}
		// yaw: 0~2PI
		if (euler[2] < 0) {
			euler[2] = PI * 2 + euler[2];
		}
		return euler;
	}

	Quaterniond vector2quat(const Vector3d &rotvec) {
		double angle = rotvec.norm();
		Vector3d vec = rotvec.normalized();
		return Quaterniond(AngleAxisd(angle, vec));
	}

	Vector3d quat2vector(const Quaterniond &quaternion) {
		AngleAxisd axisd(quaternion);
		return axisd.angle() * axisd.axis();
	}

	Matrix3d vector2skewsym(const Vector3d &vector) {
		Matrix3d mat;
		mat << 0, -vector(2), vector(1), vector(2), 0, -vector(0), -vector(1), vector(0), 0;
		return mat;
	}

	Vector3d skewsym2vector(const Matrix3d & mat) {
		Vector3d vec;
		vec << mat(2, 1), mat(0, 2), mat(1, 0);
		return vec;
	}

}
