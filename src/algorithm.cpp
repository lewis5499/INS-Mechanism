#include "algorithm.h"

int pureINSf_demo()
{
	//calculate INS reference results
	decodepureINSresb2txt();

	FILE *fpr = NULL, *fpw = NULL;
	IMU		 curIMU, preIMU;
	NavState curState, preState;
	const char *filer = "..\\dataset\\01 demo\\IMU.bin";
	const char *filew = "..\\outfiles\\01 demo\\INSresults_demo.txt";
	//open files
	if (bfileRopen(filer, &fpr, preIMU, STARTINGTIME0) < 0) { return -1; }
	if (afileWopen(filew, &fpw) < 0) { return -2; }
	printf("%s\n", "Calculate Example Data...");
	//init first State with pre-info and output
	stateInit_demo(preState);
	outfNavState(fpw, preState);
	while (!loadIMUepoch(&fpr, curIMU))
	{
		//renew current State: time, and IMU: sampling-dt 
		renewIMU_State(curState, curIMU, preIMU);
		//INS Mechanization
		insUpdate(preState, curState, preIMU, curIMU);
		//output to file
		outfNavState(fpw, curState);
		//renew IMU/State: current->previous
		rawIMUdpcy(preIMU, curIMU);
		navStatedpcy(preState, curState);
	} 
	fclose(fpr); fclose(fpw);
	return 0;
}

void stateInit_demo(NavState &State)
{
	State.time = STARTINGTIME0;
	State.pos(0) = LAT0;
	State.pos(1) = LON0;
	State.pos(2) = HEIGHT0;
	State.vel(0) = VEL0_N;
	State.vel(1) = VEL0_E;
	State.vel(2) = VEL0_D;
	State.att.euler(0) = ROLL0;
	State.att.euler(1) = PITCH0;
	State.att.euler(2) = (YAW0 < 0) ? YAW0 + 2 * PI : YAW0;
	State.att.cbn = euler2dcm(State.att.euler);
	State.att.qbn = euler2quat(State.att.euler);
}

int pureINSf()
{
	FILE *fpr = NULL, *fpw = NULL;
	IMU		 curIMU, preIMU;
	NavState curState, preState;
	const char *filer = "..\\dataset\\02 A15\\11_31_52_CHA3_IMU.bin";
	const char *filew = "..\\outfiles\\02 A15\\pureINSresults.txt";

	//open files
	if (bfileRopen(filer, &fpr, preIMU, STILLSPAN1END) < 0) { return -1; }
	if (afileWopen(filew, &fpw) < 0) { return -2; }
	printf("%s\n\n", "Calculate IMU Data...");

	//init first State with pre-info and output
	stateInit(preState);
	outfNavState(fpw, preState);
	while (!loadIMUepoch(&fpr, curIMU))
	{
		//renew current State: time, and IMU: sampling-dt 
		renewIMU_State(curState, curIMU, preIMU);	
		//INS Mechanization: att, vel, pos Update
		insUpdate(preState, curState, preIMU, curIMU);
		//output to file
		outfNavState(fpw, curState);
		//renew IMU/State: current->previous
		rawIMUdpcy(preIMU, curIMU);
		navStatedpcy(preState, curState);
	}
	fclose(fpr); fclose(fpw);
	printf("Done!\n");
	return 0;
}

void stateInit(NavState & State)
{
	State.time = STILLSPAN1END;
	State.pos(0) = LAT;
	State.pos(1) = LON;
	State.pos(2) = HEIGHT;
	State.vel(0) = VEL_N;
	State.vel(1) = VEL_E;
	State.vel(2) = VEL_D;
	State.att.euler(0) = ROLL;
	State.att.euler(1) = PITCH;
	State.att.euler(2) = (YAW < 0) ? YAW + 2 * PI : YAW;
	State.att.cbn = euler2dcm(State.att.euler);
	State.att.qbn = euler2quat(State.att.euler);
}

void renewIMU_State(NavState &curState, IMU &curIMU, const IMU &preIMU)
{
	double dt = curIMU.time - preIMU.time;
	if (dt>0.1) {
		dt = 1.0 / (double)SAMPLING_RATE;
	}else{
		curIMU.dt = dt;
	}
	curState.time = curIMU.time;
}

void insUpdate(const NavState &preState, NavState &curState, const IMU &preIMU, const IMU &curIMU) 
{
	// NOTE: irreversible order!
	attUpdate(preState, curState, preIMU, curIMU);
	velUpdate(preState, curState, preIMU, curIMU);
	posUpdate(preState, curState, preIMU, curIMU);
}

void attUpdate(const NavState &preState, NavState &curState, const IMU &preIMU, const IMU &curIMU) 
{
	Vector2d rmrn;
	Vector3d wie_n, wen_n;
	Quaterniond qnn, qbb;

	/* compute rmrn, wie_n, wen_n: k-1 */
	rmrn = RmRn(preState.pos[0]);
	wie_n = iewn(preState.pos[0]);
	wen_n = enwn(rmrn, preState.pos, preState.vel);

	/* n-frame rotation vector (n(k-1)->n(k)) */
	qnn = vector2quat(-(wie_n + wen_n) * curIMU.dt);
	/* b-frame rotation vector (b(k)->b(k-1)): compensate the second-order coning correction term */
	qbb = vector2quat(curIMU.dtheta + preIMU.dtheta.cross(curIMU.dtheta) / 12.0);

	/* attitude update finish! */
	curState.att.qbn = (qnn * preState.att.qbn * qbb).normalized();
	curState.att.cbn = quat2dcm(curState.att.qbn);
	curState.att.euler = dcm2euler(curState.att.cbn);
}

void velUpdate(const NavState &preState, NavState &curState, const IMU &preIMU, const IMU &curIMU)
{
	if (zeroVelCorr(preState, curState, preIMU, curIMU)) { return; }

	Vector2d rmrn;
	Vector3d d_vfb, d_vfn, d_vgn, gl, midvel, midpos, wie_n, wen_n, zetaVec3;
	Matrix3d Cnn, I33 = Matrix3d::Identity();

	/* estimate k-1 -> k-1/2: vel, pos */
	// calculate vec3d: rmrn, wie(projected->n-frame), wen(projected->n-frame), gl(normal gravity)
	rmrn = RmRn(preState.pos[0]);
	wie_n = iewn(preState.pos[0]);
	wen_n = enwn(rmrn, preState.pos, preState.vel);

	// d_vgn: gravity and Coriolis force
	gl << 0, 0, NormalGravity(preState.pos);
	d_vgn = (gl - (2 * wie_n + wen_n).cross(preState.vel)) * curIMU.dt;

	// d_vfn: the specific force (compensate rotational and sculling motion)
	d_vfb = curIMU.dvel + curIMU.dtheta.cross(curIMU.dvel) / 2.0 + 
		(preIMU.dtheta.cross(curIMU.dvel) + preIMU.dvel.cross(curIMU.dtheta)) / 12.0;
	zetaVec3 = (wie_n + wen_n) * curIMU.dt / 2;
	Cnn = I33 - vector2skewsym(zetaVec3);	// project d_vfb->n-frame
	d_vfn = Cnn * preState.att.cbn * d_vfb;				

	// velocity at k-1/2: estimated!
	midvel = preState.vel + (d_vfn + d_vgn) / 2.0;

	// position at k-1/2, extrapolation!
	midpos(2) = preState.pos[2] - midvel[2] * curIMU.dt / 2.0;
	midpos(0) = preState.pos[0] + midvel[0] / (rmrn[0] + midpos[2])*curIMU.dt / 2.0;
	midpos(1) = preState.pos[1] + midvel[1] / ((rmrn[1] + midpos[2])*cos(midpos[0]))*curIMU.dt / 2.0;

	/* recompute vel Update -> k */
	// vel, pos at k-1/2 exist! -> rmrn, wie_n, wen_n at k-1/2
	rmrn = RmRn(midpos[0]);
	wie_n = iewn(midpos[0]);
	wen_n = enwn(rmrn, midpos, midvel);

	// d_vfn: the specific force
	Cnn = I33 - vector2skewsym((wie_n + wen_n) * curIMU.dt / 2.0);
	d_vfn = Cnn * preState.att.cbn * d_vfb;

	// d_vgn: gravity and Coriolis force
	gl << 0, 0, NormalGravity(midpos);
	d_vgn = (gl - (2 * wie_n + wen_n).cross(midvel)) * curIMU.dt;

	// velocity update finish!
	curState.vel = preState.vel + d_vfn + d_vgn;
}

void posUpdate(const NavState &preState, NavState &curState, const IMU &preIMU, const IMU &curIMU)
{
	Vector3d midvel, midpos;

	/* Vel has been updated: vel at k exist! */
	// compute velocity and position at k-1/2 
	midvel = (curState.vel + preState.vel) / 2.0;
	midpos = preState.pos + dRne(preState.pos) * midvel * curIMU.dt / 2.0;

	//position update finish!
	curState.pos = preState.pos + dRne(midpos) * midvel * curIMU.dt;
}

bool zeroVelCorr(const NavState &preState, NavState &curState, const IMU &preIMU, const IMU &curIMU)
{
	if (isIn(curState.time, STILLSPAN1START, STILLSPAN1END) ||
		isIn(curState.time, STILLSPAN2START, STILLSPAN2END) ||
		isIn(curState.time, STILLSPAN3START, STILLSPAN3END) ||
		isIn(curState.time, STILLSPAN4START, STILLSPAN4END) ||
		isIn(curState.time, STILLSPAN5START, STILLSPAN5END)) {
		curState.vel.setZero();	
		return true;
	}
	return false;
}