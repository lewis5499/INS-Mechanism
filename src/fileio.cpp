#include "fileio.h"

int bfileRopen(const char *filer, FILE **fp, IMU& firstIMU, const double t0)
{
	if((*fp = fopen(filer, "rb")) == NULL) { // open 'binary' file
		printf("Cannot open raw binary file. \n");
		return -1;
	}
	if (epochSYNC(fp, firstIMU, t0) < 0) { return -1; }
	return 0;
}

int afileWopen(const char *filew, FILE **fp)
{
	if ((*fp = fopen(filew, "w+")) == NULL) { // save as 'ANSI' txt file
		printf("Cannot create target file: %s. \n", filew);
		return -1;
	}
	return 0;
}

int epochSYNC(FILE **fp, IMU& firstIMU, const double t0)
{
	while (!feof(*fp)) {
		loadIMUepoch(fp, firstIMU);
		if (fabs(firstIMU.time - t0) < THRESHOLD) { break; }
	}
	if (feof(*fp)) { return -1; }
	return 0;
}

int loadIMUepoch(FILE **fp, IMU& curIMU)
{
	uint8_t buff[EPOCHBUFFLEN];
	uint8_t *p = buff;

	if (!fread(buff, 8, 7, *fp)) { return -1; }
	if (feof(*fp)) { return -1; }

	curIMU.time = R8(p);		p += 8;
	//thisEpoch.dt = 0.005;			
	curIMU.dtheta(0) = R8(p);	p += 8;
	curIMU.dtheta(1) = R8(p);	p += 8;
	curIMU.dtheta(2) = R8(p);   p += 8;
	curIMU.dvel(0)   = R8(p);	p += 8;
	curIMU.dvel(1)   = R8(p);	p += 8;
	curIMU.dvel(2)   = R8(p);	
	return 0;
}

void outfNavState(FILE *fp, const NavState& curState)
{
	fprintf(fp, "%18.10f %15.10f %15.10f %10.4f %12.8f %12.8f %12.8f %15.10f %15.10f %15.10f\n",
		curState.time, R2D*(curState.pos(0)), R2D*(curState.pos(1)), curState.pos(2),
		curState.vel(0), curState.vel(1), curState.vel(2), R2D*(curState.att.euler(0)),	R2D*(curState.att.euler(1)),
		curState.att.euler(2)>PI ? R2D*(curState.att.euler(2)-2*PI): R2D*(curState.att.euler(2)));
	//printf("%18.10f %15.10f %15.10f %10.4f %12.8f %12.8f %12.8f %15.10f %15.10f %15.10f\n",
	//	curState.time, R2D*(curState.pos(0)), R2D*(curState.pos(1)), curState.pos(2),
	//	curState.vel(0), curState.vel(1), curState.vel(2), R2D*(curState.att.euler(0)),	R2D*(curState.att.euler(1)),
	//	curState.att.euler(2)>PI ? R2D*(curState.att.euler(2)-2*PI): R2D*(curState.att.euler(2)));
}

void decodepureINSresb2txt()
{
	FILE *fpr = NULL, *fpw = NULL;
	double buff[10];
	const char *bfname = "..\\dataset\\01 demo\\PureINS.bin"; 
	const char *ofname = "..\\outfiles\\01 demo\\INSresults_ref.txt";
	if (!(fpr = fopen(bfname, "rb"))) { return; }
	if (!(fpw = fopen(ofname, "w"))) { return; }
	printf("%s\n", "Decode Example Data reference results...");
	while (!feof(fpr)) {
		if (!fread(buff, 8, 10, fpr)) { return; }
		if (feof(fpr)) { return; }
		fprintf(fpw, "%18.10f %15.10f %15.10f %10.4f %12.8f %12.8f %12.8f %15.10f %15.10f %15.10f\n",
			buff[0], buff[1], buff[2], buff[3], buff[4], buff[5], buff[6], buff[7], buff[8], buff[9]);
	}
	fclose(fpr); fclose(fpw);
}
