/*
 * Copyright (C) 2014 YuanFeng Technology
 * liqiangman@yftech.com
 */

#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

typedef struct {
	float w;
	float x;
	float y;
	float z;
} QUAT_TYPE, *QUAT_TYPP;

typedef struct {
	float t;
	float x;
	float y;
	float z;
} GYRO_TYPE, ACC_TYPE, *GYRO_TYPP, *ACC_TYPP;

typedef struct {
	float ax;
	float ay;
	float az;
	float gx;
	float gy;
	float gz;
} SENSOR_TYPE, *SENSOR_TYPP;

#define GYRO_S_STABLE   0
#define GYRO_S_UNSTABLE 1
#define ACC_MAX_VALUE   19.6

static float s_acc_x, s_acc_y, s_acc_z;
static float s_acc_modmd, s_gyro_modl;

static int s_gyro_status;
static float s_gyro_start, s_gyro_last;
static float s_gyro_modm, s_gyro_modt, s_gyro_maxw, s_gyro_maxt;
static float s_gyro_calx, s_gyro_caly, s_gyro_calz, s_gyro_calt;
static float s_gyro_quatw = 1, s_gyro_quatx, s_gyro_quaty, s_gyro_quatz;
static float s_gyro_lastw = 1, s_gyro_lastx, s_gyro_lasty, s_gyro_lastz;

static float s_tang_coff[2][4];

#define ACC_BUF_LEN     300
#define SENSOR_F_GYRO   1
#define SENSOR_F_ACC    2
static SENSOR_TYPE s_sensor_buf[ACC_BUF_LEN];
static SENSOR_TYPP s_sensor_bufps, s_sensor_bufpu, s_sensor_bufpm, s_sensor_bufpc = s_sensor_buf;
static int s_sensor_flag;

static void update_sensor(int flag)
{
	if(flag & s_sensor_flag) {
		s_sensor_flag = flag;
		s_sensor_bufpc++;
		if(s_sensor_bufpc == s_sensor_buf+ACC_BUF_LEN) {
			s_sensor_bufpc = s_sensor_buf;
		}
		if(s_sensor_bufpc == s_sensor_bufps) {
			//over written
			s_sensor_bufps = 0;
		}
	}
	else {
		s_sensor_flag |= flag;
	}
}

static void init_quat(float t)
{
	s_gyro_quatw = 1;
	s_gyro_quatx = s_gyro_quaty = s_gyro_quatz = 0;
	s_gyro_modm = s_gyro_maxw = 0;
	s_acc_modmd = 0;
	s_gyro_last = t;
	s_gyro_start = t;
	s_gyro_status = GYRO_S_STABLE;
}

static void update_quat(float t, float x, float y, float z)
{
float W;
	float sinr, cosr, siny, cosy, sinp, cosp;
	float temp, w, tw, tx, ty, tz;
	x -= s_gyro_calx;
	y -= s_gyro_caly;
	z -= s_gyro_calz;
	temp = x*x + y*y + z*z;

	update_sensor(SENSOR_F_GYRO);
	s_sensor_bufpc->gx = x;
	s_sensor_bufpc->gy = y;
	s_sensor_bufpc->gz = z;

	if(s_gyro_start == 0) {
		init_quat(t);
		s_sensor_bufps = s_sensor_bufpc;
		return;
	}

	if(temp > s_gyro_modm) {
		s_gyro_modm = temp;
		s_gyro_modt = t;
		s_sensor_bufpm = s_sensor_bufpc;
	}
	s_gyro_modl = temp;

	temp =  s_gyro_quatx * x + s_gyro_quaty * y + s_gyro_quatz * z;
	if(temp > s_gyro_maxw) {
		s_gyro_maxw = temp;
		s_gyro_maxt = s_gyro_modt;
	}
W = temp;

	temp = (t-s_gyro_last) * 0.5;
	x *= temp;
	y *= temp;
	z *= temp;

	/*
	sinr = sin(x);
	cosr = cos(x);
	sinp = sin(y);
	cosp = cos(y);
	siny = sin(z);
	cosy = cos(z);
	*/

	sinr = x; cosr = 1 - x * x / 2;
	sinp = y; cosp = 1 - y * y / 2;
	siny = z; cosy = 1 - z * z / 2;

	w = cosr * cosp * cosy + sinr * sinp * siny;
	x = sinr * cosp * cosy - cosr * sinp * siny;
	y = cosr * sinp * cosy + sinr * cosp * siny;
	z = cosr * cosp * siny - sinr * sinp * cosy;

	tw = s_gyro_quatw * w - s_gyro_quatx * x - s_gyro_quaty * y - s_gyro_quatz * z;
	tx = s_gyro_quatw * x + s_gyro_quatx * w + s_gyro_quaty * z - s_gyro_quatz * y;
	ty = s_gyro_quatw * y + s_gyro_quaty * w + s_gyro_quatz * x - s_gyro_quatx * z;
	tz = s_gyro_quatw * z + s_gyro_quatz * w + s_gyro_quatx * y - s_gyro_quaty * x;

	printf("==q %.3f %f %f (%f %f %f %f) (%f,%f %f %f)\r\n", t, s_gyro_modl, W, w, x, y, z, tw, tx, ty, tz);

	if(tw < 0.9999) { //~1.6DEG
		if(s_gyro_status == GYRO_S_STABLE) {
			float time = (s_gyro_last - s_gyro_start) * 0.5;
			s_gyro_status = GYRO_S_UNSTABLE;
			temp = time / s_gyro_modm;
			if(temp > s_gyro_calt) {
				s_gyro_calt = temp;
				s_gyro_calx += s_gyro_quatx/time;
				s_gyro_caly += s_gyro_quaty/time;
				s_gyro_calz += s_gyro_quatz/time;
				init_quat(t);
				printf("**c %f %f %f %f\r\n", s_gyro_calt, s_gyro_calx, s_gyro_caly, s_gyro_calz);
				return;
			}
			s_sensor_bufpu = s_sensor_bufpc;
			printf("**s unstable\r\n");
		}
		else if(tw > s_gyro_quatw) {
			printf("**t change direction %f %f %f %f\r\n", s_gyro_modm, s_gyro_maxw, s_gyro_modt, s_gyro_maxt);
			if(s_gyro_quatw < 0.999) { //~5DEG
				temp = s_gyro_lastx * s_gyro_quatx + s_gyro_lasty * s_gyro_quaty + s_gyro_lastz * s_gyro_quatz;
				printf("**t valid direction %f %f\r\n", s_gyro_lastw*s_gyro_quatw-temp, temp);
				if(s_gyro_quatw < 0.985 || s_gyro_lastw*s_gyro_quatw-temp >= s_gyro_quatw) { //~20
					printf("**t valid2 %x\r\n", s_gyro_status);
					s_gyro_lastw = s_gyro_quatw;
					s_gyro_lastx = s_gyro_quatx;
					s_gyro_lasty = s_gyro_quaty;
					s_gyro_lastz = s_gyro_quatz;
					cal_tangents();
				}
				s_sensor_bufps = s_sensor_bufpc;
			}
			init_quat(t);
			return;
		}
	}
	/*
	else if(s_gyro_status == GYRO_S_STABLE && s_gyro_modl < 0.01) {
		s_sensor_bufps = s_sensor_bufpc;
	}
	*/
	s_gyro_quatw = tw;
	s_gyro_quatx = tx;
	s_gyro_quaty = ty;
	s_gyro_quatz = tz;
	s_gyro_last = t;
}

#define GYRO_BUF_LEN 1
static GYRO_TYPE s_gyro_buf[GYRO_BUF_LEN];
static GYRO_TYPP s_gyro_bufp = s_gyro_buf;
static float s_gyro_buft, s_gyro_bufx, s_gyro_bufy, s_gyro_bufz;

void update_gyro(float t, float x, float y, float z)
{
	s_gyro_buft -= s_gyro_bufp->t;
	s_gyro_bufx -= s_gyro_bufp->x;
	s_gyro_bufy -= s_gyro_bufp->y;
	s_gyro_bufz -= s_gyro_bufp->z;
	s_gyro_bufp->t = t;
	s_gyro_bufp->x = x;
	s_gyro_bufp->y = y;
	s_gyro_bufp->z = z;
	s_gyro_buft += t;
	s_gyro_bufx += x;
	s_gyro_bufy += y;
	s_gyro_bufz += z;
	s_gyro_bufp++;
	if(s_gyro_bufp == s_gyro_buf+GYRO_BUF_LEN) {
		s_gyro_bufp = s_gyro_buf;
	}
	update_quat(s_gyro_buft/GYRO_BUF_LEN, s_gyro_bufx/GYRO_BUF_LEN, s_gyro_bufy/GYRO_BUF_LEN, s_gyro_bufz/GYRO_BUF_LEN);
}

static ACC_TYPE s_acc_buf[GYRO_BUF_LEN];
static ACC_TYPP s_acc_bufp = s_acc_buf;
static float s_acc_buft, s_acc_bufx, s_acc_bufy, s_acc_bufz;

static void update_acc(float t, float x, float y, float z)
{
	s_acc_buft -= s_acc_bufp->t;
	s_acc_bufx -= s_acc_bufp->x;
	s_acc_bufy -= s_acc_bufp->y;
	s_acc_bufz -= s_acc_bufp->z;
	s_acc_bufp->t = t;
	s_acc_bufp->x = x;
	s_acc_bufp->y = y;
	s_acc_bufp->z = z;
	s_acc_buft += t;
	s_acc_bufx += x;
	s_acc_bufy += y;
	s_acc_bufz += z;
	s_acc_bufp++;
	if(s_acc_bufp == s_acc_buf+GYRO_BUF_LEN) {
		s_acc_bufp = s_acc_buf;
	}
	update_sensor(SENSOR_F_ACC);
	s_sensor_bufpc->ax = s_acc_bufx/GYRO_BUF_LEN;
	s_sensor_bufpc->ay = s_acc_bufy/GYRO_BUF_LEN;
	s_sensor_bufpc->az = s_acc_bufz/GYRO_BUF_LEN;
}

/*
static void update_acc(float t, float x, float y, float z)
{
	update_sensor(SENSOR_F_ACC);
	s_sensor_bufpc->x = x;
	s_sensor_bufpc->y = y;
	s_sensor_bufpc->z = z;
}
*/

int main(int argc , char * argv[])
{
	FILE * file;
	char buffer[256];
	char * sensor = "sensor.txt";

	if(argc > 1) {
		sensor = argv[1];
	}
	file = fopen(sensor, "r");
	if(file == 0) {
		printf("failed to open file %s\n", sensor);
		return -1;
	}
	s_gyro_calt =  2000;
	s_gyro_calx =  0.023;
	s_gyro_caly = -0.009;
	s_gyro_calz = -0.006;
	printf("sensor file name %s\r\n", sensor);
	while(fgets(buffer, sizeof(buffer), file)) {
		float time, accx, accy, accz, gyrx, gyry, gyrz, magx, magy, magz;
		if(!memcmp("%%%", buffer, 3)) {
			if(!memcmp("exit", buffer+3, 4)) break;
			else if(!memcmp("start", buffer+3, 5)) {
				//printf("ingore %d entries\n", s_number);
			}
			/*
			else if(!memcmp("threshold", buffer+3, 9)) {
				s_threshold = (DATA_TYPE)atoi(buffer+13);
				printf("threshold update to %d\n", s_threshold);
			}
			*/
			else if(!memcmp("desc", buffer+3, 4)) {
				printf("description:%s\n", buffer+8);
			}
		}
		if(sscanf(buffer, "%f %f %f %f %f %f %f %f %f %f", &time, 
		   &accx, &accy, &accz, &gyrx, &gyry, &gyrz, &magx, &magy, &magz) == 10) {
			//update_gyro(time, gyrx, gyry, gyrz);
			//update_quat(time, gyrx - s_gyro_calx, gyry - s_gyro_caly, gyrz - s_gyro_calz);
			update_acc(time, accx, accy, accz);
			update_gyro(time, gyrx, gyry, gyrz);
			//update_acc(convert_acc(accx), convert_acc(accy), convert_acc(accz));
		}
	}
	fclose(file);
	return 0;
}
