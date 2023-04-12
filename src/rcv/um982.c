#include "rtklib.h"

#define UM982SYNC1   0xAA        /* um982 message start sync code 1 */
#define UM982SYNC2   0x44        /* um982 message start sync code 2 */
#define UM982SYNC3   0xB5        /* um982 message start sync code 3 */

#define UM982HLEN    24          /* um982 message header length (bytes) */

#define ID_OBSVM	 12         /* message id: um982 channel observations */


/* get fields (little-endian) ------------------------------------------------*/
static unsigned short U2(unsigned char *p) {unsigned short u; memcpy(&u,p,2); return u;}
static unsigned int   U4(unsigned char *p) {unsigned int   u; memcpy(&u,p,4); return u;}
static int            I4(unsigned char *p) {int            i; memcpy(&i,p,4); return i;}
static float          R4(unsigned char *p) {float          r; memcpy(&r,p,4); return r;}
static double         R8(unsigned char *p) {double         r; memcpy(&r,p,8); return r;}

/* get observation data index ------------------------------------------------*/
static int obsindex(obs_t *obs, gtime_t time, int sat)
{
    int i,j;

    if (obs->n>=MAXOBS) return -1;
    for (i=0;i<obs->n;i++) {
        if (obs->data[i].sat==sat) return i;
    }
    obs->data[i].time=time;
    obs->data[i].sat=sat;
    for (j=0;j<NFREQ+NEXOBS;j++) {
        obs->data[i].L[j]=obs->data[i].P[j]=0.0;
        obs->data[i].D[j]=0.0;
        obs->data[i].SNR[j]=obs->data[i].LLI[j]=0;
        obs->data[i].code[j]=CODE_NONE;
    }
    obs->n++;
    return i;
}
/* sync header ---------------------------------------------------------------*/
static int sync_um982(unsigned char *buff, unsigned char data)
{
    buff[0]=buff[1]; buff[1]=buff[2]; buff[2]=data;
	return buff[0]==UM982SYNC1&&buff[1]==UM982SYNC2&&buff[2]==UM982SYNC3;
}

/* decode um982 tracking status -------------------------------------------------
* deocode um982 tracking status
* args   : unsigned int stat I  tracking status field
*          int    *sys   O      system (SYS_???)
*          int    *code  O      signal code (CODE_L??)
*          int    *plock O      phase-lock flag   (0=not locked, 1=locked)
*          int    *clock O      code-lock flag    (0=not locked, 1=locked)
* return : signal frequency (0:L1,G1,E1,B1
							 1:L2,G2,B1-2
							 2:L5,E5a,B2a
							 3:L6,E6,B3,G2a
							 4:L7,E5b,B2,B2b
							 5:L8,E5a+b,B2a+b
							 -1:error)
* notes  : refer [1][3]
*-----------------------------------------------------------------------------*/
static int decode_trackstat(unsigned int stat, int *sys, int *code, int *plock,
							int *clock)
{
	int satsys,sigtype,l2type,freq=0;
	satsys =(stat>>16)&7;
	*plock =(stat>>10)&1;
	*clock =(stat>>12)&1;
	sigtype=(stat>>21)&0x1F;
    l2type =(stat>>26)&1;

    switch (satsys) {
        case 0: *sys=SYS_GPS; break;
        case 1: *sys=SYS_GLO; break;
		case 2: *sys=SYS_SBS; break;
		case 3: *sys=SYS_GAL; break;
		case 4: *sys=SYS_CMP; break;
		case 5: *sys=SYS_QZS; break;
        default:
			trace(2,"um982 unknown system: sys=%d\n",satsys);
            return -1;
	}
    if (*sys==SYS_GPS||*sys==SYS_QZS) {
		switch (sigtype) {
			case  0: freq=0; *code=CODE_L1C; break; // L1C/A
			case  3: freq=0; *code=CODE_L1L; break; // L1C pilot
			case 11: freq=0; *code=CODE_L1S; break; // L1C data semicodeless
			case  9: freq=1;
					 *code = l2type ? CODE_L2S : CODE_L2W;
					 break; 						// L2C(M) or L2P
			case 17: freq=1; *code=CODE_L2L; break; // L2C(L)
			case 14: freq=2; *code=CODE_L5Q; break; // L5 pilot
			case  6: freq=2; *code=CODE_L5I; break; // L5 data
			default: freq=-1; break;
        }
    }
    else if (*sys==SYS_GLO) {
		switch (sigtype) {
			case  0: freq=0; *code=CODE_L1C; break; // L1C/A
			case  5: freq=1; *code=CODE_L2C; break; // L2C/A
			case  6: freq=2; *code=CODE_L3I; break; // G3I
			case  7: freq=2; *code=CODE_L3Q; break; // G3Q
			default: freq=-1; break;
        }
    }
    else if (*sys==SYS_GAL) {
		switch (sigtype) {
			case  1: freq=0; *code=CODE_L1B; break; // E1B
			case  2: freq=0; *code=CODE_L1C; break; // E1C
			case 12: freq=2; *code=CODE_L5Q; break; // E5aQ (pilot)
			case 17: freq=4; *code=CODE_L7Q; break; // E5bQ (pilot)
			case 18: freq=3; *code=CODE_L6B; break; // E6B
			case 22: freq=3; *code=CODE_L6C; break; // E6C
            default: freq=-1; break;
        }
	}
    else if (*sys==SYS_CMP) {
        switch (sigtype) {
			case  0: freq=1; *code=CODE_L2I; break; // B1I
			case  4: freq=1; *code=CODE_L2Q; break; // B1Q
			case  8: freq=0; *code=CODE_L1L; break; // B1C (pilot)
			case 23: freq=0; *code=CODE_L1S; break; // B1C (data)
			case  5: freq=4; *code=CODE_L7Q; break; // B2Q
			case 17: freq=4; *code=CODE_L7I; break; // B2I
			case 12: freq=2; *code=CODE_L5Q; break; // B2a (pilot)
			case 28: freq=2; *code=CODE_L5I; break; // B2a (data)
			case  6: freq=3; *code=CODE_L6Q; break; // B3Q
			case 21: freq=3; *code=CODE_L6I; break; // B3I
			case 13: freq=4; *code=CODE_L7I; break; // B2b(I)
            default: freq=-1; break;
		}
    }
    else if (*sys==SYS_SBS) {
        switch (sigtype) {
			case  0: freq=0; *code=CODE_L1C; break; // L1C/A
			case  6: freq=2; *code=CODE_L5I; break; // L5I
			default: freq=-1; break;
        }
	}
    if (freq<0) {
        trace(2,"um982 signal type error: sys=%d sigtype=%d\n",*sys,sigtype);
        return -1;
	}
    return freq;
}
/* check code priority and return obs position -------------------------------*/
static int checkpri(const char *opt, int sys, int code, int freq)
{
	int nex=NEXOBS; /* number of extended obs data */

	if (sys==SYS_GPS) {
		if (strstr(opt,"-GL1P")&&freq==0) return code==CODE_L1P?0:-1;
		if (strstr(opt,"-GL2X")&&freq==1) return code==CODE_L2X?1:-1;
		if (code==CODE_L1P) return nex<1?-1:NFREQ;
		if (code==CODE_L2X) return nex<2?-1:NFREQ+1;
	}
	else if (sys==SYS_GLO) {
		if (strstr(opt,"-RL2C")&&freq==1) return code==CODE_L2C?1:-1;
		if (code==CODE_L2C) return nex<1?-1:NFREQ;
	}
    else if (sys==SYS_GAL) {
        if (strstr(opt,"-EL1B")&&freq==0) return code==CODE_L1B?0:-1;
        if (code==CODE_L1B) return nex<1?-1:NFREQ;
		if (code==CODE_L7Q) return nex<2?-1:NFREQ+1;
		if (code==CODE_L8Q) return nex<3?-1:NFREQ+2;
    }
    return freq<NFREQ?freq:-1;
}

/* decode obsvm ----------------------------------------------------------*/
static int decode_obsvm(raw_t *raw)
{
	double psr,adr,lockt,tt,dop,snr;
	int i,index,nobs,sysf,prn,sat,freq,pos;
	int sys,code,plock,clock;
	char *msg;
	unsigned char *p=raw->buff+UM982HLEN;

	trace(3,"decode_obsvm: len=%d\n",raw->len);

	nobs=U4(p);

	if (raw->outtype) {
		msg=raw->msgtype+strlen(raw->msgtype);
		sprintf(msg," nobs=%2d",nobs);
	}
	if (raw->len<UM982HLEN+4+nobs*40) {
		trace(2,"um982 obsvm length error: len=%d nobs=%d\n",raw->len,nobs);
		return -1;
	}
	for (i=0,p+=4;i<nobs;i++,p+=40) {

		sysf = U2(p);
		if (sysf != 0) prn-=37;
		prn=U2(p+2);
		psr = R8(p+4);
		adr = R8(p+12);
		dop = R4(p+24);
		lockt = R4(p+32);
        snr = U2(p+28)*0.01;

		// decode tracking status
		if ((freq=decode_trackstat(U4(p+36),&sys,&code,&plock,&clock))<0) continue;

		// obs position
		if ((pos=checkpri(raw->opt,sys,code,freq))<0) continue;

		if (!(sat=satno(sys,prn))) {
			trace(3,"um982 obsvm satellite number error: sys=%d,prn=%d\n",sys,prn);
			continue;
		}

		raw->tobs [sat-1][pos]=raw->time;
		raw->lockt[sat-1][pos]=lockt;

		if (!clock) psr=0.0;     /* code unlock */
		if (!plock) adr=dop=0.0; /* phase unlock */

		if (fabs(timediff(raw->obs.data[0].time,raw->time))>1E-9) {
			raw->obs.n=0;
		}
		if ((index=obsindex(&raw->obs,raw->time,sat))>=0) {
			raw->obs.data[index].L  [pos]=adr;
			raw->obs.data[index].P  [pos]=psr;
			raw->obs.data[index].D  [pos]=(float)dop;
			raw->obs.data[index].SNR[pos]=
				0.0<=snr&&snr<255.0?(unsigned char)(snr+0.5):0;
			raw->obs.data[index].code[pos]=code;
		}
	}
	return 1;
}


/* decode um982 message -------------------------------------------------------*/
static int decode_um982(raw_t *raw)
{
	double tow;
	int msg,week,type=U2(raw->buff+4);

	trace(3,"decode_um982: type=%3d len=%d\n",type,raw->len);

	if (!(week=U2(raw->buff+10))) {
		return -1;
	}
	week=adjgpsweek(week);
	tow =U4(raw->buff+12)*0.001;
	raw->time=gpst2time(week,tow);

    if (raw->outtype) {
		sprintf(raw->msgtype,"UM982 %4d (%4d): msg=%d %s",type,raw->len,msg,
                time_str(gpst2time(week,tow),2));
	}

	switch (type) {
		case ID_OBSVM: return decode_obsvm(raw);
	}
    return 0;
}

/* input um982 raw data from file ------------------------------------------
* fetch next unicorecomm um982 raw data and input a message from file
* args   : raw_t  *raw   IO     receiver raw data control struct
*          int    format I      receiver raw data format (STRFMT_???)
*          FILE   *fp    I      file pointer
* return : status(-2: end of file, -1...9: same as above)
*-----------------------------------------------------------------------------*/
extern int input_um982f(raw_t *raw, FILE *fp)
{
    int i,data;

    trace(4,"input_um982f:\n");

    /* synchronize frame */
    if (raw->nbyte==0) {
        for (i=0;;i++) {
            if ((data=fgetc(fp))==EOF) return -2;
            if (sync_um982(raw->buff,(unsigned char)data)) break;
            if (i>=4096) return 0;
        }
    }
	if (fread(raw->buff+3,21,1,fp)<1) return -2;
	raw->nbyte=24;

	if ((raw->len=U2(raw->buff+6)+UM982HLEN)>MAXRAWLEN-4) {
        trace(2,"um982 length error: len=%d\n",raw->len);
        raw->nbyte=0;
        return -1;
	}

	if (fread(raw->buff+24,raw->len+4-24,1,fp)<1) return -2;
	raw->nbyte=0;

	/* decode um982 message */
	return decode_um982(raw);
}

