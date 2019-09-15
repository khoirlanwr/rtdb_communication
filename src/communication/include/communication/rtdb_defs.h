/* **********************************************************************
 *	Reference	: https://bitbucket.org/fredericosantos/rtdb/wiki/Home  
 * 	Modified by	: Khoirul Anwar  
 * 				  Computer Engineering 2017 - PENS 
 * 				  2210171032
 * 			
 * 				  Just do it everyday with small starts step !!! 
 *  		      BISMILLAH ERSOW JUARA KRI 2020 !!!
 *					
 */


#ifndef __RTDBDEFS_H
#define __RTDBDEFS_H

// #define DEBUG

#define SELF 3333

#define CONFIG_FILE	"/home/khoirlnwar/communication/src/config/rtdb.ini"

#define SHMEM_KEY 0x2000
#define SHMEM_SECOND_TEAM_KEY 0x3000

#define MAX_AGENTS 10	// Jumlah max agents
#define MAX_RECS 100	// Jumlah record max shared + local (shared + local)

typedef struct
{
	int id;				
	int size;			
	int period;			
} RTDBconf_var;

#endif
