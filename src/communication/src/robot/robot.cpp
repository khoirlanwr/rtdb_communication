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

#include <stdio.h>
#include <ros/ros.h>
#include <pthread.h>
#include <unistd.h>
#include <sys/time.h>
#include <sched.h>
#include <stdlib.h>
#include <signal.h>
#include "communication/multicast.h"
#include "communication/rtdb_api.h"
#include "communication/rtdb_defs.h"
#include "communication/rtdb_user.h"
#include "communication/struct_file.h"


int end = 0;

static void signal_catch(int sig)
{
	if (sig == SIGINT)
		end = 1;
}


int main(int argc, char *argv[])
{
	// creating object from structure
	vision my_vision;
	pos my_pos;
	pos other_pos;

	int ret, other_agent;
	int lifetime;

	ros::init(argc, argv, "Robot__Node");
	ros::NodeHandle robotNode;

	if((signal(SIGINT, signal_catch)) == SIG_ERR)
	{
		printf("Error registering signal handle.\n");
		return -1;
	}

	if (DB_init() != 0)
	{
		printf("Error DB_init function.\n");
		return -1;
	}

	printf("Insert an integer [0..255] value for the vision[0][0]\n");
	ret = scanf("%d", (int*)&(my_vision[0][0]));

	if (DB_put(VISION, my_vision) == -1)
	{
		DB_free();
		return -1;
	}

	while (!end)
	{
		// TODO
		printf("\nInsert a X position.\n");
		ret = scanf("%f", &my_pos.x);

		if (DB_put(LOC, &my_pos) == -1)
		{
			DB_free();
			return -1;
		}

		lifetime = DB_get(Whoami()==Robot1?Robot2:Robot1, LOC, &(other_pos));
		printf("Other X position: %f\t\tData age: %d ms\n", other_pos.x, lifetime);

		lifetime = DB_get(Whoami(), VISION, my_vision);
		printf("My vision: %d\t\tData age: %d ms", my_vision[0][0], lifetime);
	}

	DB_free();


	return 0;
}