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
#include <signal.h>
#include <stdlib.h>
#include "communication/multicast.h"
#include "communication/rtdb_api.h"
#include "communication/rtdb_defs.h"
#include "communication/rtdb_user.h"

#define TTUP_US 		100E3
#define BUFFER_SIZE 	1400
#define COMM_DELAY_US	1500
#define MIN_UPDATE_DELAY_US 1000


#define PERRNO(txt) \
	printf("ERROR: (%s / %s): " txt ": %s\n", __FILE__, __FUNCTION__, strerror(errno))

#define PERR(txt, par...) \
	printf("ERROR: (%s / %s): " txt "\n", __FILE__, __FUNCTION__, ## par)

#ifdef DEBUG
#define PDEBUG(txt, par...) \
	printf("DEBUG: (%s / %s): " txt "\n", __FILE__, __FUNCTION__, ## par)
#else
#define PDEBUG(txt, par...)
#endif

#define NOT_RUNNING 0
#define RUNNING 	1
#define INSERT 		2
#define REMOVE 		3
#define MAX_REMOVE_TICKS 10

#define NO  0
#define YES 1	

int end, timer, delay;
int lostPacket[MAX_AGENTS];
int myNumber;

struct timeval lastSendTimeStamp;
int MAX_DELTA;
int RUNNING_AGENTS;

struct _frameHeader
{
	unsigned char number; // agent number
	unsigned int counter; // frame counter
	char stateTable[MAX_AGENTS]; // vision for each agents
	int noRecs; // number of records 
}; 

// frame[1].stateTable[0] = NOT_RUNNING
// frame[1].stateTable[1] = RUNNING
// etc 

struct _agent
{
	char state; 		// RUNNING, NOT_RUNNING, INSERT, REMOVE
	char dynamicID; 	// this dynamicID is assigned to only active agents
	char received; 		// to determines agents packet is received or not
	struct timeval receiveTimeStamp; // last receive timestamp
	int delta;			// different between expected and real 
	unsigned int lastFrameCounter; 	// frame number
	char stateTable[MAX_AGENTS]; // vision of each agent state
	int removeCounter;	// counter to remove agents 
}; 

struct _agent agent[MAX_AGENTS];

// ************************
// signal catch
// 
static void signal_catch(int sig)
{
	if (sig == SIGINT) 
		end = 1;
	
	else 
		if (sig == SIGALRM)
			timer++;

}

void update_stateTable()
{
	// TODO 
	int i, j;

	for(i=0; i<MAX_AGENTS; i++)
	{
		if (i != myNumber) 
		{
			switch (agent[i].state) 
			{
				case RUNNING: 
					if(agent[i].received == NO)
						agent[i].state = REMOVE;
				break;

				case NOT_RUNNING:
					if(agent[i].received == YES)
						agent[i].state = INSERT;
				break;

				case INSERT:
					if(agent[i].received == NO)
						agent[i].state = NOT_RUNNING;
					else {
						for (j=0; j<MAX_AGENTS; j++) 
							if ((agent[j].state == RUNNING) && 
								(agent[j].stateTable[i] == NOT_RUNNING) || (agent[j].stateTable[i] == REMOVE))
								break;

						agent[i].state = RUNNING;
					}
				break;

				case REMOVE:
					if (agent[i].received == YES) {
						agent[i].removeCounter = 0;
						agent[i].state = RUNNING;
					}
					else {
						for(j=0; j<MAX_AGENTS; j++) 
							if ((agent[j].state == RUNNING) && 
								(agent[j].stateTable[i] == RUNNING) || (agent[j].stateTable[i] == INSERT))
								break;

						agent[i].removeCounter ++;
						if (agent[i].removeCounter >= MAX_REMOVE_TICKS) 
						{
							agent[i].state = NOT_RUNNING;
							agent[i].removeCounter = 0;
						}

					}

				break;

			} // end switch

		} // end if

	} // end for

	agent[myNumber].state = RUNNING;
}


int sync_ratdma(int agentNumber)
{
	// TODO
	unsigned int realDiff, expectedDiff;
	struct itimerval it;

	agent[agentNumber].received = YES;


	if ((agent[agentNumber].state == NOT_RUNNING) || (agent[agentNumber].state == INSERT))
	{
		PDEBUG("******** AGENT %d - NOT_RUNNING or INSERT *******", agentNumber);
		return (1);
	}


	realDiff = (int)((agent[agentNumber].receiveTimeStamp.tv_sec - lastSendTimeStamp.tv_sec)*1E6 + agent[agentNumber].receiveTimeStamp.tv_usec - lastSendTimeStamp.tv_usec);
	realDiff -= (int)(COMM_DELAY_US);

	expectedDiff = (int)((agent[agentNumber].dynamicID - agent[myNumber].dynamicID) * (TTUP_US/RUNNING_AGENTS));
	if (expectedDiff < 0)
		expectedDiff += (int)TTUP_US;

	agent[agentNumber].delta = realDiff - expectedDiff;


	// if dynamicID of agent is 0
	// only agent 0 can make adjustments
	if (agent[myNumber].dynamicID == 0) 
	{
		if ((agent[agentNumber].delta < MAX_DELTA) && (agent[agentNumber].delta > delay))
		{
			if (agent[agentNumber].delta > (int)MIN_UPDATE_DELAY_US) 
			{
				delay = agent[agentNumber].delta;
				PDEBUG("Delay between my Agent %d(%d) and %d(%d) -> %d us", myNumber, agent[myNumber].dynamicID, agentNumber, agent[agentNumber].dynamicID, delay);
			}	
		}
	}

	else 
	{
		if (agent[agentNumber].dynamicID == 0) 
		{
			expectedDiff = (int)(TTUP_US - expectedDiff);
			expectedDiff -= (int)COMM_DELAY_US;
			it.it_value.tv_usec = (long int)(expectedDiff % (int)1E6);
			it.it_value.tv_sec = (long int)(expectedDiff / (int)1E6);
			it.it_interval.tv_usec = (__suseconds_t)(TTUP_US);
			it.it_interval.tv_sec = 0;
			setitimer(ITIMER_REAL, &it, NULL);

		}
	}


	return 0;

}


void *receiveDataThread(void *socketDesc)
{
	// TODO
	char recvBuffer[BUFFER_SIZE];
	struct _frameHeader frameHeader;
	
	int agentNumber;
	RTDBconf_var rec;

	int life, size, indexBuffer;

	while (!end) 
	{
		// reset all data in address
		bzero(recvBuffer, BUFFER_SIZE);

		if ((receiveData(*(int*)socketDesc, recvBuffer, BUFFER_SIZE)) > 0)
		{	
			// start indexBuffer from 0
			indexBuffer = 0;

			// copy the frame Header from recvBuffer to frameHeader object directly using address
			memcpy (&frameHeader, recvBuffer + indexBuffer, sizeof(frameHeader));
			indexBuffer += sizeof(frameHeader);

			// get the number of agent
			agentNumber = frameHeader.number;

			// receive from ourself
			// not supposed to occur, just to prevent!
			if(agentNumber == myNumber) 
				continue;

			// get time when packet from agent received
			gettimeofday(&(agent[agentNumber].receiveTimeStamp), NULL);

			// counting lost packets 
			// get lastFrameCounter for this agent
			if ((agent[agentNumber].lastFrameCounter + 1) != frameHeader.counter)
				lostPacket[agentNumber] = frameHeader.counter - agent[agentNumber].lastFrameCounter;			
			agent[agentNumber].lastFrameCounter = frameHeader.counter;

			// update stateTable of each agent in this agent perception in our RTDB 
			for(int i=0; i<MAX_AGENTS; i++)
				agent[agentNumber].stateTable[i] = frameHeader.stateTable[i];


			// looping for MAX_RECORDS then copying it spesicied memory address
			for (int i = 0; i < MAX_RECS; i++)
			{	
				// copying id of records 
				memcpy(&rec.id, recvBuffer + indexBuffer, sizeof(rec.id));
				indexBuffer += sizeof(rec.id);

				// copying size of records
				memcpy(&rec.size, recvBuffer + indexBuffer, sizeof(rec.size));
				indexBuffer += sizeof(rec.size);

				// copying periods of records
				memcpy(&life, recvBuffer + indexBuffer, sizeof(life));
				indexBuffer += sizeof(life);

				life += (int)(COMM_DELAY_US/1E3);

				if ((size = DB_comm_put(agentNumber, rec.id, rec.size, recvBuffer + indexBuffer, life)) != (int)(rec.size))
				{	
					PERR("Error in frame/rtdb: from = %d, id item = %d, received size = %d, local size = %d", agentNumber, rec.id, rec.size, size);
					break;
				}

				indexBuffer += rec.size;
			} // end looping for 

#ifndef UNSYNC
	sync_ratdma(agentNumber);
#endif		
		} // end if
	}

}


int main(int argc, char *argv[])
{
	int socketDesc;
	RTDBconf_var rec[MAX_RECS];
	int n_shared_recs;
	
	pthread_attr_t thread_attr;
	pthread_t recvThread;
	
	struct itimerval it;
	struct sched_param proc_sched;
	int i, j, life;

	int indexBuffer = 0;
	char sendBuffer[BUFFER_SIZE];

	int counterRunningAgents, counterFrame;
	struct _frameHeader frameHeader; 
	struct timeval tempTimeStamp;

	// reset all flag
	timer=0;
	delay=0;
	end=0;

	ros::init(argc, argv, "Communication_Node");
	ros::NodeHandle commNode;

	// sets scheduling process
	proc_sched.sched_priority=60;
	if ((sched_setscheduler(getpid(), SCHED_FIFO, &proc_sched)) < 0)
	{
		PERRNO("setshceduler");
		return -1;
	}

	// set this function signal_catch
	// to handle SIGLARM signal
	if (signal(SIGALRM, signal_catch) == SIG_ERR)
	{
		PERRNO("signal");
		return -1;
	}

	if (signal(SIGINT, signal_catch) == SIG_ERR)
	{
		PERRNO("signal");
		return -1;
	}

	// opening socket, using this param
	// enp3s0 for Ethernet
	// wlp2s0 for Wireless
	if ((socketDesc = openSocket("enp3s0")) == -1)
	{
		PERR("openMulticastSocket");
		printf("\nUsage: comm <interface_name>\n\n");
		return -1;
	}

	// init DB 
	if (DB_init() == -1)
	{
		PERRNO("DB_init");
		closeSocket(socketDesc);
		return -1;
	}  


	// // get number of shared records
	if((n_shared_recs = DB_comm_ini(rec)) < 1)
	{
		PERR("DB_comm_ini");
		DB_free();
		closeSocket(socketDesc);

		return -1;
	} 

	/* initialize reset */
	for (i=0; i<MAX_AGENTS; i++) 
	{
		lostPacket[i] = 0;
		agent[i].lastFrameCounter = 0;
		agent[i].state = NOT_RUNNING;
		agent[i].removeCounter = 0;
	}

	myNumber = Whoami();
	agent[myNumber].state = RUNNING;

	// thread receive data
	pthread_attr_init (&thread_attr);
	pthread_attr_setinheritsched (&thread_attr, PTHREAD_INHERIT_SCHED);
	if ((pthread_create(&recvThread, &thread_attr, receiveDataThread, (void *)&socketDesc)) != 0)
	{	
		PERRNO("pthread_create");
		DB_free();
		closeSocket(socketDesc);

		return -1;
	}


	// set the round periods 100,000 us or 100 ms
	it.it_value.tv_usec=(__suseconds_t)(TTUP_US);
	it.it_value.tv_sec=0;
	it.it_interval.tv_usec=(__suseconds_t)(TTUP_US);
	it.it_interval.tv_sec=0;
	setitimer(ITIMER_REAL, &it, NULL);

	counterFrame = 0;

	while (!end)
	{
		// 
		pause();

		if (timer == 0)
			continue;


#ifndef UNSYNC
		// dynamic agent 0
		if ((delay > (int)MIN_UPDATE_DELAY_US) && (agent[myNumber].dynamicID == 0) && (timer == 1))
		{
			it.it_value.tv_usec = (__suseconds_t)(delay - (int)MIN_UPDATE_DELAY_US/2);
			it.it_value.tv_sec = 0;
			it.it_interval.tv_usec = (__suseconds_t)(TTUP_US);
			it.it_interval.tv_sec = 0;
			setitimer(ITIMER_REAL, &it, NULL);
			delay = 0;
			continue;
		}
#endif
		timer = 0;

		indexBuffer = 0;
		bzero(sendBuffer, BUFFER_SIZE);
		
		update_stateTable();

		// update RUNNING_AGENTS
		counterRunningAgents = 0;

		for(i=0; i<MAX_AGENTS; i++) 
		{
			if ((agent[i].state == RUNNING) || (agent[i].state == REMOVE))
			{
				agent[i].dynamicID = counterRunningAgents;
				counterRunningAgents ++;
			}

			agent[myNumber].stateTable[i] = agent[i].state;
		}

		RUNNING_AGENTS = counterRunningAgents;

		MAX_DELTA = (int)(2/3 * (TTUP_US/RUNNING_AGENTS));


		// start packing data and frame header
		// set the frameHeader
		frameHeader.number = myNumber;
		frameHeader.counter = counterFrame;
		counterFrame ++;

		for (i=0; i<MAX_AGENTS; i++)
			frameHeader.stateTable[i] = agent[myNumber].stateTable[i];

		frameHeader.noRecs = n_shared_recs;

		memcpy(sendBuffer + indexBuffer, &frameHeader, sizeof(frameHeader));
		indexBuffer += sizeof(frameHeader);


		for(i=0; i<n_shared_recs; i++)
		{
			memcpy(sendBuffer + indexBuffer, &rec[i].id, sizeof(rec[i].id));
			indexBuffer += sizeof(rec[i].id);

			memcpy(sendBuffer + indexBuffer, &rec[i].size, sizeof(rec[i].size));
			indexBuffer += sizeof(rec[i].size);

			life = DB_get(myNumber, rec[i].id, sendBuffer + indexBuffer + sizeof(life));
			memcpy(sendBuffer + indexBuffer, &life, sizeof(life));

			indexBuffer = indexBuffer + sizeof(life) + rec[i].size;  
		}

		// end packing data 
		if (indexBuffer > BUFFER_SIZE) 
		{
			PERR("Pretended frame is bigger than available buffer.");
			PERR("Please increase the buffer size or reduce the number of dissemination records");
			break;
		}

		// sending data by socket
		if (sendData(socketDesc, sendBuffer, indexBuffer) != indexBuffer) 
			PERRNO("Error sending data");

		// get time when sending packet(for syncronization calculation) 
		gettimeofday(&tempTimeStamp, NULL);
		lastSendTimeStamp.tv_sec  = tempTimeStamp.tv_sec;
		lastSendTimeStamp.tv_usec = tempTimeStamp.tv_usec;

		for(i=0; i<MAX_AGENTS; i++) 
		{
			agent[i].delta = 0;
			agent[i].received = NO;
		}	

	}	

	printf("Communication: STOPPED.\nCleaning process ...\n");

	// close socket
	closeSocket(socketDesc);

	// Join thread
	pthread_join(recvThread, NULL);

	// freeing memory 
	DB_free();
	
	// 
	printf("Communication: FINISHED\n");

	return 0;
}