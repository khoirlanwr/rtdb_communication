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


typedef char vision[800][600];

typedef struct _vec
{
	float x;
	float y;
} pos;

enum
{
	STOP,
	START,
	OUR_CORNER,
	THEIR_CORNER,
	OUR_KICKOFF,
	THEIR_KICKOFF
};

int refBoxCommand;

