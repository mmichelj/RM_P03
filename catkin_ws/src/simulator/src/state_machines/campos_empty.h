/***********************************************
*                                              *
*      campos_empty.h	                       *
*                                              *
*      Diego Cordero                           *
*      Jesus Savage			                   *
*      Miguel Michel			               *
*					                           *
*                                              *
*              Bio-Robotics Laboratory         *
*              UNAM, 2019                      *
*                                              *
*                                              *
************************************************/
/*Esta funcion recibe los datos de luz de los sensores, encuentra la posición del destino, calcula la fuerza de atracción y mueve al robot*/
#include <stdio.h> 
#include <math.h> 

#define THRESHOLD_FOLLOWER 30

 

int campos_empty(float intensity,float *light_values,movement *movements,float max_advance, float max_turn_angle)
{

 int sensor = 0;
 int i=0;
 int result = 0;
 float thd=0;
 float Dd=0;
 float qdest[2]={0,0};
 float Fatr[2]={0,0};
 float qn[2]={0,0};
 float mqn=0;
 float thqn=0;
 float etha=0.8;
 float mF=0;
 float max_light_intensity=0;

 if(intensity > THRESHOLD_FOLLOWER)
 {

	movements->twist = 0.0;
 	movements->advance = 0.0;
	result = 1;
	printf("\n **************** Reached light source ******************************\n");
 }
 else
 {
	 for(i = 1; i < 8; i++) 
 	{
		printf("\n **************** Light values %f******************************\n", light_values[i]);
	    if( light_values[i] > light_values[sensor])
		sensor = i;
		max_light_intensity=light_values[sensor];
 	}
 	
 	if(sensor > 4)
	   sensor = -(8 - sensor);


	Dd=0.9979*pow(max_light_intensity, -0.8495);
	thd = sensor * 3.1315 / 16;

	qdest[0]=Dd*cos(thd);
	qdest[1]=Dd*sin(thd);

	printf("\n **************** qdestx=%f qdesty=%f ******************************\n",qdest[0],qdest[1]);

	Fatr[0]=-etha*qdest[0];
	Fatr[1]=-etha*qdest[1];

	mF=sqrt(pow(Fatr[0],2)+pow(Fatr[1],2));
	Fatr[0]=Fatr[0]/mF;
	Fatr[1]=Fatr[1]/mF;

	qn[0]=-max_advance*Fatr[0];
	qn[1]=-max_advance*Fatr[1];

	mqn=sqrt(pow(qn[0],2)+pow(qn[1],2));
	thqn=atan(qn[1]/qn[0]);
	
	movements->twist = thqn;
 	movements->advance = mqn;
 }

 return result;

}
