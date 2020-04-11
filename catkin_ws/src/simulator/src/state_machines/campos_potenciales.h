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

 

int campos_potenciales(float intensity,float *light_values,movement *movements,float max_advance, float max_turn_angle,float *observations, int size, float laser_value, float ang_origen, float ang_final)
{
	/*
      IMPORTANTE: El angulo del sensor de en medio es 0. Los de la izquierda son +. Los de la derecha 2*pi-el angulo.
    */

//Constantes del comportamiento del robot
float etha=3000;
float n=0.000008; //5
//


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
 float mF=0;
 float max_light_intensity=0;

 float Frep[2]={0,0};
 float magFrep=0;
 float d=5;
 float thobs=0;
 float qobs[2]={0,0};

 float Ftot[2]={0,0};
 float mpi=3.1415;

 //borrar
 float pFrep[2]={0,0};
 float maxFrep[2]={0,0};
 float prevFrep[2]={0,0};

 
int a,b;
int iz,de,salida;
int j;
int par_flag=0;

iz = de = salida = 0;

 if(intensity > THRESHOLD_FOLLOWER)
 {

	movements->twist = 0.0;
 	movements->advance = 0.0;
	result = 1;
	printf("\n **************** Reached light source ******************************\n");
 }
 else
 {
	 //********** Fuerza de atracción ************//
	 for(i = 1; i < 8; i++) 
 	{
		//printf("\n **************** Light values %f******************************\n", light_values[i]);
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

	//printf("\n **************** qdestx=%f qdesty=%f ******************************\n",qdest[0],qdest[1]);

	Fatr[0]=-etha*qdest[0];
	Fatr[1]=-etha*qdest[1];

	mF=sqrt(pow(Fatr[0],2)+pow(Fatr[1],2));
	Fatr[0]=Fatr[0]/mF;
	Fatr[1]=Fatr[1]/mF;

	if(Dd<0.02){
		Fatr[0]=Fatr[0]*2;
		Fatr[1]=Fatr[1]*2;
	}

	//********** Fuerza de repulsión ************//

	d=5;

	if( size % 2 != 0)
    {
        a = ( size - 1 ) / 2;
        b = a + 1;
		par_flag=0;
    }
    else
    {
        a = b = size / 2;
		par_flag=1;
    }

	
	
	//Para todas las lecturas de los sensores
	for(int i=0;i<size;i++){
		//printf("************** observations[%d]: %f ***************\n", i, observations[i]);

		if(observations[i]<laser_value){
			//Calcular magnitud de la fuerza de repulsión para cada sensor
			//La fuerza de repulsion solo se calcula para los sensores que estan dentro del rango
			
			//Para obtener qobs_x y qobs_y:

			thobs=ang_origen+i*(ang_origen+ang_final)/size;

			qobs[0]=observations[i]*cos(thobs);
			qobs[1]=observations[i]*sin(thobs);
			//printf("***observations[%d]: %f thobs[%d]: %f qobs[0]=%f qobs[1]=%f cos(%f)=%f sin(%f)=%f ***\n", i, observations[i],i,thobs,qobs[0],qobs[1],thobs,cos(thobs),thobs,sin(thobs));
			//printf("************** cos(thobs)[%d]=%f sin(thobs)[%d]= %f fabs(qobs[0])=%f fabs(qobs[1])=%f***************\n", i, cos(thobs),i,sin(thobs),fabs(qobs[0]),fabs(qobs[1]));
			//Para obtener Frep_x y Frep_y
			
			if(Dd>observations[i]){
				if(fabs(qobs[0])>0.01){
					pFrep[0]=-n*(1/fabs(qobs[0])-1/d)*(1/(pow(qobs[0],2)));
					Frep[0]=Frep[0]-n*(1/fabs(qobs[0])-1/d)*(1/(pow(qobs[0],2)));
					maxFrep[0]= (pFrep[0]<prevFrep[0])?prevFrep[0]:pFrep[0];
					prevFrep[0]=pFrep[0];
				}
				if(fabs(qobs[1])>0.01){
					Frep[1]=Frep[1]-n*(1/fabs(qobs[1])-1/d)*(1/(pow(qobs[1],2)));
					pFrep[1]=-n*(1/fabs(qobs[1])-1/d)*(1/(pow(qobs[1],2)));
					maxFrep[1]= (pFrep[1]<prevFrep[1])?prevFrep[1]:pFrep[1];
					prevFrep[1]=pFrep[1];
				}	
			}
			if(observations[i]<0.02){
				Frep[0]=Frep[0];
				Frep[1]=Frep[1];
			}
			//printf("************** Frep[%d]: (%f,%f) maxFrep[0]=%f maxFrep[1]=%f***************\n\n", i, pFrep[0],pFrep[1],maxFrep[0],maxFrep[1]);
		}
	}


	printf("************** Frep[%d]: (%f,%f) Fatr[%d]=(%f,%f)***************\n\n", i, Frep[0],Frep[1],i,Fatr[0],Fatr[1]);

	Ftot[0]=1.5*Fatr[0]+Frep[0];
	Ftot[1]=1.5*Fatr[1]+Frep[1];

	printf("******** Ftot[0]=%f Ftot[1]=%f ********", Ftot[0],Ftot[1]);

	//********** Gradient descent ************//
	qn[0]=-max_advance*Ftot[0];
	qn[1]=-max_advance*Ftot[1];


	//********** Movimiento del robot ************//

	mqn=sqrt(pow(qn[0],2)+pow(qn[1],2));
	thqn=atan(qn[1]/qn[0]);

	if(mqn<0 && mqn<-max_advance){
		mqn=-max_advance;
	} else if(mqn>0 && mqn>max_advance){
		mqn=max_advance;
	}
	
	movements->twist = thqn;
 	movements->advance = mqn;
 }

 return result;

}
