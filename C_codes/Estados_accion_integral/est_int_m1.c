
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <math.h>
#include "supportLib.h"
#include "pbPlots.h"


pthread_t hilo_timer;
pthread_t hilo_m1_control;
pthread_t hilo_m1_dinamica;
pthread_attr_t attr;
pthread_mutex_t  count_mutex;
pthread_cond_t   count_limite_cv;
pthread_mutex_t  count_mutex_2;
pthread_cond_t   count_limite_cv_2;
pthread_mutex_t  count_mutex_3;
pthread_cond_t   count_limite_cv_3;

int id_main, id_timer, id_m1_control, id_m1_dinamica;
#define dt 0.02


//Calculo de señal de control para motor x
float Kix[1][3]={841.7093,130.6615,3.3561e3};
#define Kix11 841.7093
#define Kix12 130.6615
#define Kix13 3.3561e+03
float errint_x=0;
float ux=0;
float ref_x=0;

//Datos sensados del motor x
float xx_actual=0; 
float xv_actual=0;

//Calculo de señal de control para motor y
float Kiy[1][3]={1.6271e4,3.2524e3,6.4826e4};
#define Kiy11 1.6271e+04
#define Kiy12 3.2524e+03
#define Kiy13 6.4826e+04
float errint_y=0;
float uy=0;
float ref_y=0;

//Datos sensados del motor y
float yx_actual=0;
float yv_actual=0;

//Simulacion motor x
float Fric_X=0.2; //0.2
float Ffx = 0;
float xx_futuro=0; 
float xv_futuro=0;
float Ax_d[2][2]={{1,0.0189},{0,0.8906}};
#define Ax_d11 1
#define Ax_d21 0
#define Ax_d12 0.0189
#define Ax_d22 0.8906
float Bx_d[2][1]={{2.8408e-5},{0.0028}};
#define Bx_d11 2.8408e-05
#define Bx_d21 0.0028
float Wfx_d[2][1]={{0},{-0.0024}};
#define Wfx_d11 0
#define Wfx_d21 -0.0024

//Simulacion motor y
float Fric_Y=0.25; //0.25
float Ffy = 0; 
float yx_futuro=0; 
float yv_futuro=0;
float Ay_d[2][2]={{1,0.02},{0.02,0.9952}};
#define Ay_d11 Ay_d[1][1] //1
#define Ay_d21 Ay_d[2][1] //0
#define Ay_d12 Ay_d[1][2] //0.02
#define Ay_d22 Ay_d[2][2] //0.9952
float By_d[2][1]={{1.4440e-6},{1.4428e-4}};
#define By_d11 By_d[1][1] //1.4440e-06
#define By_d21 By_d[2][1] //1.4428e-04
float Wfy_d[2][1]={{-2.5720e-5},{-0.0026}};
#define Wfy_d11 Wfy_d[1][1] //-2.5720e-05
#define Wfy_d21 Wfy_d[2][1) //-0.0026

//Variables para correr el programa
#define limite_puntos 5500

//Variables para guardar la respuesta
int indice_salida_m1=0;
float salida_m1[limite_puntos];

int a;
int sucede_dinamica=0;

//Variables para leer referencia
int indice_referencia=0;
float ref_x_array[limite_puntos];

//Variables generales de programa
int fin=0;


void *timer(void *arg)
{
    id_timer=pthread_self() ;
    while(1){
	    sleep(dt);
	    pthread_mutex_lock(&count_mutex_2);
	    	pthread_cond_signal(&count_limite_cv_2);
	    pthread_mutex_unlock(&count_mutex_2);
    };
}



void *m1_control(void *arg)
{
    id_m1_control=pthread_self() ;
    //Señal al hilo_m1_control
    printf("Inicia m1_control\n");
    while(indice_salida_m1<limite_puntos){
    	    //printf("xc%d\n",indice_salida_m1);
            pthread_mutex_lock(&count_mutex_2);
               	pthread_cond_wait(&count_limite_cv_2, &count_mutex_2);
	    pthread_mutex_unlock(&count_mutex_2);
	   
	    while(sucede_dinamica==0){
	    };
	    
	    sucede_dinamica=0;
	      
	    pthread_mutex_lock(&count_mutex);
	    
	    	    //"Sensar posicion y velocidad"
		    xx_actual=xx_futuro;
		    xv_actual=xv_futuro;
		    //Leer referencia
		    //ref_x=1; //Punto de referencia deseada (se actualiza cada dt)//BBBBBBBBBBBBBB
		    ref_x=ref_x_array[indice_referencia];
		    indice_referencia++;
		    
		    //Ejecutar logica de control
		    errint_x = errint_x + (xx_actual - ref_x)*dt;	 //error de posicion X
		    //printf("err: %f\n",errint_x);
		    ux = -1*Kix11*xx_actual-1*Kix12*xv_actual-1*Kix13*errint_x; //Calculo de señal de control
		    pthread_cond_signal(&count_limite_cv);
		    
	    pthread_mutex_unlock(&count_mutex);
    };
    printf("Fin m1_control\n");
    pthread_exit(NULL);
}

void *m1_dinamica(void *arg)
{
    id_m1_dinamica=pthread_self() ; 
	
    printf("Inicia m1_dinamica\n");
    
    if(indice_salida_m1==0){
	    sucede_dinamica=1;
    };
    
    while(indice_salida_m1<limite_puntos){
            Ffx = Fric_X*xv_actual;  //fricción actual
            xx_futuro=Ax_d11*xx_actual+Ax_d12*xv_actual+Bx_d11*ux+Wfx_d11*Ffx;
            xv_futuro=Ax_d21*xx_actual+Ax_d22*xv_actual+Bx_d21*ux+Wfx_d21*Ffx;
            salida_m1[indice_salida_m1]=xx_futuro;
	    indice_salida_m1++;
	    
            //Señal al hilo_m1_control
          
            sucede_dinamica=1;
	    
	    if(indice_salida_m1!=limite_puntos){
            pthread_mutex_lock(&count_mutex);
    	    pthread_cond_wait(&count_limite_cv, &count_mutex); //
            pthread_mutex_unlock(&count_mutex);
            }
    };
    printf("Fin de dinamica\n");

}


void imprimirSalidaConsola(){
	int i;
	FILE *fp;
	//abrimos el archivo
	fp = fopen ("Prueba_step_m1" , "w");
	//-------------------------------------------------------------------
	//Se trata de agregar las variables a los arreglos para poder plotear
	double plot_x[limite_puntos];
	double plot_y[limite_puntos];
	double plot_y2[limite_puntos];
	//-------------------------------------------------------------------
	
	
	for(i=0;i<limite_puntos;i++){
		//printf("%.3f\n",salida_m1[i]);
		fprintf(fp, "%.2f \n" , salida_m1[i]);
		//rellenamos los arrays
		plot_x[i] = i*dt;
		plot_y2[i] = ref_x_array[i];
		plot_y[i] = salida_m1[i];
	}
	//printf("%d\n",limite_puntos);
	
	//Se comienza con el ploteo de las variables
	RGBABitmapImageReference *imageRef = CreateRGBABitmapImageReference();
	
	ScatterPlotSeries *series = GetDefaultScatterPlotSeriesSettings();
	series->xs = plot_x;
	series->xsLength = sizeof(plot_x)/sizeof(double);
	series->ys = plot_y;
	series->ysLength = sizeof(plot_y)/sizeof(double);
	//series->linearInterpolation = false;
	//series->pointType = L"dots";
	//series->pointTypeLength = wcslen(series->pointType);
	//series->color = CreateRGBColor(1, 0, 0);
	series->linearInterpolation = true;
	series->lineType = L"solid";
	series->lineTypeLength = wcslen(series->lineType);
	series->lineThickness = 1;
	series->color = CreateRGBColor(1, 0, 0);

	ScatterPlotSeries *series2 = GetDefaultScatterPlotSeriesSettings();
	series2->xs = plot_x;
	series2->xsLength = sizeof(plot_x)/sizeof(double);
	series2->ys = plot_y2;
	series2->ysLength = sizeof(plot_y2)/sizeof(double);
	series2->linearInterpolation = true;
	series2->lineType = L"solid";
	series2->lineTypeLength = wcslen(series->lineType);
	series2->lineThickness = 1;
	series2->color = CreateRGBColor(0, 0, 1);

	ScatterPlotSettings *settings = GetDefaultScatterPlotSettings();
	settings->width = 2400;
	settings->height = 1600;
	settings->autoBoundaries = true;
	settings->autoPadding = true;
	
	settings->title = L"Seguimiento de motor 1 a una referencia - Referencia(Azul) - Movimiento(Rojo)";
	settings->titleLength = wcslen(settings->title);
	settings->xLabel = L"Tiempo [s]";
	settings->xLabelLength = wcslen(settings->xLabel);
	settings->yLabel = L"Posición[m]";
	settings->yLabelLength = wcslen(settings->yLabel);
	ScatterPlotSeries *s [] = {series, series2};
	settings->scatterPlotSeries = s;
	settings->scatterPlotSeriesLength = 2;

	DrawScatterPlotFromSettings(imageRef, settings);

	size_t length;
	double *pngData = ConvertToPNG(&length , imageRef ->image);
	WriteToFile(pngData , length , "plot.png");
	
	printf("Son %d datos",limite_puntos);
	fclose(fp);
}


void cargarReferencias(){
	FILE *fp;
	float linea;
	char* ruta_Archivo = "ref_x.txt";
	
	fp = fopen(ruta_Archivo, "r");
	//Revisamos si se abrio correctamente el archivo
	if (fp == NULL) {
		printf("El archivo no se encuentra en la ruta %s" , ruta_Archivo);
	}
	int i=0;
	for (int contador=0; contador < limite_puntos ; contador ++){
		fscanf(fp, "%f" , &linea);
		//printf("a %f - " , linea);
		ref_x_array[i]=(linea);
		//printf("- %f \n" , ref_x_array[i]);
		i++;
	}
	fclose(fp);
	printf("Se cargo correctamente");
}
int main()
{ 
  id_main=pthread_self() ;
  cargarReferencias();
  
  int N; 
  printf("\n Ingrese un numero para iniciar :");
  scanf("%d",&N);

  pthread_attr_init(&attr);
  pthread_attr_setdetachstate(&attr,PTHREAD_CREATE_JOINABLE);
  
  /* Initialize mutex and condition variable objects */
  pthread_mutex_init(&count_mutex, NULL);
  pthread_cond_init (&count_limite_cv, NULL);
  pthread_mutex_init(&count_mutex_2, NULL);
  pthread_cond_init (&count_limite_cv_2, NULL);
  pthread_mutex_init(&count_mutex_3, NULL);
  pthread_cond_init (&count_limite_cv_3, NULL);

  pthread_create(&hilo_timer,&attr,timer, NULL);
  pthread_create(&hilo_m1_control,&attr,m1_control, NULL);
  pthread_create(&hilo_m1_dinamica,&attr,m1_dinamica, NULL);


  //Finaliza hilo_m1_control con e<0.0001
  pthread_join(hilo_m1_control, NULL);
  printf("LLegó el hilo_m1_control muerto\n");
  pthread_cancel(hilo_timer);
  pthread_cancel(hilo_m1_dinamica);
  printf("Se cancelaron los hilos hilo_timer y hilo_m1_dinamica\n");
  pthread_join(hilo_timer, NULL);
  pthread_join(hilo_m1_dinamica, NULL);
  printf("Se esperó la cancelación de los hilos\n");
  
  pthread_attr_destroy(&attr);
  pthread_mutex_destroy(&count_mutex);
  pthread_cond_destroy(&count_limite_cv);
  
  imprimirSalidaConsola();
  printf("\nIngrese un numero para finalizar :");
  scanf("%d",&N);

  pthread_exit (NULL);

}
