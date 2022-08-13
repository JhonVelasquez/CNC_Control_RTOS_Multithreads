
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
pthread_t hilo_m2_control;
pthread_t hilo_m2_dinamica;
pthread_attr_t attr;
pthread_attr_t attr_2;
pthread_mutex_t  count_mutex;
pthread_cond_t   count_limite_cv;
pthread_mutex_t  count_mutex_2;
pthread_cond_t   count_limite_cv_2;
pthread_mutex_t  count_mutex_3;
pthread_cond_t   count_limite_cv_3;
pthread_mutex_t  count_mutex_4;
pthread_cond_t   count_limite_cv_4;

int id_main, id_timer, id_m1_control, id_m1_dinamica , id_m2_control, id_m2_dinamica;
#define dt 0.02
#define pi 3.1415


//Flag para usar hilo de control por reatroalimentacion sin (0) o con observador (1)
int tipo_control; 

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

//Datos para el calculo de corriente del motor 
float a_x = -19.6350;
float b_x = 0.5;

//Datos para el calculo de corriente del motor 
float a_y = -1.0996;
float b_y = 0.0333;

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
#define Wfx_d11 -2.4943e-5
#define Wfx_d21 -0.0024

//Simulacion motor y
float Fric_Y=0.25; //0.25
float Ffy = 0; 
float yx_futuro=0; 
float yv_futuro=0;
float Ay_d[2][2]={{1,0.02},{0.02,0.9952}};
#define Ay_d11 1
#define Ay_d21 0
#define Ay_d12 0.02
#define Ay_d22 0.9952
float By_d[2][1]={{1.4440e-6},{1.4428e-4}};
#define By_d11 1.4440e-06
#define By_d21 1.4428e-04
float Wfy_d[2][1]={{-2.5720e-5},{-0.0026}};
#define Wfy_d11 -2.5720e-05
#define Wfy_d21 -0.0026

//Observador de 

//Variables para el observador Y
float yx_actual_obsv = 0;
float yv_actual_obsv = 0;
float yx_futuro_obsv = 0;
float yv_futuro_obsv = 0;
float error_y_obsv = 0;
float Ly_11 = 1.2595;
float Ly_21=  19.7261;

//Variables para el observador X
float xx_actual_obsv = 0;
float xv_actual_obsv = 0;
float xx_futuro_obsv = 0;
float xv_futuro_obsv = 0;
float error_x_obsv = 0;
float Lx_11 = 1.1548;
float Lx_21 = 14.4665;


//Variables para correr el programa
#define limite_puntos 5500

//Variable para guaradar voltaje y corriente de x
float voltaje_x[limite_puntos];
float velocidad_x[limite_puntos];
float corriente_x[limite_puntos];

//Variable para guaradar voltaje y corriente de y
float voltaje_y[limite_puntos];
float velocidad_y[limite_puntos];
float corriente_y[limite_puntos];

//Variables para guardar la respuesta
int indice_salida_m1=0;
float salida_m1[limite_puntos];


int sucede_dinamica_x=0;

//Variables para leer referencia en x 
int indice_referencia_x=0;
float ref_x_array[limite_puntos];

//--------------------------------------------//

//Variables para guardar la respuesta
int indice_salida_m2=0;
float salida_m2[limite_puntos];

int sucede_dinamica_y=0;

//Variables para leer referencia en x 
int indice_referencia_y=0;
float ref_y_array[limite_puntos];


//Variables generales de programa
int fin=0;


void *timer(void *arg)
{
    id_timer=pthread_self() ;
    while(1){
	    sleep(dt);
	    //Para el hilo x 
	    pthread_mutex_lock(&count_mutex_2);
	    	pthread_cond_signal(&count_limite_cv_2);
	    pthread_mutex_unlock(&count_mutex_2); 
	    //para el hilo y
	    pthread_mutex_lock(&count_mutex_3);
	    	pthread_cond_signal(&count_limite_cv_3);
	    pthread_mutex_unlock(&count_mutex_3); 
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
	   
	    while(sucede_dinamica_x==0){
	    };
	    
	    sucede_dinamica_x=0;
	      
	    pthread_mutex_lock(&count_mutex);
	    
	    	if (tipo_control == 1){
				
				xx_actual=xx_futuro;
			    xv_actual=xv_futuro;
			    xx_actual_obsv=xx_futuro_obsv; //Lectura de la salida del observador
			    xv_actual_obsv=xv_futuro_obsv;

			    ref_x=ref_x_array[indice_referencia_x];
			    indice_referencia_x++;

			    error_x_obsv=xx_actual-xx_actual_obsv; //Error observación   
			    errint_x = errint_x + (xx_actual - ref_x)*dt;	 //error de posicion X
			     
			    ux = -1*Kix11*xx_actual_obsv-1*Kix12*xv_actual_obsv-1*Kix13*errint_x; //Calculo de señal de control
			    //Avisa al hilo de la dinamica que puede usar el u

			    // aplicación de señal de control al observador
	    		xx_futuro_obsv=Ax_d11*xx_actual_obsv+Ax_d12*xv_actual_obsv+Bx_d11*ux+Lx_11*error_x_obsv;
	    		xv_futuro_obsv=Ax_d21*xx_actual_obsv+Ax_d22*xv_actual_obsv+Bx_d21*ux+Lx_11*error_x_obsv;     
		
	    	}

	    	else{


		    	//"Sensar posicion y velocidad"
			    xx_actual=xx_futuro;
			    xv_actual=xv_futuro;
			    //Leer referencia
			    //ref_x=1; //Punto de referencia deseada (se actualiza cada dt)
			    ref_x=ref_x_array[indice_referencia_x];
			    indice_referencia_x++;
			    
			    //Ejecutar logica de control
			    errint_x = errint_x + (xx_actual - ref_x)*dt;	 //error de posicion X
			    //printf("err: %f\n",errint_x);
			    ux = -1*Kix11*xx_actual-1*Kix12*xv_actual-1*Kix13*errint_x; //Calculo de señal de control

			}


		    pthread_cond_signal(&count_limite_cv);
		    
	    pthread_mutex_unlock(&count_mutex);
    };
    printf("Fin control m1\n");
    pthread_exit(NULL);
}

void *m1_dinamica(void *arg)
{
    id_m1_dinamica=pthread_self() ; 
	
    printf("Inicia m1_dinamica\n");
    
    if(indice_salida_m1==0){
	    sucede_dinamica_x=1;
    };
    
    while(indice_salida_m1<limite_puntos){

        voltaje_x[indice_salida_m1]=ux;
        velocidad_x[indice_salida_m1]=xv_actual;

        Ffx = Fric_X*xv_actual;  //fricción actual
        xx_futuro=Ax_d11*xx_actual+Ax_d12*xv_actual+Bx_d11*ux+Wfx_d11*Ffx;
        xv_futuro=Ax_d21*xx_actual+Ax_d22*xv_actual+Bx_d21*ux+Wfx_d21*Ffx;
        
        salida_m1[indice_salida_m1]=xx_futuro;
	    indice_salida_m1++;
    	
	    //printf("%.6f\n", xx_futuro);
        //Señal al hilo_m1_control
          
        sucede_dinamica_x=1;
	    
	    if(indice_salida_m1!=limite_puntos){
            pthread_mutex_lock(&count_mutex);
    	    pthread_cond_wait(&count_limite_cv, &count_mutex); //
            pthread_mutex_unlock(&count_mutex);
            }
    };
    printf("Fin de dinamica  m1\n");

}


void *m2_control(void *arg)
{
    id_m2_control=pthread_self() ;
    //Señal al hilo_m1_control
    printf("Inicia m2_control\n");
    while(indice_salida_m2<limite_puntos){
    	    //printf("xc%d\n",indice_salida_m1);
            pthread_mutex_lock(&count_mutex_3);
               	pthread_cond_wait(&count_limite_cv_3, &count_mutex_3);
	    pthread_mutex_unlock(&count_mutex_3);
	   
	    while(sucede_dinamica_y==0){
	    };
	    
	    sucede_dinamica_y=0;
	      
	    pthread_mutex_lock(&count_mutex_4);

	    	if (tipo_control == 1){

			    yx_actual=yx_futuro;
			    yv_actual=yv_futuro;
			    yx_actual_obsv=yx_futuro_obsv; //Lectura de la salida del observador
			    yv_actual_obsv=yv_futuro_obsv;

			    ref_y=ref_y_array[indice_referencia_y];
			    indice_referencia_y++;
			    error_y_obsv=yx_actual-yx_actual_obsv; //Error observación   
			    errint_y = errint_y + (yx_actual - ref_y)*dt;	 //error de posicion X
			    // aplicación de señal de control al observador
			    yx_futuro_obsv=Ay_d11*yx_actual_obsv+Ay_d12*yv_actual_obsv+By_d11*uy+Ly_11*error_y_obsv;
			    yv_futuro_obsv=Ay_d21*yx_actual_obsv+Ay_d22*yv_actual_obsv+By_d21*uy+Ly_21*error_y_obsv;
			     
			    uy = -1*Kiy11*yx_actual_obsv-1*Kiy12*yv_actual_obsv-1*Kiy13*errint_y; //Calculo de señal de control
			    //Avisa al hilo de la dinamica que puede usar el u
			     


	    	}
	    	else{
	    
		    	//"Sensar posicion y velocidad"
			    yx_actual=yx_futuro;
			    yv_actual=yv_futuro;
			    //Leer referencia
			    //ref_x=1; //Punto de referencia deseada (se actualiza cada dt)
			    ref_y=ref_y_array[indice_referencia_y];
			    indice_referencia_y++;
			    
			    //Ejecutar logica de control
			    errint_y = errint_y + (yx_actual - ref_y)*dt;	 //error de posicion y
			    //printf("err: %f\n",errint_x);
			    uy = -1*Kiy11*yx_actual-1*Kiy12*yv_actual-1*Kiy13*errint_y; //Calculo de señal de control

			}
		    pthread_cond_signal(&count_limite_cv_4);
		    
	    pthread_mutex_unlock(&count_mutex_4);
    };
    printf("Fin de control m2\n");
    pthread_exit(NULL);
}


void *m2_dinamica(void *arg)
{
    id_m2_dinamica=pthread_self() ; 
	
    printf("Inicia m2_dinamica\n");
    
    if(indice_salida_m2==0){
	    sucede_dinamica_y=1;
    };
    
    while(indice_salida_m2<limite_puntos){

        voltaje_y[indice_salida_m2]=uy;
        velocidad_y[indice_salida_m2]=yv_actual;

        Ffy = Fric_Y*yv_actual;  //fricción actual
        yx_futuro=Ay_d11*yx_actual+Ay_d12*yv_actual+By_d11*uy+Wfy_d11*Ffy;
        yv_futuro=Ay_d21*yx_actual+Ay_d22*yv_actual+By_d21*uy+Wfy_d21*Ffy;
        salida_m2[indice_salida_m2]=yx_futuro;
	    indice_salida_m2++;	
            //Señal al hilo_m2_control
          
            sucede_dinamica_y=1;
	    
	    if(indice_salida_m2!=limite_puntos){
            pthread_mutex_lock(&count_mutex_4);
    	    pthread_cond_wait(&count_limite_cv_4, &count_mutex_4); //
            pthread_mutex_unlock(&count_mutex_4);
            }
    };
    printf("Fin de dinamica m2\n");

}


void imprimirSalidaConsola_x(){
	int i;
	FILE *fp;
	char* nombre_x;
	if(tipo_control == 1) {
		nombre_x = "Prueba_x_obs_real.txt";
	}
	else{
		nombre_x = "Prueba_x.txt";
	}
	//abrimos el archivo
	fp = fopen (nombre_x , "w");
	//-------------------------------------------------------------------
	//Se trata de agregar las variables a los arreglos para poder plotear
	double plot_x[limite_puntos];
	double plot_y[limite_puntos];
	double plot_y2[limite_puntos];
	//-------------------------------------------------------------------
	
	
	for(i=0;i<limite_puntos;i++){
		//printf("%.3f\n",salida_m1[i]);
		fprintf(fp, "%.6f \n" , salida_m1[i]);
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
	char* plot_x_obs;
	if(tipo_control == 1) {
		plot_x_obs = "plot_x_obs_real.png";
	}
	else{
		plot_x_obs = "plot_x.png";
	}
	size_t length;
	double *pngData = ConvertToPNG(&length , imageRef ->image);
	WriteToFile(pngData , length , plot_x_obs);
	
	printf("Son %d datos",limite_puntos);
	fclose(fp);	

}

void imprimirSalidaConsola_y(){
	int i;
	FILE *fp;
	char* nombre_y;
	if(tipo_control == 1) {
		nombre_y = "Prueba_y_obs_real.txt";
	}
	else{
		nombre_y = "Prueba_y.txt";
	}
	//abrimos el archivo
	fp = fopen (nombre_y , "w");
	//-------------------------------------------------------------------
	//Se trata de agregar las variables a los arreglos para poder plotear
	double plot_x[limite_puntos];
	double plot_y[limite_puntos];
	double plot_y2[limite_puntos];
	//-------------------------------------------------------------------
	
	
	for(i=0;i<limite_puntos;i++){
		//printf("%.3f\n",salida_m1[i]);
		fprintf(fp, "%.6f \n" , salida_m2[i]);
		//rellenamos los arrays
		plot_x[i] = i*dt;
		plot_y2[i] = ref_y_array[i];
		plot_y[i] = salida_m2[i];
	}
	//printf("%d\n",limite_puntos);
	
	//Se comienza con el ploteo de las variables
	RGBABitmapImageReference *imageRef = CreateRGBABitmapImageReference();
	
	ScatterPlotSeries *series = GetDefaultScatterPlotSeriesSettings();
	series->xs = plot_x;
	series->xsLength = sizeof(plot_x)/sizeof(double);
	series->ys = plot_y;
	series->ysLength = sizeof(plot_y)/sizeof(double);
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
	
	settings->title = L"Seguimiento de motor 2 a una referencia - Referencia(Azul) - Movimiento(Rojo)";
	settings->titleLength = wcslen(settings->title);
	settings->xLabel = L"Tiempo [s]";
	settings->xLabelLength = wcslen(settings->xLabel);
	settings->yLabel = L"Posición[m]";
	settings->yLabelLength = wcslen(settings->yLabel);
	ScatterPlotSeries *s [] = {series, series2};
	settings->scatterPlotSeries = s;
	settings->scatterPlotSeriesLength = 2;

	DrawScatterPlotFromSettings(imageRef, settings);
	char* plot_y_obs;
	if(tipo_control == 1) {
		plot_y_obs = "plot_y_obs_real.png";
	}
	else{
		plot_y_obs = "plot_y.png";
	}
	size_t length;
	double *pngData = ConvertToPNG(&length , imageRef ->image);
	WriteToFile(pngData , length , plot_y_obs);
	
	printf("Son %d datos",limite_puntos);
	fclose(fp);	

}


void imprimirSalidaConsola_x_y(){
	int i;
	FILE *fp;
	char* nombre_x_y_obs;
	if(tipo_control == 1) {
		nombre_x_y_obs = "Prueba_x_y_obs_real.txt";
	}
	else{
		nombre_x_y_obs = "Prueba_x_y.txt";
	}
	//abrimos el archivo
	fp = fopen (nombre_x_y_obs , "w");
	//-------------------------------------------------------------------
	//Se trata de agregar las variables a los arreglos para poder plotear
	double plot_x[limite_puntos];
	double plot_y[limite_puntos];
	double plot_x_2[limite_puntos];
	double plot_y_2[limite_puntos];
	//-------------------------------------------------------------------
	
	
	for(i=0;i<limite_puntos;i++){
		fprintf(fp, "%.6f  %.6f\n" , salida_m1[i] , salida_m2[i]);
		//rellenamos los arrays
		plot_x[i] = salida_m1[i];
		plot_y[i] = salida_m2[i];
		plot_x_2[i] = ref_x_array[i];
		plot_y_2[i] = ref_y_array[i];
	}
	//printf("%d\n",limite_puntos);
	
	//Se comienza con el ploteo de las variables
	RGBABitmapImageReference *imageRef = CreateRGBABitmapImageReference();
	
	ScatterPlotSeries *series = GetDefaultScatterPlotSeriesSettings();
	series->xs = plot_x;
	series->xsLength = sizeof(plot_x)/sizeof(double);
	series->ys = plot_y;
	series->ysLength = sizeof(plot_y)/sizeof(double);
	series->linearInterpolation = true;
	series->lineType = L"solid";
	series->lineTypeLength = wcslen(series->lineType);
	series->lineThickness = 1;
	series->color = CreateRGBColor(1, 0, 0);

	ScatterPlotSeries *series2 = GetDefaultScatterPlotSeriesSettings();
	series2->xs = plot_x_2;
	series2->xsLength = sizeof(plot_x_2)/sizeof(double);
	series2->ys = plot_y_2;
	series2->ysLength = sizeof(plot_y_2)/sizeof(double);
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
	
	settings->title = L"Seguimiento de ruta a una referencia - Referencia(Azul) - Movimiento(Rojo)";
	settings->titleLength = wcslen(settings->title);
	settings->xLabel = L"Tiempo [s]";
	settings->xLabelLength = wcslen(settings->xLabel);
	settings->yLabel = L"Posición[m]";
	settings->yLabelLength = wcslen(settings->yLabel);
	ScatterPlotSeries *s [] = {series, series2};
	settings->scatterPlotSeries = s;
	settings->scatterPlotSeriesLength = 2;

	DrawScatterPlotFromSettings(imageRef, settings);
	char* plot_x_y_obs;
	if(tipo_control == 1) {
		plot_x_y_obs = "plot_x_y_obs_real.png";
	}
	else{
		plot_x_y_obs = "plot_x_y.png";
	}
	size_t length;
	double *pngData = ConvertToPNG(&length , imageRef ->image);
	WriteToFile(pngData , length , plot_x_y_obs);
	
	printf("Son %d datos",limite_puntos);
	fclose(fp);	

}

void imprimirPotencia_x_y(){
	int i;
	FILE *fp;
	char* nombre_x_y_obs;
	if(tipo_control == 1) {
		nombre_x_y_obs = "Potencia_x_y_obs_real.txt";
	}
	else{
		nombre_x_y_obs = "Potencia_x_y.txt";
	}
	//abrimos el archivo
	fp = fopen (nombre_x_y_obs , "w");
	//-------------------------------------------------------------------
	//Se trata de agregar las variables a los arreglos para poder plotear
	double plot_x[limite_puntos];
	double plot_y[limite_puntos];
	double plot_x_2[limite_puntos];
	double plot_y_2[limite_puntos];
	//-------------------------------------------------------------------
	
	
	for(i=0;i<limite_puntos;i++){
		fprintf(fp, "%.6f  %.6f\n" , (voltaje_x[i]*b_x + a_x*velocidad_x[i] )*voltaje_x[i], (voltaje_y[i]*b_y + a_y*velocidad_y[i])*voltaje_y[i]);
		//rellenamos los arrays
		plot_x[i] = i*dt;
		plot_y[i] = (voltaje_x[i]*b_x + a_x*velocidad_x[i])*voltaje_x[i];
		plot_x_2[i] = i*dt;
		plot_y_2[i] = (voltaje_y[i]*b_y + a_y*velocidad_y[i])*voltaje_y[i];
	}
	//printf("%d\n",limite_puntos);
	
	//Se comienza con el ploteo de las variables
	RGBABitmapImageReference *imageRef = CreateRGBABitmapImageReference();
	
	ScatterPlotSeries *series = GetDefaultScatterPlotSeriesSettings();
	series->xs = plot_x;
	series->xsLength = sizeof(plot_x)/sizeof(double);
	series->ys = plot_y;
	series->ysLength = sizeof(plot_y)/sizeof(double);
	series->linearInterpolation = true;
	series->lineType = L"solid";
	series->lineTypeLength = wcslen(series->lineType);
	series->lineThickness = 1;
	series->color = CreateRGBColor(1, 0, 0);

	ScatterPlotSeries *series2 = GetDefaultScatterPlotSeriesSettings();
	series2->xs = plot_x_2;
	series2->xsLength = sizeof(plot_x_2)/sizeof(double);
	series2->ys = plot_y_2;
	series2->ysLength = sizeof(plot_y_2)/sizeof(double);
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
	
	settings->title = L"Potencia consumida - Potencia_X(Azul) - Potencia_Y(Rojo)";
	settings->titleLength = wcslen(settings->title);
	settings->xLabel = L"Tiempo [s]";
	settings->xLabelLength = wcslen(settings->xLabel);
	settings->yLabel = L"Posición[m]";
	settings->yLabelLength = wcslen(settings->yLabel);
	ScatterPlotSeries *s [] = {series, series2};
	settings->scatterPlotSeries = s;
	settings->scatterPlotSeriesLength = 2;

	DrawScatterPlotFromSettings(imageRef, settings);
	char* plot_x_y_obs;
	if(tipo_control == 1) {
		plot_x_y_obs = "plot_pot_x_y_obs_real.png";
	}
	else{
		plot_x_y_obs = "plot_pot_x_y.png";
	}
	size_t length;
	double *pngData = ConvertToPNG(&length , imageRef ->image);
	WriteToFile(pngData , length , plot_x_y_obs);
	
	printf("Son %d datos",limite_puntos);
	fclose(fp);	

}

void cargarReferencias(){
	
	float linea;
	char* ruta_Archivo_x = "ref_x.txt";
	char* ruta_Archivo_y = "ref_y.txt";
	
	//Cargamos X
	FILE *fp;
	fp = fopen(ruta_Archivo_x, "r");
	//Revisamos si se abrio correctamente el archivo
	if (fp == NULL) {
		printf("El archivo no se encuentra en la ruta %s" , ruta_Archivo_x);
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
	printf("Se cargo correctamente x");

	//Cargamos y
	FILE *fp_2;
	fp_2 = fopen(ruta_Archivo_y, "r");
	//Revisamos si se abrio correctamente el archivo
	if (fp_2 == NULL) {
		printf("El archivo no se encuentra en la ruta %s" , ruta_Archivo_y);
	}
	int p=0;
	for (int contador=0; contador < limite_puntos ; contador ++){
		fscanf(fp_2, "%f" , &linea);
		//printf("a %f - " , linea);
		ref_y_array[p]=(linea);
		//printf("- %f \n" , ref_x_array[i]);
		p++;
	}
	fclose(fp_2);
	printf("Se cargo correctamente y");
}



int main()
{ 
  id_main=pthread_self() ;
  cargarReferencias();
  

  printf("\n Ingrese tipo de control para iniciar \n (1) con observador \n (0) sin observador  :");
  scanf("%d",&tipo_control);

  pthread_attr_init(&attr);
  pthread_attr_init(&attr_2);
  pthread_attr_setdetachstate(&attr,PTHREAD_CREATE_JOINABLE);
  
  /* Initialize mutex and condition variable objects */
  //Mutex para el motor x
  pthread_mutex_init(&count_mutex, NULL);
  pthread_cond_init (&count_limite_cv, NULL);
  pthread_mutex_init(&count_mutex_2, NULL);
  pthread_cond_init (&count_limite_cv_2, NULL);
  //Mutex para el motor y
  pthread_mutex_init(&count_mutex_3, NULL);
  pthread_cond_init (&count_limite_cv_3, NULL);
  pthread_mutex_init(&count_mutex_4, NULL);
  pthread_cond_init (&count_limite_cv_4, NULL);

  pthread_create(&hilo_timer,&attr,timer, NULL);
  //Declaramos los hilos correspondientes al motor m1
  pthread_create(&hilo_m1_control,&attr,m1_control, NULL);
  pthread_create(&hilo_m1_dinamica,&attr,m1_dinamica, NULL);
  //Declaramos los hilos correspondientes al motor m1 y m2 
  pthread_create(&hilo_m2_control,&attr_2,m2_control, NULL);
  pthread_create(&hilo_m2_dinamica,&attr_2,m2_dinamica, NULL);

  //Finaliza hilo_m1_control con e<0.0001
  pthread_join(hilo_m1_control, NULL);
  printf("LLegó el hilo_m1_control muerto\n");
  pthread_join(hilo_m2_control, NULL);
  printf("LLegó el hilo_m2_control muerto\n");
  pthread_cancel(hilo_timer);
  pthread_cancel(hilo_m1_dinamica);
  pthread_cancel(hilo_m2_dinamica);
  printf("Se cancelaron los hilos hilo_timer e hilo_m1_dinamica e hilo_m2_dinamica\n");
  pthread_join(hilo_timer, NULL);
  pthread_join(hilo_m1_dinamica, NULL);
  pthread_join(hilo_m2_dinamica, NULL);
  printf("Se esperó la cancelación de los hilos\n");
  pthread_attr_destroy(&attr);
  pthread_attr_destroy(&attr_2);
  //Destruimos los mutex
  pthread_mutex_destroy(&count_mutex);
  pthread_mutex_destroy(&count_mutex_2);
  pthread_mutex_destroy(&count_mutex_3);
  pthread_mutex_destroy(&count_mutex_4);
  pthread_cond_destroy(&count_limite_cv);


  //Salida de verificacion
  imprimirSalidaConsola_x();
  imprimirSalidaConsola_y();
  imprimirSalidaConsola_x_y();
  imprimirPotencia_x_y();
  printf("\nIngrese un numero para finalizar :");
  scanf("%d",&tipo_control);

  pthread_exit (NULL);

}
