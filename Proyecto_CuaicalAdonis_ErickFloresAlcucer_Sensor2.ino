#include "QuickSortLib.h"
const double pi = 3.1416;
double  duracion;
double  pulso;
double  ruido;
double  distancia;
int     echo = 8;
#define datos 45
int     trig = 9;
double  gra;
double  temp = 0.0;
double  suma1 = 0;
double  graf1, graf2, graf3;
double  sumV;
double  varianza;
double  suma = 0;
double  promedio;
double  valores[datos];
double  valores_salida[datos];
double  valores_salida1[datos];
int     cont = 0;
double NSR1[datos];
double NSR2[datos];
double  nsrr = 0, nsrr1 = 0;
double  valores_gauss[datos];
void MeanFilter(double *valores, double *valores_salida, int tam, int filter);
void MeadianFilter(double *valores, double *valores_salida1, int tam1, int filter1);

void setup() {
  Serial.begin(9600);
  pinMode(trig, OUTPUT);
  pinMode(echo,  INPUT);

}

void loop() {
  if (cont < datos) {
    medicion();

  } else {

    MeanFilter((double *)&valores[0], (double *)&valores_salida[0], datos, 5);
    //FiltroGaussiano();
    MedianFilter((double *)&valores[0], (double *)&valores_salida1[0], datos, 5);
    NSR((double *)valores, (double *)valores_salida, (double *)valores_salida1, (double *)NSR1, (double *)NSR2  );
    dibujar();
  }
}
void medicion(void) {
  pulso = analogRead(A0);
  ruido = analogRead(A1);
  ruido = map(ruido, 0, 1023, 0, 50);
  valores[cont] = pulso + ruido;
  delay(100);
  cont++;


}
void MeanFilter(double * valores, double * valores_salida, int tam, int filter) {
  int i, j;
  for (i = floor(filter / 2); i < tam - floor(filter / 2) - 1; i++) {
    valores_salida[i] = 0;
    for (j = -floor(filter / 2); j < floor(filter / 2) + 1; j++) {
      valores_salida[i] = valores_salida[i] + valores[i + j];
    }
    valores_salida[i] = valores_salida[i] / filter;



  }
}
void FiltroGaussiano() {
  // calculo de la media u
  for (int i = 0; i < datos; i++) {
    suma += valores[i];
  }
  promedio = 0;

  // calculo de la varianza
  for (int j = 0; j < datos; j++) {
    sumV += pow((valores[j] - promedio), 2);
  }
  varianza = 1;

  // Aplicacion de la fórmula
  for (int p = 0; p < datos; p++) {
    valores_gauss[p] = 0;
    valores_gauss[p] = (exp(-(pow(valores[p] - promedio, 2) / (2.0 * pow(varianza, 2)))) / (pow(2.0 * pi * pow(varianza, 2), 0.5)));
    suma1 += valores_gauss[p];

  }
}
void MedianFilter(double * valores, double * valores_salida1, int tam1, int filter1) {
  double valoresMedia[filter1];
  int i, j;
  for (i = floor(filter1 / 2); i < tam1 - floor(filter1 / 2) - 1; i++) {
    for (j = -floor(filter1 / 2); j < floor(filter1 / 2) + 1; j++) {
      valoresMedia[j] = valores[i + j];


    }
    j = 0;


    QuickSortAsc(valoresMedia, 0, filter1);


    valores_salida1[i] = valoresMedia[((filter1 + 1) / 2) - 1];


  }



} void QuickSortAsc(double* valoresMedia, const int left, const int right)
{
  int i = left, j = right;
  double tmp;

  double pivot = valoresMedia[(left + right) / 2];
  while (i <= j)
  {
    while (valoresMedia[i] < pivot) i++;
    while (valoresMedia[j] > pivot) j--;
    if (i <= j)
    {
      tmp = valoresMedia[i];
      valoresMedia[i] = valoresMedia[j];
      valoresMedia[j] = tmp;
      i++;
      j--;
    }
  };

  if (left < j)
    QuickSortAsc(valoresMedia, left, j);
  if (i < right)
    QuickSortAsc(valoresMedia, i, right);
}
void dibujar(void) {
  for (int y = 0; y < datos; y++) {

    Serial.println(String(valores_salida[y]) + String(" ") + String(valores[y]) + String(" ") + String(valores_gauss[y]) + String(" ") + String(valores_salida1[y]));
    graf3 = map(valores_salida1[y], 0, 1300, 0, 255);
    //graf2 = map(1000 * valores_gauss[y] / suma, 0, 1200, 0, 255);
    graf1 = map(valores_salida[y], 0, 1300, 0, 255);
    gra = map(valores[y], 0, 1300, 0, 255);
    double nsr=map(NSR1[y],0,1023, 0, 255);
    double nsr1=map(NSR2[y],0,1023,0,255);  
    analogWrite(4, graf1);
    delay(10);
    analogWrite(3, gra);
    delay(10);
    //analogWrite(5, graf2);
    //delay(10);
    analogWrite(6, graf3) ;
    delay(10);
    analogWrite(10, nsr) ;
    delay(10);
    analogWrite(11, nsr1) ;
    delay(10);
    Serial.println(String(NSR1[y])+String(NSR2[y]));
  }
}
void NSR(double *valores, double *valores_salida, double *valores_salida1, double*NSR1, double *NSR2  ) {
  for (int i = 3; i < datos; i++) {
    NSR1[i] = 10 * log(pow((valores[i]), 2) / pow(valores_salida[i] - valores[i], 2));
    nsrr += NSR1[i];
    NSR2[i] = 10 * log(pow((valores[i]), 2) / pow(valores_salida1[i] - valores[i], 2));
    nsrr1 += NSR2[i];
  }
}
