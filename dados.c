#include <Math.h>

float media(float leitura[], int leituras) {
  float media = leitura[0];
  for( int i = 1 ; i < leituras ; i++ ) media += leitura[i];
  media = media / leituras;
  return media;
}


float desv_padrao(float leitura[], int leituras, float media) { 
  float dp;

  // Calcula desvio padrão
  dp = pow((leitura[0] - media),2);
  for( int i = 1 ; i < leituras ; i++ ) {
      dp += pow((leitura[i] - media),2);
  }

  return sqrt(dp/leituras);  
}


float media_cota(float leitura[], int leituras) {
  /* Faz a media dos valores da amostra apenas dos valores q estão dentro do desvio padrão ( ignora extremos ) */
  float valor_esperado;
  float media_final = -1;
  float dp, cota_inf, cota_sup;
  int i, considerados;

  // Calcula o valor esperado ( média bruta )
  valor_esperado = media( leitura , leituras );

  // Desvio padrão e cotas superiores e inferiores
  dp = desv_padrao(leitura, leituras, valor_esperado);
  //dp = dp/2; //reduzir a tolerancia
  cota_inf = valor_esperado - dp; if (cota_inf < 3.0) cota_inf = 3.0;
  cota_sup = valor_esperado + dp; 

   // Calcula a media considerando apenas valores dentro do desvio padrão
   i = 0;
   // Captura o primeiro valor dentro dos limites
   while( (leitura[i] > cota_sup || leitura[i] < cota_inf) && i <= leituras ) i++;
   if( i>leituras ) return valor_esperado; // Caso todos os valores estejam fora do desvio padrão, retorna o valor esperado
   media_final = leitura[i];
   considerados = 1;
   // Faz a media dentro dos valores que estao dentro dos limites
   for( ; i < leituras ; i++ ) {
        if ( !(leitura[i] > cota_sup || leitura[i] < cota_inf) ) {
          media_final += leitura[i];
          considerados++;
        }     
   }
  media_final = media_final / considerados;

  // Retorno
  return media_final;
}

