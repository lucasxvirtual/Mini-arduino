/* 
 *   HelloWorld do Servo: Inicia o servo em 0 e move de 0 a 90 e depois de 90 a 0 em loop.
 */
#include <Servo.h>
#include <LiquidCrystal.h>

#include "dados.c"

// Pinos
#define SERVO_PIN 10
#define US_T 9
#define US_E 8
#define BUZZER 7
#define LED 6

// Constantes
#include "constantes.c"

// Variaveis globais do servo
Servo ser;
int pos = 0;
int pos_old = 0;
int inc = 1;

// Variaveis globais do ultrasom
long microsec;
float d_cm;
long us_response_time = US_MAX_RESPONSE_TIME;
int alarm_counter = 0;

// Variaveis globais dos dados
float map_zero[SERVO_MAX];
float map_scan[MEDIAN_MAP];

// Variaveis globais do buzzer
int buz_flag = 0;
long buz_delay = millis();

// Variaveis globais do led
float led_min = 0;
int led_freq = 0;
long led_delay = 0;

// LCD
LiquidCrystal lcd(12, 11, 5, 4, 3, 2);
byte lcd_blocao[8] = { B11111 , B11111 , B11111, B11111, B11111 , B11111, B11111 , B11111 };
byte lcd_vazio[8] = { B00000 , B00000 , B00000, B00000, B00000 , B00000, B00000 , B00000 };
int pos_t;

// Variaveis de estado
#include "var_estado.h"

// Variaveis temporarias
int flag1 = 0;
long t_long_1 = 0;
long temp = 0;


// Funcios prototipos
float media_cota(float leitura[], int leituras); //Dados.c

// Funcoes nativas
void buzz(int buzzer_pin, int freq, int t) {
  if ( millis() > buz_delay ) {
    tone( buzzer_pin , freq , t );
    buz_delay = millis() + BUZZ_MIN_DELAY;
  }
}

// Verifica se o novo n_dist é menor do que algum da fila dos registrados, e insere na posição
int fila_artefato_insere( char n_dist , int n_pos , int* pos , char* dist , int sz ) {
  int i = 0;
  int index;

  for ( i = 0 ; i < sz ; i++ ) 
    if ( n_dist < dist[i] ) 
      break; 

  if( i >= sz ) return 0; // O novo é menor do que todos

  // Encaixar o novo no lugar
  
  index = i;

  for ( i = (sz-1) ; i > index ; i-- ) {
    dist[i] = dist[i-1];
    pos[i] = pos[i-1]; 
  }

  dist[index] = n_dist;
  pos[index] = n_pos;

  if( index == 0 ) return 2;
  else return 1;
}

// Remove os registros de artefatos conhecidos pelo led que tenham sua pos entre pos1 e pos2
int fila_artefato_remove( int pos , int* led_pos , char* led_dist , int sz ) {
  int temp;
  int t_pos = led_pos[0];

  int i = 0;
  int j = 1;

  if( led_dist[0] >= 100 ) return 0;
  if( t_pos != pos ) return 0;

  // Remove os fora de critério
  while( j < sz ) {
    if( led_dist[j] < 100 && ( t_pos != pos ) ) {
      led_dist[i] = led_dist[j];
      led_pos[i] = led_pos[j];
      i++; j++;
    } else {
      j++;
    }
  }

  // Zera os que não existem mais
  for( ; i < sz ; i++ ) {
      led_dist[i] = 100;
      led_pos[i] = 0; 
  }

  return 1;
}

// Cria uma mascara de bits de um palito vertical para o LCD
void lcd_criarPalito( byte* saida, char altura, char pos) {
  // Altura: 0 ~ 8
  // Pos: 1 ~ 5
  
  char i, altura_t;
  for( char i = 7 ; i >= 0 ; i-- ) {
    altura_t = 8 - i;
    if ( altura_t <= altura ) {
      saida[i] = B10000 >> (pos-1);
    } else {
      saida[i] = B00000;
    }      
  }
}

// Cria uma mascara de bits de um caractere vazio para o LCD
void lcd_caracVazio( byte* saida ) {
  int i;
  for ( i = 0 ; i < 8 ; i++ ) {
    saida[i] = B00000;
  }  
}

// Junta 5 mascaras de bits para o LCD
void lcd_juntarPalitos( byte* saida, byte* p1 , byte* p2 , byte* p3 , byte* p4 , byte* p5 ) {
  char i,j;

  for ( i = 0 ; i < 8 ; i++ ) {
    saida[i] = 0 | p1[i] | p2[i] | p3[i] | p4[i] | p5[i] ;
  }
}

// Escreve os dados de posição no CLD, INCOMPLETO!
/*
void lcd_escreveRadar(char* dist, int pos_min, int pos_max ) {
  // lcd.clear();
  int i,j,indice_t;
  int altura_0, altura_1;
  int altura_t,pos_t;
  int pos_segmento = ( pos_max - pos_min ) / 16;

  byte palitos[2][16][8];
  byte carac[8];

  for( i = 0 ; i < 16 ; i++ ) {
    for( j = 0 ; j < pos_segmento ; j++ ) {
      indice_t = (i*pos_segmento) + j;
      altura_t = dist[indice_t] * 16 / 100;
      pos_t = 5 * (j+1) / pos_segmento;

      lcd_caracVazio( palitos[0][i] );
      lcd_caracVazio( palitos[1][i] );
      
      if( altura_t > 8 ) {
        lcd_criarPalito( palitos[0][i], altura_t-8 , pos_t);
        lcd_criarPalito( palitos[1][i], 8 , pos_t);
      } else {
        lcd_criarPalito( palitos[0][i], 0 , pos_t);
        lcd_criarPalito( palitos[1][i], altura_t , pos_t);
      }
      
    }


    lcd_juntarPalitos( carac , palitos[0][0] , palitos[0][1] , palitos[0][2] , palitos[0][3] , palitos[0][4] );     
    lcd.createChar(0, carac); 
    lcd.setCursor(i, 0); 
    lcd.write(byte(0));

    lcd_juntarPalitos( carac , palitos[1][0] , palitos[1][1] , palitos[1][2] , palitos[1][3] , palitos[1][4] );
    lcd.createChar(1, carac); 
    lcd.setCursor(i, 1);
    lcd.write(byte(1));
  }
}
*/
  
void setup() {
  // put your setup code here, to run once:
  ser.attach(SERVO_PIN);

  // LCD Begin
  lcd.begin(16, 2);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Configurando");
  lcd.setCursor(0, 1);
  lcd.print("Radar");
  
 // Ultrasom
 pinMode(US_T,OUTPUT);
 digitalWrite(US_T,LOW);
 delayMicroseconds(10);
 pinMode(US_E,INPUT);
 
 // Buzzer
 pinMode(BUZZER,OUTPUT);
 //tone(BUZZER,440,500);

 // Led
 pinMode(LED,OUTPUT);
 for( int i = 0 ; i < LED_ARTF_LOG ; i++ )
  st_led_artf_dist[i] = 100;
  
 // Serial 
  Serial.begin(9600);
  
  // Printa inicio do codigo funcional
  Serial.println("INICIO");
  
  // Obter mapa do local
  ser.write(SERVO_MIN);  
  delay(1000);
  int mapping_delay = 100;

  digitalWrite( LED , HIGH );
  for( int pos = SERVO_MIN ; pos < SERVO_MAX ; pos++ ) {
    ser.write(SERVO_POS_LOCK);
    delay(mapping_delay); //Delay para ter certeza que o servo moveu para a posição desejada

    //Inicializa registro de posicoes
    st_artf_dist[pos] = 100;

    st_ultrasom_error = 0;
    for( int j = 0 ; j < MEDIAN_MAP ; j++ ) {
      microsec = 0;
      while(microsec == 0) {
        digitalWrite(US_T,HIGH);
        delayMicroseconds(10);
        digitalWrite(US_T,LOW);    
        microsec = pulseIn(US_E,HIGH);
        st_ultrasom_error++;
      }

      Serial.print(pos); Serial.print("["); Serial.print(st_ultrasom_error); Serial.print("]; ");
      
      map_scan[j] =  microsec / 29.4 / 2;   
      
      if( j == 0 ) st_led_max_distance =  map_scan[j];  
      else if ( map_scan[j] > st_led_max_distance ) st_led_max_distance = map_scan[j];
    }

    // Fazer média com corte e registrar valor
    map_zero[pos] = media_cota(map_scan, MEDIAN_MAP); 
  } 

  //Imprimir mapa local
  for( int i = SERVO_MIN ; i < SERVO_MAX ; i++ ) {
    Serial.print(i);
    Serial.print(" : ");
    Serial.println(map_zero[i]);
  }  

  // Resetar o ultrasom
  digitalWrite(US_T,LOW);    
  delayMicroseconds(10); 
  
  // Posiciona Servo
  pos = SERVO_MIN;
  ser.write(SERVO_POS_LOCK);
  delay(2000);

  // Estado inicial do ambiente
  st_ultrasom_read = 0;
  st_ultrasom_error = 0;
  alarm_counter = 0;
  st_serv_mov = 0;

  // Apaga o led indicando que começou a vigilancia
  digitalWrite( LED , LOW );

  // Config LCD
  lcd.clear();
  lcd.createChar(0, lcd_blocao);
  lcd.createChar(1, lcd_vazio);
}

void loop() {  
  
  /* 
   *    ##########################################################################
   *    CONTROLE DE ROTAÇÃO DO SERVO
   *    ##########################################################################
   */ 
  // Movimenta o servo
  if( st_serv_mov == 0 ) { 
    pos_old = pos;
    pos += inc;
    
    if(pos<SERVO_MIN) {
      pos = SERVO_MIN;
      inc = inc * -1;
      //Serial.println("Sobe...");
    } 
    if (pos>SERVO_MAX) {
      pos = SERVO_MAX;
      inc = inc * -1;
      //Serial.println("Desce...");
    }

    // Rotaciona
    //pos=90; // DEBUG
    ser.write(SERVO_POS_LOCK);
    st_serv_mov = 1;
    st_serv_timer1 = millis() + 10; // aguarda um tempo de segurança para certificar que o servo moveu
    
  } 
  if ( st_serv_mov == 1 && millis() > st_serv_timer1 ) {
    st_serv_mov = 2;
  }

  /* 
   *    ##########################################################################
   *    CONTROLE DE LEITURA DO ULTRASSOM
   *    ##########################################################################
   */  

  // Leitura do ultrassom - por média com cota de corte
  if ( st_serv_mov == 2 ) {
    // Serial.print("Angulo: "); Serial.print(pos); Serial.print(" | Leitura: "); Serial.println(st_ultrasom_read); // Debug
    if ( st_ultrasom_read < MEDIAN_SCAN ) {
        digitalWrite(US_T,HIGH);
        delayMicroseconds(10);
        digitalWrite(US_T,LOW);  
        
        //temp = millis(); // DEBUG
        microsec = 0;
        microsec = pulseIn(US_E,HIGH,us_response_time); // CUIDADO! Função lenta
        // microsec = pulseIn(US_E,HIGH); // DEBUG
        //Serial.println( millis()); // DEBUG
  
        if ( microsec > 0 ) {        
          map_scan[st_ultrasom_read] = microsec / 29.4 / 2; 
          st_ultrasom_read++;
        } else {
          // Contabiliza e controla fluxo baseado na quantidade leituras nao concluidas ( erros )
          st_ultrasom_error++;
          us_response_time = st_ultrasom_error * SCAN_ERROR_MULTIPLIER;
          if(st_ultrasom_error >= MAX_SCAN_ERROR) {
            st_serv_mov = 3;
            st_led_detect = 2;
            st_artefato = 2;
          }
        }
    } else {
      d_cm = media_cota(map_scan, MEDIAN_SCAN);
      // Reseta estado e libera o servo para a proxima movimentação 
      st_serv_mov = 3;
      st_artefato = 1;
    }
  }

  if ( st_serv_mov == 3 ) {
    // Escreve o LCD
    //lcd_escreveRadar(st_artf_dist, SERVO_MIN, SERVO_MAX );
    
    // Reseta estado e libera o servo para a proxima movimentação 
    st_ultrasom_read = 0;
    st_ultrasom_error = 0;
    us_response_time = US_MAX_RESPONSE_TIME;
    if( alarm_counter >= US_ALARMCOUNTER_DECREASE ) alarm_counter -= US_ALARMCOUNTER_DECREASE;
    st_serv_mov = 0;
  }

    /* 
   *    ##########################################################################
   *    SISTEMA DE DETECÇÃO DE ARTEFATOS
   *    ##########################################################################
   */  
   pos_t = 16 * (pos-SERVO_MIN) / (SERVO_MAX-SERVO_MIN);
    if ( st_artefato==1 ) {
      st_artefato=0;
      
      // Detecta se um valor abaixo da media mapeada foi encontrado
      if ( d_cm < (map_zero[pos]*0.8) ) { 
        alarm_counter += 10;
      }
  
      // Só acusa a presença de algo se um o valor escaneado aparecer mais de SERVO_ALARMS_MAX vezes
      if( alarm_counter >= US_ALARMCOUNTER_MAX ) {
        alarm_counter = 0;    
        buzz( BUZZER , 440 , 10 );

        // LCD - Exibir obstaculo
        lcd.setCursor( 15-pos_t-1, 0);
        lcd.write(byte(0));        
        lcd.setCursor( 15-pos_t-1, 1);
        lcd.write(byte(0));        
         
        // Avisa ao led
        st_led_detect = 1;

        // Armazena posição
        st_artf_dist[pos] = 100.0 * d_cm / st_led_max_distance;

        // Serial.print("---> "); Serial.print(pos); Serial.print(" : "); Serial.print(d_cm); Serial.print(" / "); Serial.println(map_zero[pos]); // DEBUG
      } else {
        st_led_detect = 2;

        // Limpa posição no registro
        st_artefato=2;
        
        //Serial.print(pos); Serial.print(" : "); Serial.print(d_cm); Serial.print(" / "); Serial.println(map_zero[pos]); // DEBUG
      }


    }
    if ( st_artefato==2 ) {
      // Limpa posição no registro
      st_artf_dist[pos] = 100;
      st_artefato=0; 

      // LCD - Limpar representacao de artefatos
      // lcd.setCursor( 15-pos_t-1, 0);
      // lcd.write(byte(1));        
      // lcd.setCursor( 15-pos_t-1, 1);
      // lcd.write(byte(1)); 
    }

    /* 
   *    ##########################################################################
   *    CONTROLE DO LED
   *    ##########################################################################
   */ 

  // Um novo artefato foi detectado, recalcular distancia minima
   if ( st_led_detect == 1 ) {
    st_led_dist_temp = 100.0 * d_cm / st_led_max_distance;

    flag1 = fila_artefato_insere( st_led_dist_temp , pos , st_led_artf_pos , st_led_artf_dist , LED_ARTF_LOG );
    if ( flag1 > 0 ) st_led_blink = 1;  
    if ( flag1 == 2 ) st_led_detect = 3;
    else st_led_detect = 0;
    
    st_led_freq = 100 + (st_led_dist_temp*10);   
   }

   // Um artefato pode ter sumido, recalcular lista
   if ( st_led_detect == 2 ) {
      if ( fila_artefato_remove( pos , st_led_artf_pos , st_led_artf_dist , LED_ARTF_LOG ) == 1 )
        st_led_detect = 3;
      else
        st_led_detect = 0;        
   }

    // Se o primeiro da lista mudou ( mais proximo do ultrasom ), recalcular frequencia de pisca pisca
   if ( st_led_detect == 3 ) {
    st_led_detect = 0;

    if ( st_led_artf_dist[0] < 100 ) {
    
      st_led_freq = 100 + (st_led_artf_dist[0]*10);
  
    } else {
      st_led_blink = 0;
      digitalWrite( LED , LOW );
    }
   }

   // Pisca o Led
   if ( st_led_blink == 1 && millis() > st_led_delay ) {
     st_led = !st_led;
     digitalWrite( LED , st_led );
     st_led_delay = millis() + st_led_freq;
   }

  
  
}
