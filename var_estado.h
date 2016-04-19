#include "constantes.c"

// Controle de movimentação do servo
char st_serv_mov = 0; // 0=calculando movimento; 1=se movendo; 2=aguardando leitura do ultrasom
long st_serv_timer1 = 0; // Timer para ter certeza que o servo se moveu

// Controle de coleta de dados do ultrasom
short st_ultrasom_read = 0; // quantidade de leituras feita pelo ultrasom
short st_ultrasom_error = 0;

// Detecção de artefatos
short st_artefato = 0;

// Armazenamento de estado de leitura de artefatos
char st_artf_dist[SERVO_MAX];


// Led
char st_led_detect = 0; //Foi detectado um novo artefato
char st_led_blink = 0; //Foi detectado um novo artefato
char st_led = 0;
long st_led_delay = 0;
int st_led_freq = 0;
float st_led_max_distance;
char st_led_dist_temp;

char st_led_artf_dist[LED_ARTF_LOG];
int st_led_artf_pos[LED_ARTF_LOG];






