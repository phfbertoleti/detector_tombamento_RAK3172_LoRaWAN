/* Header file: definições de localização GPS */
#ifndef HEADER_DEFS_GPS_LOCALIZACAO
#define HEADER_DEFS_GPS_LOCALIZACAO

/* Definição - baudrate da comunicação com módulo GPS */
#define BAUDRATE_MODULO_GPS     9600

/* Definição - tempo para receber dados da UART do GPS */
#define TEMPO_RECEPCAO_MODULO_GPS     1000  //ms

/* Definição - tempo entre leituras GPS */
#define TEMPO_ENTRE_LEITURAS_GPS      5000 //ms

/* Definições - latitude e longitude de demonstração 
   Utilizar em caso de demonstração em local sem sinal GPS
*/
#define LATITUDE_DEMONSTRACAO     -22.8892271
#define LONGITUDE_DEMONSTRACAO    -47.0798311

/* Protótipos */
void parse_direcoes(int * pt_dir_leste_oeste, int * pt_dir_norte_sul);
void concatena_linha_gps(char byte_recebido);
bool verifica_se_ha_linha_completa_para_parsear(char byte_recebido);
void obtem_lat_long(float * pt_latitude_GPS, float * pt_longitude_GPS, unsigned long * pt_age_GPS, int dir_norte_sul, int dir_leste_oeste);

#endif
