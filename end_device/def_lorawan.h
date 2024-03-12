/* Header file: definições LoRaWAN */
#ifndef HEADER_DEFS_LORAWAN
#define HEADER_DEFS_LORAWAN

#include <Arduino.h>

/* Definição - tempo entre envios */
#define TEMPO_ENTRE_ENVIOS_LORAWAN   (900000) //900 ms = 15 minutos

/* Definições - banda de operação e credenciais LoRaWAN */
/* Não se esqueça de colocar aqui suas próprias credenciais LoRaWAN! */
#define ABP_BAND     (RAK_REGION_AU915)
#define ABP_DEVADDR  {0x00, 0x00, 0x00, 0x00}
#define ABP_APPSKEY  {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}
#define ABP_NWKSKEY  {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}

/* Estrutura de dados para enviar via LoRaWAN */
typedef struct __attribute__((packed))
{
    char angulo_tombamento;
    float lat_gps;   //Little Endian (DCBA)
    float long_gps;  //Little Endian (DCBA)
}TDados_LoRaWAN;

#define TAM_STRUCT_DADOS_LORAWAN   sizeof(TDados_LoRaWAN)

/* Protótipos */
void faz_uplink_lorawan(uint8_t angulo_tombamento, float lat_gps, float long_gps);
void inicializa_lorawan(void);

#endif
