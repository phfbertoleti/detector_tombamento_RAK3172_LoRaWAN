#include "def_gps_localizacao.h"
#include <TinyGPS.h>

/* Variáveis usadas para leitura do módulo GPS */
TinyGPS gps;
String gps_tmp_data = "";
float latitude;
float longitude;

/* Função: faz o parse de direções, determinando se as coordenadas GPS são ao norte ou sul e leste ou oeste
 * Parâmetros: ponteiro para as direções
 * Retorno: nenhum
*/
void parse_direcoes(int * pt_dir_leste_oeste, int * pt_dir_norte_sul)
{
    if (gps_tmp_data.indexOf(",E,") != -1)
    {
        *pt_dir_leste_oeste = 0;
    }
    else
    {
        *pt_dir_leste_oeste = 1;
    }

    if (gps_tmp_data.indexOf(",S,") != -1)
    {
        *pt_dir_norte_sul = 0;
    }
    else
    {
        *pt_dir_norte_sul = 1;
    }

    gps_tmp_data = "";
}

/* Função: concatena byte recebido da UART do módulo GPS na linha de informações do GPS
 * Parâmetros: byte recebido
 * Retorno: nenhum
*/
void concatena_linha_gps(char byte_recebido)
{
    gps_tmp_data = gps_tmp_data + byte_recebido;
}

/* Função: verifica se já existe linha completa para parsear
 * Parâmetros: byte recebido
 * Retorno: nenhum
*/
bool verifica_se_ha_linha_completa_para_parsear(char byte_recebido)
{
    return gps.encode(byte_recebido);
}


/* Função: obtem latitude e longitude (formato float)
 * Parâmetros: ponteiros para latitude, longiture e age do GPS
 * Retorno: nenhum
*/
void obtem_lat_long(float * pt_latitude_GPS, float * pt_longitude_GPS, unsigned long * pt_age_GPS, int dir_norte_sul, int dir_leste_oeste)
{
    gps.f_get_position(pt_latitude_GPS, pt_longitude_GPS, pt_age_GPS);

    if (dir_norte_sul == 1)
    {
        latitude = (-1.0)*latitude;
    }

    if (dir_leste_oeste == 0)
    {
        longitude = (-1.0)*longitude;
    }
}
