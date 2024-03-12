/* Projeto: end-device da solução de detecção de tombamento de veículos
 *          Projeto apresentado na live: "Conectividade LoRaWAN da teoria à 
 *          prática com RAK3172 e TagoIO"
 * Descrição: projeto capaz de medir o ângulo de tombamento de um veículo e,
 *            se este ângulo exceder o limite estabelecido, considera que o 
 *            veículo tombou / capotou e manda uma mensagem de alerta (com  
 *            coordenadas GPS) via LoRaWAN para a TagoIO.
 *            Ainda, caso não haja tombamento, é periódicamente enviado o
 *            ângulo de tombamento calculado, para fins de monitoramento.
 * Data: Março/2024           
 */
#include "def_lorawan.h"
#include "def_acc_gyro.h"
#include "def_gps_localizacao.h"

/* Comente a linha abaixo para desabilitar as mensagens de debug relativas aos Ângulos de Euler */
#define MOSTRA_DEBUG_ANGULOS_EULER

/* Comente a linha abaixo para desabilitar as mensagens de debug relativas ao GPS */
#define MOSTRA_DEBUG_GPS

/* Descomente a linha abaixo para habilitar latitude e longitude fixas para demonstração */
#define HABILITA_LAT_LONG_FIXA_PARA_DEMONSTRACAO

/* Variaveis globais */
unsigned long timestamp_envio_lorawan = 0;
unsigned long timestamp_envio_lorawan_alerta = 0;
unsigned long timestamp_leitura_GPS = 0;
unsigned long age_gps;
float lat_lida;
float long_lida;
  
#ifdef MOSTRA_DEBUG_ANGULOS_EULER
unsigned long timestamp_debug_angulos_euler = 0;
#endif

/* Prototipos */
unsigned long diferenca_tempo(unsigned long t_ref);
bool INT1_Flag = false;

/* Função: calcula a diferença de tempo entre uma referência e o instante atual
 * Parâmetros: referência de tempo
 * Retorno: diferença calculada
*/
unsigned long diferenca_tempo(unsigned long t_ref)
{
    return (millis() - t_ref);
}

void setup()
{
  /* Inicializa acelerometro */
  init_acc_gyro();
  
  /* Inicializacao do GPS e limpeza do buffer de recepção da uart 
     de comunicação com módulo GPS */
  pinMode(WB_IO1, OUTPUT);
  digitalWrite(WB_IO1, 0);
  delay(1000);
  digitalWrite(WB_IO1, 1);
  delay(1000);
  Serial1.begin(BAUDRATE_MODULO_GPS);
  while(!Serial1);
    
  /* Incializa status do envio de alerta de tombamento */
  seta_status_alerta_tombamento(false);

  #ifdef MOSTRA_DEBUG_ANGULOS_EULER
      timestamp_debug_angulos_euler = millis();
  #endif    

  /* Inicializações do LoRaWAN */
  inicializa_lorawan();

  /* Inicialização das temporizações */
  timestamp_envio_lorawan = millis();  
  timestamp_envio_lorawan_alerta = millis();  
  timestamp_leitura_GPS = millis();
}

void loop()
{
  float ax, ay, az;
  float gx, gy, gz;
  float roll_atual;
  float pitch_atual;
  float yaw_atual;
  float pitch_roll_aux;
  bool ha_dado_validos_gps;
  char byte_gps;
  unsigned long timestamp_recepcao_serial_GPS;
  int dir_norte_sul;
  int dir_leste_oeste;
      
  /* Leitura de giroscópio e acelerômetro, já considerando os offsets */
  le_acc_gyro_com_offset(&ax, &ay, &az, &gx, &gy, &gz);
  
  /* Calcula ângulos de Euler*/
  Mahony_AHS_update(gx, gy, gz, ax, ay, az);
  obtem_angulos_de_euler((float *)&roll_atual, (float *)&pitch_atual, (float *)&yaw_atual);

  /* Inverte roll com pitch (para adequar orientação espacial da placa) */
  pitch_roll_aux = pitch_atual;
  pitch_atual = roll_atual;
  roll_atual = pitch_roll_aux;

  #ifdef MOSTRA_DEBUG_ANGULOS_EULER
  if ( (millis() - timestamp_debug_angulos_euler) >= 1000)
  {
      Serial.printf("Pitch = %.2f grau(s) / Roll = %.2f grau(s) / Yaw = %.2f grau(s)\n", pitch_atual, roll_atual, yaw_atual); 
      timestamp_debug_angulos_euler = millis();
  }
  #endif

  /* Faz o módulo do ângulo roll (ângulo de tombamento) */
  roll_atual = abs(roll_atual);
  delay(TEMPO_ENTRE_LEITURAS_IMU);

  /* Verifica se é o momento de fazer o envio LoRaWAN */
  if (maquina_estados_tombamento_veiculo(roll_atual) == true)
  {
      /* Verifica se o envio tem como motivo a periodicidade de um alerta OU a detecção de um tombamento */
      if ( (diferenca_tempo(timestamp_envio_lorawan_alerta) >= TEMPO_ENTRE_ENVIOS_LORAWAN) || (obtem_status_alerta_tombamento() == true) )
      {
          if (obtem_status_alerta_tombamento() == true)
          {
              Serial.println("*** Envio de mensagem LoRaWAN de alerta de tombamento!");
          }          

          Serial.printf("Coordenadas GPS do envio de mensagem LoRaWAN de alerta: %.5f, %.5f", lat_lida, long_lida);
          faz_uplink_lorawan((uint8_t)roll_atual, lat_lida, long_lida); 
          timestamp_envio_lorawan_alerta = millis();
          seta_status_alerta_tombamento(false);
      }
  }
  else
  {
      /* Verifica se é o momento de fazer um envio periódico (sem alerta) */
      if (diferenca_tempo(timestamp_envio_lorawan) >= TEMPO_ENTRE_ENVIOS_LORAWAN)
      {
          Serial.println("*** Envio de mensagem LoRaWAN periodica");
          Serial.printf("Coordenadas GPS do envio de mensagem LoRaWAN periodica: %.5f, %.5f", lat_lida, long_lida);
          faz_uplink_lorawan((uint8_t)roll_atual, lat_lida, long_lida); 
          timestamp_envio_lorawan = millis();
      }
  }

  /* Verifica se é o momento de fazer a leitura do módulo GPS */
  if (diferenca_tempo(timestamp_leitura_GPS) >= TEMPO_ENTRE_LEITURAS_GPS)
  {
      #ifdef HABILITA_LAT_LONG_FIXA_PARA_DEMONSTRACAO
      lat_lida = LATITUDE_DEMONSTRACAO;
      long_lida = LONGITUDE_DEMONSTRACAO;
      Serial.printf("[AVISO] Para demonstracao, latitude e longitude fixas em: %.5f, %.5f\n\r", lat_lida, long_lida);
      
      #else 
      
      Serial.println("Lendo GPS...");
      ha_dado_validos_gps = false;
      timestamp_recepcao_serial_GPS = millis();

      #ifdef MOSTRA_DEBUG_GPS
          Serial.println("Dump dos dados lidos do modulo GPS: ");
      #endif

      while (diferenca_tempo(timestamp_recepcao_serial_GPS) <= TEMPO_RECEPCAO_MODULO_GPS)
      {
          while (Serial1.available())
          {
              byte_gps = Serial1.read();

              #ifdef MOSTRA_DEBUG_GPS
              Serial.print(byte_gps);
              #endif
              
              concatena_linha_gps(byte_gps);

              /* Verifica se há dados válidos vindos do módulo GPS para fazer parse */
              if (verifica_se_ha_linha_completa_para_parsear(byte_gps))
              {
                  ha_dado_validos_gps = true;
              }
          }
      }
      
      parse_direcoes(&dir_leste_oeste, &dir_norte_sul);
      
      /* Se há nova linha para parsear, aplica o parse */
      if (ha_dado_validos_gps == true)
      {
          obtem_lat_long(&lat_lida, &long_lida, &age_gps, dir_norte_sul, dir_leste_oeste);

          #ifdef MOSTRA_DEBUG_GPS
              Serial.printf("** Latitude: %.5f\n", lat_lida);
              Serial.printf("** Longitude: %.5f\n", long_lida);
          #endif
      }
      else
      {
          Serial.println("Sem dados válidos do GPS.");          
          lat_lida = 0.0;
          long_lida = 0.0;          
      }
      #endif
      
      timestamp_leitura_GPS = millis();
  }  
}
