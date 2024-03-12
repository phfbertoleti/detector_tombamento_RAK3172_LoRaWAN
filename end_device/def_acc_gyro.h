/* Header file: definições da IMU */
#ifndef HEADER_DEFS_ACELEROMETRO_GIROSCOPIO
#define HEADER_DEFS_ACELEROMETRO_GIROSCOPIO

#include "Rak_BMX160.h"

/* Definição - GPIO de interrupção do acelerometro */
#define INT1_PIN WB_IO3 //Acelerometro no slot C


/* Definições - tipos de eventos que gerarão interrupção no acelerômetro */
#define HIGH_G_INT       0x07 // Gera interrupção ao verificar movimento nso eixos x, y e z
#define HIGH_G_THRESHOLD 0x80 // Gera interrupção a 1g

/* Definição - aceleração da gravidade */
#define VALOR_GRAVIDADE    9.80665  // m/s²

/* Definição - ângulo limite de tombamento */
#define ANGULO_LIMITE_TOMBAMENTO         40

/* Definições - acelerômetro */
#define NUM_LEITURAS_CALIBRACAO          200
#define TEMPO_ENTRE_LEITURAS_IMU         20    //ms

/* Definições - thresholds (acelerômetro e giroscópio) */
#define MIN_THRESHOLD_ACC                      0.3  //m/s²
#define MIN_THRESHOLD_GYRO                     2.0  //rad/s

/* Definição - tempo entre leituras do IMU */
#define TEMPO_ENTRE_LEITURAS_IMU    100 //ms

/* Definições - filtro Mahony */
#define sampleFreq  512.0f          //frequencia de amostragem (em Hz)
#define twoKpDef    (256.0f * 1.0f)
#define twoKiDef    (32.0f * 1.0f)

/* Definições - estados */
#define ESTADO_AGUARDA_TOMBAMENTO     0x00
#define ESTADO_EM_TOMBAMENTO          0x01

/* Prototypes */
void init_acc_gyro(void);
void Mahony_AHS_update(float gx, float gy, float gz, float ax, float ay, float az);
void obtem_angulos_de_euler(float * ptr_roll, float * ptr_pitch, float * ptr_yaw);
void le_acc_gyro(float * ptr_ax, float * ptr_ay, float * ptr_az, float * ptr_gx, float * ptr_gy, float * ptr_gz);
void le_acc_gyro_com_offset(float * ptr_ax, float * ptr_ay, float * ptr_az, float * ptr_gx, float * ptr_gy, float * ptr_gz);
bool maquina_estados_tombamento_veiculo(float angulo_tombamento);
bool obtem_status_alerta_tombamento(void);
void seta_status_alerta_tombamento(bool sts_alerta_tomb);

#endif
