#include "def_acc_gyro.h"

/* Variáveis do filtro Mahony AHRS */
volatile float twoKp = twoKpDef; //2 * ganho proporcional (Kp)
volatile float twoKi = twoKiDef; //2 * ganho integral (Ki)
volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
volatile float integralFBx = 0.0f,  integralFBy = 0.0f, integralFBz = 0.0f; //erros da parte integral

/* Variaveis dos offsets de acelerômetro e giroscópio */
float offset_ax = 0;
float offset_ay = 0;
float offset_az = 0;
float offset_gx = 0;
float offset_gy = 0;
float offset_gz = 0;

/* Variável que informa se deve ser enviado alerta de tombamento ou não */
bool envia_alerta_tombamento = false;

/* Variável da máquina de estados de detecção de tombamento */
char estado_tombamento = ESTADO_AGUARDA_TOMBAMENTO;

/* Objeto para ler IMU */
RAK_BMX160 bmx160;

/* Prototypes das funções locais */
float invSqrt(float x);
void calibra_acc_giroscopio(void);
void To_INT1_Interrupt(void);

/* Função: init acelerometro e giroscópio
   Parâmetros: nenhum
   Retorno: nenhum
*/
void init_acc_gyro(void)
{
  pinMode(WB_IO2, OUTPUT);
  digitalWrite(WB_IO2, HIGH);
  delay(300);
  pinMode(LED_BLUE, OUTPUT);
  digitalWrite(LED_BLUE, HIGH);
  time_t timeout = millis();
  Serial.begin(115200);

  while (!Serial)
  {
    if ((millis() - timeout) < 5000)
    {
      delay(100);
    }
    else
    {
      break;
    }
  }

  pinMode(INT1_PIN, INPUT_PULLDOWN);
  attachInterrupt(digitalPinToInterrupt(INT1_PIN), To_INT1_Interrupt, RISING);

  if (bmx160.begin() != true)
  {
    Serial.println("bmx160 init false");
    while (1)
      delay(100);
  }

  bmx160.wakeUp(); // enable the gyroscope and accelerometer sensor

  uint8_t PMU_Status = 0;
  bmx160.readReg(0x03, &PMU_Status, 1);
  Serial.printf("PMU_Status=%x\r\n", PMU_Status);

  bmx160.InterruptConfig(HIGH_G_INT, HIGH_G_THRESHOLD); // Enable HIGH_G_Interrupt ands et the accelerometer threshold
  bmx160.ODR_Config(BMX160_ACCEL_ODR_200HZ, BMX160_GYRO_ODR_200HZ); // set output data rate

  float OrdBuf[2] = {0};
  bmx160.get_ORD_Config(&OrdBuf[0], &OrdBuf[1]);
  Serial.printf("OrdBuf[0]=%f,OrdBuf[1]=%f\r\n", OrdBuf[0], OrdBuf[1]);

  /* Opções de range de giroscópio (em graus por segundo):     
     125, 250, 500, 1000 e 2000
  */
  bmx160.setGyroRange(eGyroRange_500DPS);

  /* Opções de range de acelerômetro (em g):     
     2, 4, 8 e 16
  */  
  bmx160.setAccelRange(eAccelRange_2G);

  /* Calibra acelerometro e giroscópio */
  calibra_acc_giroscopio();

}

/* Função: filtro Mahony
   Parâmetros: - giroscópio nos 3 eixos (gx, gy e gz)
               - acelerômetro nos 3 eixos (ax, ay e az)
   Retorno: nenhum
*/
void Mahony_AHS_update(float gx, float gy, float gz, float ax, float ay, float az)
{
  float recipNorm;
  float halfvx, halfvy, halfvz;
  float halfex, halfey, halfez;
  float qa, qb, qc;

  if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) 
  {
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    halfvx = q1 * q3 - q0 * q2;
    halfvy = q0 * q1 + q2 * q3;
    halfvz = q0 * q0 - 0.5f + q3 * q3;

    halfex = (ay * halfvz - az * halfvy);
    halfey = (az * halfvx - ax * halfvz);
    halfez = (ax * halfvy - ay * halfvx);

    if (twoKi > 0.0f) 
    {
      integralFBx += twoKi * halfex * (1.0f / sampleFreq);
      integralFBy += twoKi * halfey * (1.0f / sampleFreq);
      integralFBz += twoKi * halfez * (1.0f / sampleFreq);
      gx += integralFBx;  // apply integral feedback
      gy += integralFBy;
      gz += integralFBz;
    }
    else {
      integralFBx = 0.0f;
      integralFBy = 0.0f;
      integralFBz = 0.0f;
    }

    gx += twoKp * halfex;
    gy += twoKp * halfey;
    gz += twoKp * halfez;
  }

  gx *= (0.5f * (1.0f / sampleFreq));
  gy *= (0.5f * (1.0f / sampleFreq));
  gz *= (0.5f * (1.0f / sampleFreq));
  qa = q0;
  qb = q1;
  qc = q2;
  q0 += (-qb * gx - qc * gy - q3 * gz);
  q1 += (qa * gx + qc * gz - q3 * gy);
  q2 += (qa * gy - qb * gz + q3 * gx);
  q3 += (qa * gz + qb * gy - qc * gx);

  recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;
}

/* Função: fast inverse square-root
   Parâmetros: valor
   Retorno: fast inverse square-root
*/
float invSqrt(float x)
{
  volatile float halfx = 0.5f * x;
  volatile float y = x;
  volatile long i = *(long*)&y;
  i = 0x5f3759df - (i >> 1);
  y = *(float*)&i;
  y = y * (1.5f - (halfx * y * y));
  y = y * (1.5f - (halfx * y * y));
  return y;
}

/* Função: obtem angulos de euler a partir dos quaternions
   Parâmetros: - Ponteiro para roll
               - Ponteiro para pitch
               - Ponteiro para yaw
   Retorno: nenhum
*/
void obtem_angulos_de_euler(float * ptr_roll, float * ptr_pitch, float * ptr_yaw)
{
  *ptr_roll = atan2f(q0 * q1 + q2 * q3, 0.5f - q1 * q1 - q2 * q2) * (180.0 / M_PI);
  *ptr_pitch = asinf(-2.0f * (q1 * q3 - q0 * q2)) * (180.0 / M_PI);
  *ptr_yaw = atan2f(q1 * q2 + q0 * q3, 0.5f - q2 * q2 - q3 * q3) * (180.0 / M_PI);
}

/* Função: faz calibração do acelerometro e giroscopio
   Parametros: nenhum
   Retorno: nenhum
*/
void calibra_acc_giroscopio(void)
{
  int i;
  float soma_ax = 0;
  float soma_ay = 0;
  float soma_az = 0;
  float soma_gx = 0;
  float soma_gy = 0;
  float soma_gz = 0;
  float valor_ax = 0;
  float valor_ay = 0;
  float valor_az = 0;
  float valor_gx = 0;
  float valor_gy = 0;
  float valor_gz = 0;

  for (i = 0; i < NUM_LEITURAS_CALIBRACAO; i++)
  {
    le_acc_gyro(&valor_ax, &valor_ay, &valor_az,
                &valor_gx, &valor_gy, &valor_gz);

    soma_ax = soma_ax + valor_ax;
    soma_ay = soma_ay + valor_ay;
    soma_az = soma_az + (-1.0 * valor_az);

    soma_gx = soma_gx + valor_gx;
    soma_gy = soma_gy + valor_gy;
    soma_gz = soma_gz + valor_gz;
  }

  offset_ax = soma_ax / NUM_LEITURAS_CALIBRACAO;
  offset_ay = soma_ay / NUM_LEITURAS_CALIBRACAO;
  offset_az = VALOR_GRAVIDADE - (soma_az / NUM_LEITURAS_CALIBRACAO);

  offset_gx = soma_gx / NUM_LEITURAS_CALIBRACAO;
  offset_gy = soma_gy / NUM_LEITURAS_CALIBRACAO;
  offset_gz = soma_gz / NUM_LEITURAS_CALIBRACAO;
}

/* Função: faz a leitura do acelerometro e giroscopio (sem aplicar offsets)
   Parametros: ponteiros para acelerações e componentes do giroscópio (3 eixos)
   Retorno: nenhum
*/
void le_acc_gyro(float * ptr_ax, float * ptr_ay, float * ptr_az,
                 float * ptr_gx, float * ptr_gy, float * ptr_gz)
{
  sBmx160SensorData_t Omagn, Ogyro, Oaccel;

  bmx160.getAllData(&Omagn, &Ogyro, &Oaccel);
  *ptr_ax = Oaccel.x;
  *ptr_ay = Oaccel.y;
  *ptr_az = Oaccel.z;
  *ptr_gx = Ogyro.x;
  *ptr_gy = Ogyro.y;
  *ptr_gz = Ogyro.z;
}

/* Função: faz a leitura do acelerometro e giroscopio (aplicando offsets)
   Parametros: ponteiros para acelerações e componentes do giroscópio (3 eixos)
   Retorno: nenhum
*/
void le_acc_gyro_com_offset(float * ptr_ax, float * ptr_ay, float * ptr_az,
                            float * ptr_gx, float * ptr_gy, float * ptr_gz)
{
  sBmx160SensorData_t Omagn, Ogyro, Oaccel;
  float az_temp;

  bmx160.getAllData(&Omagn, &Ogyro, &Oaccel);
  *ptr_ax = Oaccel.x - offset_ax;
  *ptr_ay = Oaccel.y - offset_ay;
  az_temp = (-1.0) * Oaccel.z;
  *ptr_az = az_temp - offset_az;

  *ptr_gx = Ogyro.x - offset_gx;
  *ptr_gy = Ogyro.y - offset_gy;
  *ptr_gz = Ogyro.z - offset_gz;

  /* Aplica janelamento nas acelerações e componentes do giroscópios */
  if (abs(*ptr_ax) < MIN_THRESHOLD_ACC)
    *ptr_ax = 0;

  if (abs(*ptr_ay) < MIN_THRESHOLD_ACC)
    *ptr_ay = 0;

  if (abs(*ptr_az) < MIN_THRESHOLD_ACC)
    *ptr_az = 0;

  if (abs(*ptr_gx) < MIN_THRESHOLD_GYRO)
    *ptr_gx = 0;

  if (abs(*ptr_gy) < MIN_THRESHOLD_GYRO)
    *ptr_gy = 0;

  if (abs(*ptr_gz) < MIN_THRESHOLD_GYRO)
    *ptr_gz = 0;
}

/* Função: máquina de estados da análise de veículo em tombamento
 *  Parâmetros: último ângulo de tombamento medido
 *  Retorno: true: deve ser feito envio de mensagem LoRaWAN alertando tombamento
 *           false: nenhum envio de mensagem LoRaWAN é necessário
*/
bool maquina_estados_tombamento_veiculo(float angulo_tombamento)
{
    bool status_envia_tombamento = false;
    
    switch (estado_tombamento)
    {
        case ESTADO_AGUARDA_TOMBAMENTO:            
            if (angulo_tombamento >= ANGULO_LIMITE_TOMBAMENTO)
            {
                // tombamento detectado
                Serial.println("*** Tombamento identificado!");
                envia_alerta_tombamento = true;
                estado_tombamento = ESTADO_EM_TOMBAMENTO;
                status_envia_tombamento = true;
            }
            else
            {
                // sem tombamento
                status_envia_tombamento = false;  
            }
            
            break;

        case ESTADO_EM_TOMBAMENTO:
            if (angulo_tombamento >= ANGULO_LIMITE_TOMBAMENTO)
            {
                //continua em tombamento
                status_envia_tombamento = true;
            }
            else
            {
                // tombamento acabou
                Serial.println("*** Fim do tombamento"); 
                status_envia_tombamento = false;
                estado_tombamento = ESTADO_AGUARDA_TOMBAMENTO;            
            }
            break;

        default:
            status_envia_tombamento = false;
            break;                    
    }

    return status_envia_tombamento;
}

/* Função: obtém status do envio de alerta de tombamento
 *  Parâmetros: nenhum
 *  Retorno: status do envio de alerta de tombamento
*/
bool obtem_status_alerta_tombamento(void)
{
    return envia_alerta_tombamento;
}

/* Função: seta status do envio de alerta de tombamento
 *  Parâmetros: status do envio de alerta tombamento
 *  Retorno: nenhum
*/
void seta_status_alerta_tombamento(bool sts_alerta_tomb)
{
    envia_alerta_tombamento = sts_alerta_tomb;
}

void To_INT1_Interrupt(void)
{
  //INT1_Flag = true;
}
