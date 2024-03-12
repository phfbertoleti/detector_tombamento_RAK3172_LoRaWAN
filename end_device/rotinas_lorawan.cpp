#include "def_lorawan.h"

/* Protótipos de funções locais */
void recvCallback(SERVICE_LORA_RECEIVE_T * data);
void sendCallback(int32_t status);

/* Função: faz inicialização do LoRaWAN
   Parâmetros: nenhum
   Retorno: nenhum
*/
void inicializa_lorawan(void)
{
    uint8_t node_dev_addr[4] = ABP_DEVADDR;
    uint8_t node_app_skey[16] = ABP_APPSKEY;
    uint8_t node_nwk_skey[16] = ABP_NWKSKEY;
    
    if(api.lorawan.nwm.get() != 1)
    {
        Serial.printf("Set Node device work mode %s\r\n",
        api.lorawan.nwm.set(1) ? "Success" : "Fail");
        api.system.reboot();
    }

    api.lorawan.njm.set(RAK_LORA_ABP);
    api.lorawan.txp.set(0);

    /* Para mais informações sobre os channel masks disponíveis, acesse:
     * https://news.rakwireless.com/get-started-with-rui3-api/
     */
    uint16_t channel_mask = 0x0001;
    api.lorawan.mask.set(&channel_mask);
    
    api.lorawan.daddr.set(node_dev_addr, 4);    
    api.lorawan.appskey.set(node_app_skey, 16);        
    api.lorawan.nwkskey.set(node_nwk_skey, 16);
    api.lorawan.band.set(ABP_BAND);
    
    api.lorawan.deviceClass.set(RAK_LORA_CLASS_A);
    api.lorawan.adr.set(true);
    api.lorawan.dr.set(2);        
    api.lorawan.rety.set(0);    
    api.lorawan.cfm.set(0);
    api.lorawan.dcs.set(0);

    /* Registra callbacks de envio e recepção */
    api.lorawan.registerRecvCallback(recvCallback);
    api.lorawan.registerSendCallback(sendCallback);
}

/* Função: Callback de recepção (LoRaWAN)
   Parâmetros: dados do evento LoRaWAN
   Retorno: nenhum
*/
void recvCallback(SERVICE_LORA_RECEIVE_T * data)
{
    if (data->BufferSize > 0) {
        Serial.printf("Mensagem de downlink foi recebida.");
        for (int i = 0; i < data->BufferSize; i++) {
            Serial.printf("%x", data->Buffer[i]);
        }
        Serial.print("\r\n");
    }
}

/* Função: Callback de envio (LoRaWAN)
   Parâmetros: status do evento LoRaWAN
   Retorno: nenhum
*/
void sendCallback(int32_t status)
{
    if (status == RAK_LORAMAC_STATUS_OK) {
        Serial.printf("Mensagem LoRaWAN enviada com sucesso\r\n");
    } else {
        Serial.printf("Erro ao enviar mensagem LoRaWAN\r\n");
    }
}

/* Função: faz uplink (envio) LoRaWAN
 *  Parâmetros: - angulo de tombamento
 *              - latitude
 *              - longitude
 *  Retorno: nenhum
*/
void faz_uplink_lorawan(uint8_t angulo_tombamento, float lat_gps, float long_gps)
{
    uint8_t data_len = 0;
    uint8_t payload_lorawan[TAM_STRUCT_DADOS_LORAWAN];
    char * pt_dado;
    TDados_LoRaWAN dados_lorawan;

    Serial.println("Coordenadas GPS do envio:");
    Serial.print(lat_gps);
    Serial.print(" ,");
    Serial.print(long_gps);

    /* Formata payload */
    memset(payload_lorawan, 0x00, sizeof(payload_lorawan));
    memset((char *)&dados_lorawan, 0x00, TAM_STRUCT_DADOS_LORAWAN);

    dados_lorawan.angulo_tombamento = angulo_tombamento;

    pt_dado = (char *)&lat_gps;
    memcpy((char *)&dados_lorawan.lat_gps, pt_dado, sizeof(float));
    
    pt_dado = (char *)&long_gps;
    memcpy((char *)&dados_lorawan.long_gps, pt_dado, sizeof(float));
    
    pt_dado = (char *)&dados_lorawan;
    memcpy(payload_lorawan, pt_dado, TAM_STRUCT_DADOS_LORAWAN);
    data_len = TAM_STRUCT_DADOS_LORAWAN;
    
    /** Send the data package */
    if (api.lorawan.send(data_len, payload_lorawan, 2, true, 1)) 
    {
        Serial.printf("Envio LoRaWAN requisitado...\r\n");
    } 
    else 
    {
        Serial.printf("Falha no envio LoRaWAN!\r\n");
    }
}
