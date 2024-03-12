/* Parser: detecção de tombamento de veículos */
const ignore_vars = [];

/**
* This is the main function to parse the payload. Everything else doesn't require your attention.
* @param {String} payload_raw
* @returns {Object} containing key and value to TagoIO
*/
function parsePayload(payload_raw) {
    try {
        // COnverte payload recebido para formato hexadecimal
        const buffer = Buffer.from(payload_raw, 'hex');

        // O payload é composto por:
        // Byte 0 - ângulo de tombamento (em módulo)
        // Bytes 1 a 4 - latitude
        // Bytes 5 a 8 - longitude
        var angulo_tombamento = buffer.readInt8(0);
        var latitude = buffer.readFloatLE(1);
        var longitude = buffer.readFloatLE(5);

        // Obtem status do veículo em função do ângulo de tombamento enviado
        var status_veiculo;
        if (angulo_tombamento >= 40) {
            status_veiculo = "EMERGÊNCIA! Veículo capotado";
        }
        else {
            status_veiculo = "Veículo ok";
        }

        // More information about buffers can be found here: https://nodejs.org/api/buffer.html
        const data = [
            { variable: 'angulo_tombamento', value: angulo_tombamento, unit: '°' },
            { variable: 'status_veiculo', value: status_veiculo, unit: ' ' },
            { variable: "localizacao", value: "localizacao", "location": { "lat": latitude, "lng": longitude } },
        ];
        return data;
    }
    catch (e) {
        console.log(e);
        return [{ variable: 'parse_error', value: e.message }];
    }
}

// Remove/filtra variaveis indesejadas e faz parse to payload para JSON
payload = payload.filter(x => !ignore_vars.includes(x.variable));

const payload_raw = payload.find(x => x.variable === 'payload_raw' || x.variable === 'payload' || x.variable === 'data');
if (payload_raw) {
    const { value, serie, time } = payload_raw;

    if (value) {
        payload = payload.concat(parsePayload(value).map(x => ({ ...x, serie, time: x.time || time })));
    }
}