#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "wifi_app.h"

#include "lora.h"
#include "nvs_utils.h"

static const char *TAG = "MAIN";

void app_main(void){
	esp_err_t ret = nvs_init_storage();
    ESP_ERROR_CHECK(ret);
    ESP_LOGI(TAG, "System start");

    if (lora_init() != ESP_OK){
        ESP_LOGE(TAG, "LoRa init failed");
        return;
    }

    if (lora_configure() != ESP_OK){
        ESP_LOGE(TAG, "LoRa config failed");
        return;
    }

    if (lora_rx_start() != ESP_OK){
        ESP_LOGE(TAG, "LoRa RX start failed");
        return;
    }
    ESP_ERROR_CHECK(esp_netif_init());
    wifi_app_start();

    ESP_LOGI(TAG, "LoRa running");
}


/*
#include <math.h>
#include "ra01s.h"
#include "esp_log.h"
#include "types.h"
#include "stdio.h"
#include "types.h"
#include "string.h"

static const char *TAG = "FANET_TX";

 =========================
 * SIM OBJECTS
 * ========================= 

typedef struct {
    uint16_t id;
    trck_acft_type type;

    float lat;
    float lon;

    float alt;
    float heading;

    float speed;
    float climb;

    float step;
} sim_object_t;

 2 obiekty 
static sim_object_t obj1 = {
    .id = 0x1234,
    .type = acft_Paraglider,
    .lat = 52.2297,
    .lon = 21.0122,
    .alt = 1200,
    .heading = 90,
    .speed = 35,
    .step = 0.0001
};

static sim_object_t obj2 = {
    .id = 0x2222,
    .type = acft_Powered_Aircraft,
    .lat = 52.2305,
    .lon = 21.0100,
    .alt = 2500,
    .heading = 270,
    .speed = 180,
    .step = 0.0002
};

 =========================
 * ENCODE PACKET
 * ========================= 

static void encode_tracking(fanet_packet_t1 *pkt, sim_object_t *o)
{
    memset(pkt, 0, sizeof(fanet_packet_t1));

    pkt->header.type = FANET_PCK_TYPE_TRACKING;
    pkt->header.vendor = 0xBD;
    pkt->header.address = o->id;

    pkt->header.forward = 0;
    pkt->header.ext_header = 0;

     LAT/LON 
    int32_t lat = (int32_t)roundf(o->lat * 93206.0f);
    int32_t lon = (int32_t)roundf(o->lon * 46603.0f);

    pkt->latitude_raw = lat;
    pkt->longitude_raw = lon;

     ALT 
    pkt->altitude = (uint16_t)fminf(fmaxf(o->alt, 0), 4095);
    pkt->altitude_scale = 0;

    pkt->aircraft_type = o->type & 0x7;
    pkt->track_online = 1;

     SPEED (0.5 km/h steps) 
    int speed = (int)roundf(o->speed / 0.5f);
    if (speed < 0) speed = 0;
    if (speed > 127) speed = 127;

    pkt->speed_value = speed;
    pkt->speed_scale = 0;

     CLIMB (0.1 m/s signed) 
    int climb = (int)roundf(o->climb / 0.1f);
    if (climb < -64) climb = -64;
    if (climb > 63) climb = 63;

    pkt->climb_value = (uint8_t)(climb & 0x7F);
    pkt->climb_scale = 0;

     HEADING 
    pkt->heading = (uint8_t)roundf(o->heading * 256.0f / 360.0f);
}

 =========================
 * UPDATE SIMULATION
 * ========================= 

static void update_object(sim_object_t *o)
{
     ruch po okręgu 
    o->heading += 2.0f;
    if (o->heading > 360) o->heading -= 360;

    float rad = o->heading * (M_PI / 180.0f);

    o->lat += cosf(rad) * o->step;
    o->lon += sinf(rad) * o->step;

     wysokość 
    o->alt += sinf(rad) * 2.0f;

     climb 
    o->climb = sinf(rad) * 3.0f;
}

 =========================
 * TASK
 * ========================= 

void task_tx(void *pvParameters)
{
    ESP_LOGI(TAG, "FANET TRACKING EMULATOR START");

    uint8_t buf[255];

    fanet_packet_t1 pkt;

    while (1)
    {
         OBJECT 1 
        update_object(&obj1);
        encode_tracking(&pkt, &obj1);

        memcpy(buf, &pkt, sizeof(pkt));
        LoRaSend(buf, sizeof(pkt), SX126x_TXMODE_SYNC);

        ESP_LOGI(TAG,
            "OBJ1 lat=%.5f lon=%.5f alt=%.1f",
            obj1.lat, obj1.lon, obj1.alt
        );

        vTaskDelay(pdMS_TO_TICKS(500));

         OBJECT 2 
        update_object(&obj2);
        encode_tracking(&pkt, &obj2);

        memcpy(buf, &pkt, sizeof(pkt));
        LoRaSend(buf, sizeof(pkt), SX126x_TXMODE_SYNC);

        ESP_LOGI(TAG,
            "OBJ2 lat=%.5f lon=%.5f alt=%.1f",
            obj2.lat, obj2.lon, obj2.alt
        );

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}




void app_main()
{
	// Initialize LoRa
	LoRaInit();
	int8_t txPowerInDbm = 22;

	uint32_t frequencyInHz = 0;

	frequencyInHz = 868000000;
	ESP_LOGI(TAG, "Frequency is 433MHz");


	ESP_LOGW(TAG, "Enable TCXO");
	float tcxoVoltage = 3.3; // use TCXO
	bool useRegulatorLDO = true; // use DCDC + LDO

	//LoRaDebugPrint(true);
	if (LoRaBegin(frequencyInHz, txPowerInDbm, tcxoVoltage, useRegulatorLDO) != 0) {
		ESP_LOGE(TAG, "Does not recognize the module");
		while(1) {
			vTaskDelay(1);
		}
	}
	
	uint8_t spreadingFactor = 7;
	uint8_t bandwidth = 4;
	uint8_t codingRate = 1;
	uint16_t preambleLength = 8;
	uint8_t payloadLen = 0;
	bool crcOn = true;
	bool invertIrq = false;

	LoRaConfig(spreadingFactor, bandwidth, codingRate, preambleLength, payloadLen, crcOn, invertIrq);


	xTaskCreate(&task_tx, "TX", 1024*4, NULL, 5, NULL);

}
*/