#include <string.h>

#include "config.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h" // ¡Importante: Añadido para los Event Groups de FreeRTOS!

#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"

#include "queuemonitor.h"
#include "wifi_esp32.h" // Asegúrate de que este sea el nombre correcto de tu archivo
#include "stm32_legacy.h"
#define DEBUG_MODULE  "WIFI_UDP"
#include "debug_cf.h"

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4, 4, 0)
#include "esp_mac.h"
#endif
#include "espnow.h"
#include "espnow_ctrl.h"
#include "espnow_utils.h"

#define UDP_SERVER_PORT         2390
#define UDP_SERVER_BUFSIZE      64

// --- CONFIGURACIÓN DE TU RED DOMÉSTICA ---
// Puedes definir estas en config.h o directamente aquí.
// Si las defines aquí, asegúrate de que no haya definiciones en config.h que las sobrescriban.
#ifndef HOME_WIFI_SSID
#define HOME_WIFI_SSID "INFINITUM0361"
#endif

#ifndef HOME_WIFI_PASSWORD
#define HOME_WIFI_PASSWORD "cx4WSwVdRV"
#endif

// --- CONFIGURACIÓN DEL ACCESS POINT DEL DRON (Si falla la conexión a la red doméstica) ---
#ifndef DRONE_AP_SSID_BASE
#define DRONE_AP_SSID_BASE "ESP-DRONE" // El SSID final será ESP-DRONE_MACADDRESS
#endif

#ifndef DRONE_AP_PASSWORD
#define DRONE_AP_PASSWORD "espdrone123" // Contraseña para el AP del dron (¡Cámbiala por seguridad!)
#endif

// Variables que probablemente vienen de config.h o el Kconfig del proyecto
#define WIFI_MAX_STA_CONN CONFIG_WIFI_MAX_STA_CONN
#define WIFI_CHANNEL_DEFAULT 1 // Canal por defecto para el AP (puede ser configurado en config.h)

// Tiempo máximo para intentar conectar a la red doméstica antes de crear el AP (en milisegundos)
#define WIFI_CONNECTION_TIMEOUT_MS 10000 

// Bits para el Event Group
#define CONNECTED_BIT BIT0

static struct sockaddr_storage source_addr;

static char current_wifi_ssid[32] = "";
static char current_wifi_pwd[64] = "";
static uint8_t current_wifi_channel = WIFI_CHANNEL_DEFAULT;

static int sock;
static xQueueHandle udpDataRx;
static xQueueHandle udpDataTx;

static bool isInit = false;
static bool isUDPInit = false;
static bool isUDPConnected = false;
static esp_netif_t *current_netif = NULL; // Para manejar el netif activo (AP o STA)

static esp_err_t udp_server_create(void *arg);

#ifndef MAC2STR
#define MAC2STR(a) (a)[0], (a)[1], (a)[2], (a)[3], (a)[4], (a)[5]
#define MACSTR "%02x:%02x:%02x:%02x:%02x:%02x"
#endif

static uint8_t calculate_cksum(void *data, size_t len)
{
    unsigned char *c = data;
    int i;
    unsigned char cksum = 0;

    for (i = 0; i < len; i++) {
        cksum += *(c++);
    }
    return cksum;
}

// --- EVENT HANDLERS ---
static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    // Manejo de eventos en modo AP (cuando otros se conectan al dron)
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t *event = (wifi_event_ap_staconnected_t *) event_data;
        DEBUG_PRINT_LOCAL("station " MACSTR " join, AID=%d", MAC2STR(event->mac), event->aid);
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t *event = (wifi_event_ap_stadisconnected_t *) event_data;
        DEBUG_PRINT_LOCAL("station " MACSTR " leave, AID=%d", MAC2STR(event->mac), event->aid);
    // Manejo de eventos en modo STA (cuando el dron se conecta a un router)
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect(); // Intenta conectar al AP
        DEBUG_PRINT_LOCAL("STA_START: Trying to connect to AP...");
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        DEBUG_PRINT_LOCAL("STA_DISCONNECTED: Connection lost or failed. Will be handled by wifiInit timeout.");
        // Aquí no reintentamos automáticamente para que la lógica de wifiInit maneje el timeout y el cambio a AP.
        // Si el EventGroup fue creado, se puede limpiar el bit si se desea forzar reintento en otra parte.
        EventGroupHandle_t wifi_event_group = (EventGroupHandle_t*)arg;
        if (wifi_event_group != NULL) {
            xEventGroupClearBits(wifi_event_group, CONNECTED_BIT); // Limpia el bit para que la espera pueda continuar si se reintenta
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *) event_data;
        DEBUG_PRINT_LOCAL("STA_GOT_IP: Got IP:" IPSTR, IP2STR(&event->ip_info.ip));
        // Notifica éxito de conexión STA
        EventGroupHandle_t wifi_event_group = (EventGroupHandle_t*)arg;
        if (wifi_event_group != NULL) {
            xEventGroupSetBits(wifi_event_group, CONNECTED_BIT); 
        }
    }
}

static void app_espnow_event_handler(void *handler_args, esp_event_base_t base, int32_t id, void *event_data)
{
    if (base != ESP_EVENT_ESPNOW) {
        return;
    }
    switch (id) {
        case ESP_EVENT_ESPNOW_CTRL_BIND: {
            espnow_ctrl_bind_info_t *info = (espnow_ctrl_bind_info_t *)event_data;
            DEBUG_PRINT_LOCAL("bind, uuid: " MACSTR ", initiator_type: %d", MAC2STR(info->mac), info->initiator_attribute);
            break;
        }
        case ESP_EVENT_ESPNOW_CTRL_UNBIND: {
            espnow_ctrl_bind_info_t *info = (espnow_ctrl_bind_info_t *)event_data;
            DEBUG_PRINT_LOCAL("unbind, uuid: " MACSTR ", initiator_type: %d", MAC2STR(info->mac), info->initiator_attribute);
            break;
        }
        default:
            break;
    }
}

static void espnow_ctrl_data_cb(espnow_attribute_t initiator_attribute,
                                       espnow_attribute_t responder_attribute,
                                       uint32_t status1,
                                       int status2,
                                       int lx_value,
                                       int ly_value,
                                       int rx_value,
                                       int ry_value,
                                       int channel_one_value,
                                       int channel_two_value)
{
    UDPPacket inPacket;
    inPacket.size = 7;
    inPacket.data[0] = 'n';
    inPacket.data[1] = 'o';
    inPacket.data[2] = 'w';
    inPacket.data[3] = lx_value & 0xFF;
    inPacket.data[4] = ly_value & 0xFF;
    inPacket.data[5] = ry_value & 0xFF;
    inPacket.data[6] = rx_value & 0xFF;
    xQueueSend(udpDataRx, &inPacket, 0);
}

// --- FUNCIONES EXISTENTES (SIN CAMBIOS MAYORES) ---
bool wifiTest(void)
{
    return isInit;
}

bool wifiGetDataBlocking(UDPPacket *in)
{
    while (xQueueReceive(udpDataRx, in, portMAX_DELAY) != pdTRUE) {
        vTaskDelay(M2T(10));
    }
    return true;
}

bool wifiSendData(uint32_t size, uint8_t *data)
{
    UDPPacket outStage = {0};
    outStage.size = size;
    memcpy(outStage.data, data, size);
    return (xQueueSend(udpDataTx, &outStage, M2T(100)) == pdTRUE);
}

static esp_err_t udp_server_create(void *arg)
{
    if (isUDPInit){
        return ESP_OK;
    }

    static struct sockaddr_in dest_addr = {0};
    dest_addr.sin_addr.s_addr = htonl(INADDR_ANY); // Escucha en cualquier IP disponible
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(UDP_SERVER_PORT);

    sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock < 0) {
        DEBUG_PRINT_LOCAL("Unable to create socket: errno %d", errno);
        return ESP_FAIL;
    }
    DEBUG_PRINT_LOCAL("Socket created");

    int err = bind(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
    if (err < 0) {
        DEBUG_PRINT_LOCAL("Socket unable to bind: errno %d", errno);
    }
    DEBUG_PRINT_LOCAL("Socket bound, port %d", UDP_SERVER_PORT);

    isUDPInit = true;
    return ESP_OK;
}

static void udp_server_rx_task(void *pvParameters)
{
    socklen_t socklen = sizeof(source_addr);
    char rx_buffer[UDP_SERVER_BUFSIZE];
    UDPPacket inPacket = {0};

    while (true) {
        if(isUDPInit == false) {
            vTaskDelay(20);
            continue;
        }
        int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer), 0, (struct sockaddr *)&source_addr, &socklen);
        if (len < 0) {
            DEBUG_PRINT_LOCAL("recvfrom failed: errno %d", errno);
            vTaskDelay(10); // Evita un bucle apretado en caso de error persistente
            continue;
        } else if(len > WIFI_RX_TX_PACKET_SIZE) {
            DEBUG_PRINT_LOCAL("Received data length = %d > %d", len, WIFI_RX_TX_PACKET_SIZE);
            continue;
        } else {
            uint8_t cksum = rx_buffer[len - 1];
            if (cksum == calculate_cksum(rx_buffer, len - 1)) {
                inPacket.size = len - 1;
                memcpy(inPacket.data, rx_buffer, inPacket.size);
                xQueueSend(udpDataRx, &inPacket, M2T(10));
                if(!isUDPConnected) isUDPConnected = true; // Marca como conectado cuando recibe el primer paquete
            }else{
                DEBUG_PRINT_LOCAL("udp packet cksum unmatched");
            }
        }
    }
}

static void udp_server_tx_task(void *pvParameters)
{
    UDPPacket outPacket = {0};
    while (TRUE) {
        if(isUDPInit == false) {
            vTaskDelay(20);
            continue;
        }
        // Solo envía si hay un cliente conectado (útil en modo AP) o si ya hubo comunicación (modo STA)
        if ((xQueueReceive(udpDataTx, &outPacket, portMAX_DELAY) == pdTRUE) && isUDPConnected) {
            outPacket.data[outPacket.size] = calculate_cksum(outPacket.data, outPacket.size);
            outPacket.size += 1;

            int err = sendto(sock, outPacket.data, outPacket.size, 0, (struct sockaddr *)&source_addr, sizeof(source_addr));
            if (err < 0) {
                DEBUG_PRINT_LOCAL("Error occurred during sending: errno %d", errno);
                continue;
            }
        } else {
             // Si no hay conexión UDP, espera un poco para no consumir CPU innecesariamente
            vTaskDelay(M2T(10));
        }
    }
}


// --- FUNCIÓN PRINCIPAL DE INICIALIZACIÓN DE WIFI ---
void wifiInit(void)
{
    if (isInit) {
        return;
    }

    udpDataRx = xQueueCreate(16, sizeof(UDPPacket));
    DEBUG_QUEUE_MONITOR_REGISTER(udpDataRx);
    udpDataTx = xQueueCreate(16, sizeof(UDPPacket));
    DEBUG_QUEUE_MONITOR_REGISTER(udpDataTx);

    espnow_storage_init();
    
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // Crear un Event Group para la comunicación entre el handler de eventos y la tarea principal
    EventGroupHandle_t wifi_event_group = xEventGroupCreate();
    
    // Registrar handlers de eventos Wi-Fi e IP
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                    ESP_EVENT_ANY_ID,
                    &wifi_event_handler,
                    (void*)wifi_event_group, // Pasar el grupo de eventos para notificar
                    NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                    IP_EVENT_STA_GOT_IP,
                    &wifi_event_handler,
                    (void*)wifi_event_group, // Pasar el grupo de eventos para notificar
                    NULL));
    
    // --- INTENTAR MODO ESTACIÓN (Cliente) ---
    DEBUG_PRINT_LOCAL("Attempting to connect to home Wi-Fi: %s", HOME_WIFI_SSID);
    current_netif = esp_netif_create_default_wifi_sta(); // Crear interfaz para STA
    
    wifi_config_t sta_config = {
        .sta = {
            .ssid = "",
            .password = "",
            .threshold.authmode = WIFI_AUTH_WPA2_PSK, // O el modo de autenticación de tu red
            .pmf_cfg = {
                .capable = true,
                .required = false
            },
        },
    };
    strcpy((char*)sta_config.sta.ssid, HOME_WIFI_SSID);
    strcpy((char*)sta_config.sta.password, HOME_WIFI_PASSWORD);

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &sta_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    // Esperar a la conexión STA o timeout
    EventBits_t bits = xEventGroupWaitBits(wifi_event_group,
                                           CONNECTED_BIT, // Espera el bit de "IP_EVENT_STA_GOT_IP"
                                           pdFALSE,       // No limpiar el bit al salir (la lógica AP lo hará)
                                           pdFALSE,       // No esperar todos los bits (solo uno)
                                           pdMS_TO_TICKS(WIFI_CONNECTION_TIMEOUT_MS));

    if (bits & CONNECTED_BIT) {
        // Conexión STA exitosa
        DEBUG_PRINT_LOCAL("Successfully connected to home Wi-Fi!");
        strcpy(current_wifi_ssid, HOME_WIFI_SSID);
        strcpy(current_wifi_pwd, HOME_WIFI_PASSWORD);
        
        // El canal en modo STA lo determina el AP. No se puede establecer estáticamente aquí.
        // Si necesitas saberlo para depuración, puedes usar esp_wifi_get_channel() más adelante.
        // current_wifi_channel = X; // No es relevante para la operación en modo STA de esta manera.
    } else {
        // Falló la conexión STA o hubo timeout, cambiar a modo Access Point (AP)
        DEBUG_PRINT_LOCAL("Failed to connect to home Wi-Fi. Switching to AP mode...");
        
        // Detener y reconfigurar Wi-Fi para AP
        ESP_ERROR_CHECK(esp_wifi_stop());
        esp_netif_destroy(current_netif); // Destruir la interfaz STA

        uint8_t mac[6];
        ESP_ERROR_CHECK(esp_wifi_get_mac(ESP_IF_WIFI_AP, mac)); // Obtener MAC para el SSID del AP
        sprintf(current_wifi_ssid, "%s_%02X%02X%02X%02X%02X%02X", DRONE_AP_SSID_BASE, MAC2STR(mac));
        strcpy(current_wifi_pwd, DRONE_AP_PASSWORD);
        current_wifi_channel = WIFI_CHANNEL_DEFAULT; // Usar el canal por defecto para el AP del dron

        current_netif = esp_netif_create_default_wifi_ap(); // Crear interfaz para AP
        wifi_config_t ap_config = {
            .ap = {
                .channel = current_wifi_channel,
                .max_connection = WIFI_MAX_STA_CONN,
                .authmode = WIFI_AUTH_WPA_WPA2_PSK, // Se asume WPA2-PSK para el AP
            },
        };

        memcpy(ap_config.ap.ssid, current_wifi_ssid, strlen(current_wifi_ssid) + 1);
        ap_config.ap.ssid_len = strlen(current_wifi_ssid);
        memcpy(ap_config.ap.password, current_wifi_pwd, strlen(current_wifi_pwd) + 1);

        if (strlen(current_wifi_pwd) == 0) {
            ap_config.ap.authmode = WIFI_AUTH_OPEN; // Si la contraseña está vacía, abre el AP
        }

        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
        ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_AP, &ap_config));
        ESP_ERROR_CHECK(esp_wifi_start());
        esp_wifi_set_channel(current_wifi_channel, WIFI_SECOND_CHAN_NONE); // Establecer canal explícitamente

        // Configurar IP estática y DHCP para el AP del dron
        // Esta es la IP que tendrá el dron cuando actúe como Access Point
        esp_netif_ip_info_t ip_info = {
            .ip.addr = ipaddr_addr("192.168.4.1"), // IP del AP del dron
            .netmask.addr = ipaddr_addr("255.255.255.0"),
            .gw.addr      = ipaddr_addr("192.168.4.1"), // Gateway del AP del dron (misma IP que el dron AP)
        };
        ESP_ERROR_CHECK(esp_netif_dhcps_stop(current_netif));
        ESP_ERROR_CHECK(esp_netif_set_ip_info(current_netif, &ip_info));
        ESP_ERROR_CHECK(esp_netif_dhcps_start(current_netif));
        DEBUG_PRINT_LOCAL("wifi_init_softap complete. SSID:%s password:%s", current_wifi_ssid, current_wifi_pwd);
    }
    
    // El EventGroup se puede liberar si ya no se va a usar en el futuro de la ejecución,
    // pero para este ejemplo, lo dejaremos para simplificar, ya que los handlers siguen apuntando a él.
    // xEventGroupDelete(wifi_event_group); 

    // --- INICIALIZACIÓN DE ESP-NOW (independiente del modo Wi-Fi) ---
    // ESP-NOW puede coexistir con el modo STA o AP
    esp_event_handler_register(ESP_EVENT_ESPNOW, ESP_EVENT_ANY_ID, app_espnow_event_handler, NULL);
    espnow_config_t espnow_config = ESPNOW_INIT_CONFIG_DEFAULT();
    espnow_init(&espnow_config);
    ESP_ERROR_CHECK(espnow_ctrl_responder_bind(30 * 1000, -55, NULL));
    espnow_ctrl_responder_data(espnow_ctrl_data_cb);

    // --- INICIALIZACIÓN DE UDP SERVER (independiente del modo Wi-Fi) ---
    if (udp_server_create(NULL) == ESP_FAIL) {
        DEBUG_PRINT_LOCAL("UDP server create socket failed");
    } else {
        DEBUG_PRINT_LOCAL("UDP server create socket succeed");
    }
    xTaskCreate(udp_server_tx_task, UDP_TX_TASK_NAME, UDP_TX_TASK_STACKSIZE, NULL, UDP_TX_TASK_PRI, NULL);
    xTaskCreate(udp_server_rx_task, UDP_RX_TASK_NAME, UDP_RX_TASK_STACKSIZE, NULL, UDP_RX_TASK_PRI, NULL);
    isInit = true;
}

// #include <string.h>

// #include "config.h"
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"

// #include "esp_system.h"
// #include "esp_wifi.h"
// #include "esp_event.h"
// #include "esp_netif.h"
// #include "lwip/err.h"
// #include "lwip/sockets.h"
// #include "lwip/sys.h"
// #include "lwip/netdb.h"

// #include "queuemonitor.h"
// #include "wifi_esp32.h"
// #include "stm32_legacy.h"
// #define DEBUG_MODULE  "WIFI_UDP"
// #include "debug_cf.h"

// #if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4, 4, 0)
// #include "esp_mac.h"
// #endif
// #include "espnow.h"
// #include "espnow_ctrl.h"
// #include "espnow_utils.h"

// #define UDP_SERVER_PORT         2390
// #define UDP_SERVER_BUFSIZE      64

// static struct sockaddr_storage source_addr;

// static char WIFI_SSID[32] = "";
// static char WIFI_PWD[64] = CONFIG_WIFI_PASSWORD;
// static uint8_t WIFI_CH = CONFIG_WIFI_CHANNEL;
// #define WIFI_MAX_STA_CONN CONFIG_WIFI_MAX_STA_CONN

// #ifndef MAC2STR
// #define MAC2STR(a) (a)[0], (a)[1], (a)[2], (a)[3], (a)[4], (a)[5]
// #define MACSTR "%02x:%02x:%02x:%02x:%02x:%02x"
// #endif

// static int sock;
// static xQueueHandle udpDataRx;
// static xQueueHandle udpDataTx;

// static bool isInit = false;
// static bool isUDPInit = false;
// static bool isUDPConnected = false;

// static esp_err_t udp_server_create(void *arg);

// static uint8_t calculate_cksum(void *data, size_t len)
// {
//     unsigned char *c = data;
//     int i;
//     unsigned char cksum = 0;

//     for (i = 0; i < len; i++) {
//         cksum += *(c++);
//     }

//     return cksum;
// }

// static void wifi_event_handler(void *arg, esp_event_base_t event_base,
//                                int32_t event_id, void *event_data)
// {
//     if (event_id == WIFI_EVENT_AP_STACONNECTED) {
//         wifi_event_ap_staconnected_t *event = (wifi_event_ap_staconnected_t *) event_data;
//         DEBUG_PRINT_LOCAL("station" MACSTR "join, AID=%d", MAC2STR(event->mac), event->aid);

//     } else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
//         wifi_event_ap_stadisconnected_t *event = (wifi_event_ap_stadisconnected_t *) event_data;
//         DEBUG_PRINT_LOCAL("station" MACSTR "leave, AID=%d", MAC2STR(event->mac), event->aid);
//     }
// }

// bool wifiTest(void)
// {
//     return isInit;
// };

// bool wifiGetDataBlocking(UDPPacket *in)
// {
//     /* command step - receive  02  from udp rx queue */
//     while (xQueueReceive(udpDataRx, in, portMAX_DELAY) != pdTRUE) {
//         vTaskDelay(M2T(10));
//     }; // Don't return until we get some data on the UDP

//     return true;
// };

// bool wifiSendData(uint32_t size, uint8_t *data)
// {
//     UDPPacket outStage = {0};
//     outStage.size = size;
//     memcpy(outStage.data, data, size);
//     // Dont' block when sending
//     return (xQueueSend(udpDataTx, &outStage, M2T(100)) == pdTRUE);
// };

// static esp_err_t udp_server_create(void *arg)
// {
//     if (isUDPInit){
//         return ESP_OK;
//     }

//     static struct sockaddr_in dest_addr = {0};
//     dest_addr.sin_addr.s_addr = htonl(INADDR_ANY);
//     dest_addr.sin_family = AF_INET;
//     dest_addr.sin_port = htons(UDP_SERVER_PORT);

//     sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
//     if (sock < 0) {
//         DEBUG_PRINT_LOCAL("Unable to create socket: errno %d", errno);
//         return ESP_FAIL;
//     }
//     DEBUG_PRINT_LOCAL("Socket created");

//     int err = bind(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
//     if (err < 0) {
//         DEBUG_PRINT_LOCAL("Socket unable to bind: errno %d", errno);
//     }
//     DEBUG_PRINT_LOCAL("Socket bound, port %d", UDP_SERVER_PORT);

//     isUDPInit = true;
//     return ESP_OK;
// }

// static void udp_server_rx_task(void *pvParameters)
// {
//     socklen_t socklen = sizeof(source_addr);
//     char rx_buffer[UDP_SERVER_BUFSIZE];
//     UDPPacket inPacket = {0};

//     while (true) {
//         if(isUDPInit == false) {
//             vTaskDelay(20);
//             continue;
//         }
//         int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer), 0, (struct sockaddr *)&source_addr, &socklen);
//         /* command step - receive  01 from Wi-Fi UDP */
//         if (len < 0) {
//             DEBUG_PRINT_LOCAL("recvfrom failed: errno %d", errno);
//             continue;
//         } else if(len > WIFI_RX_TX_PACKET_SIZE) {
//             DEBUG_PRINT_LOCAL("Received data length = %d > %d", len, WIFI_RX_TX_PACKET_SIZE);
//             continue;
//         } else {
//             uint8_t cksum = rx_buffer[len - 1];
//             //remove cksum, do not belong to CRTP
//             //check packet
//             if (cksum == calculate_cksum(rx_buffer, len - 1)) {
//                 //copy part of the UDP packet, the size not include cksum
//                 inPacket.size = len - 1;
//                 memcpy(inPacket.data, rx_buffer, inPacket.size);
//                 xQueueSend(udpDataRx, &inPacket, M2T(10));
//                 if(!isUDPConnected) isUDPConnected = true;
//             }else{
//                 DEBUG_PRINT_LOCAL("udp packet cksum unmatched");
//             }

// #ifdef DEBUG_UDP
//             printf("\nReceived size = %d cksum = %02X\n", inPacket.size, cksum);
//             for (size_t i = 0; i < inPacket.size; i++) {
//                 printf("%02X ", inPacket.data[i]);
//             }
//             printf("\n");
// #endif
//         }
//     }
// }

// static void udp_server_tx_task(void *pvParameters)
// {
//     UDPPacket outPacket = {0};
//     while (TRUE) {
//         if(isUDPInit == false) {
//             vTaskDelay(20);
//             continue;
//         }
//         if ((xQueueReceive(udpDataTx, &outPacket, portMAX_DELAY) == pdTRUE) && isUDPConnected) {
//             // append cksum to the packet
//             outPacket.data[outPacket.size] = calculate_cksum(outPacket.data, outPacket.size);
//             outPacket.size += 1;

//             int err = sendto(sock, outPacket.data, outPacket.size, 0, (struct sockaddr *)&source_addr, sizeof(source_addr));
//             if (err < 0) {
//                 DEBUG_PRINT_LOCAL("Error occurred during sending: errno %d", errno);
//                 continue;
//             }
// #ifdef DEBUG_UDP
//             printf("\nSend size = %d checksum = %02X\n", outPacket.size, outPacket.data[outPacket.size - 1]);
//             for (size_t i = 0; i < outPacket.size; i++) {
//                 printf("%02X ", outPacket.data[i]);
//             }
//             printf("\n");
// #endif
//         }
//     }
// }

// static void espnow_ctrl_data_cb(espnow_attribute_t initiator_attribute,
//                                        espnow_attribute_t responder_attribute,
//                                        uint32_t status1,
//                                        int status2,
//                                        int lx_value,
//                                        int ly_value,
//                                        int rx_value,
//                                        int ry_value,
//                                        int channel_one_value,
//                                        int channel_two_value)
// {
//     UDPPacket inPacket;
//     inPacket.size = 7;
//     inPacket.data[0] = 'n';
//     inPacket.data[1] = 'o';
//     inPacket.data[2] = 'w';
//     inPacket.data[3] = lx_value & 0xFF;
//     inPacket.data[4] = ly_value & 0xFF;
//     inPacket.data[5] = ry_value & 0xFF;
//     inPacket.data[6] = rx_value & 0xFF;
//     xQueueSend(udpDataRx, &inPacket, 0);
// }

// static void app_espnow_event_handler(void *handler_args, esp_event_base_t base, int32_t id, void *event_data)
// {
//     if (base != ESP_EVENT_ESPNOW) {
//         return;
//     }

//     switch (id) {
//     case ESP_EVENT_ESPNOW_CTRL_BIND: {
//         espnow_ctrl_bind_info_t *info = (espnow_ctrl_bind_info_t *)event_data;
//         DEBUG_PRINT_LOCAL("bind, uuid: " MACSTR ", initiator_type: %d", MAC2STR(info->mac), info->initiator_attribute);
//         break;
//     }

//     case ESP_EVENT_ESPNOW_CTRL_UNBIND: {
//         espnow_ctrl_bind_info_t *info = (espnow_ctrl_bind_info_t *)event_data;
//         DEBUG_PRINT_LOCAL("unbind, uuid: " MACSTR ", initiator_type: %d", MAC2STR(info->mac), info->initiator_attribute);
//         break;
//     }

//     default:
//         break;
//     }
// }

// void wifiInit(void)
// {
//     if (isInit) {
//         return;
//     }
//     // This should probably be reduced to a CRTP packet size
//     udpDataRx = xQueueCreate(16, sizeof(UDPPacket));
//     DEBUG_QUEUE_MONITOR_REGISTER(udpDataRx);
//     udpDataTx = xQueueCreate(16, sizeof(UDPPacket));
//     DEBUG_QUEUE_MONITOR_REGISTER(udpDataTx);

//     espnow_storage_init();
//     esp_netif_t *ap_netif = NULL;
//     ESP_ERROR_CHECK(esp_netif_init());
//     ESP_ERROR_CHECK(esp_event_loop_create_default());
//     ap_netif = esp_netif_create_default_wifi_ap();
//     uint8_t mac[6];

//     wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
//     ESP_ERROR_CHECK(esp_wifi_init(&cfg));

//     ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
//                     ESP_EVENT_ANY_ID,
//                     &wifi_event_handler,
//                     NULL,
//                     NULL));

//     ESP_ERROR_CHECK(esp_wifi_get_mac(ESP_IF_WIFI_AP, mac));
//     sprintf(WIFI_SSID, "%s_%02X%02X%02X%02X%02X%02X", CONFIG_WIFI_BASE_SSID, mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

//     wifi_config_t wifi_config = {
//         .ap = {
//             .channel = WIFI_CH,
//             .max_connection = WIFI_MAX_STA_CONN,
//             .authmode = WIFI_AUTH_WPA_WPA2_PSK,
//         },
//     };

//     memcpy(wifi_config.ap.ssid, WIFI_SSID, strlen(WIFI_SSID) + 1) ;
//     wifi_config.ap.ssid_len = strlen(WIFI_SSID);
//     memcpy(wifi_config.ap.password, WIFI_PWD, strlen(WIFI_PWD) + 1) ;

//     if (strlen(WIFI_PWD) == 0) {
//         wifi_config.ap.authmode = WIFI_AUTH_OPEN;
//     }

//     ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
//     ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_AP, &wifi_config));
//     ESP_ERROR_CHECK(esp_wifi_start());
//     esp_wifi_set_channel(WIFI_CH, WIFI_SECOND_CHAN_NONE);
//     espnow_config_t espnow_config = ESPNOW_INIT_CONFIG_DEFAULT();
//     espnow_init(&espnow_config);
//     esp_event_handler_register(ESP_EVENT_ESPNOW, ESP_EVENT_ANY_ID, app_espnow_event_handler, NULL);
//     ESP_ERROR_CHECK(espnow_ctrl_responder_bind(30 * 1000, -55, NULL));
//     espnow_ctrl_responder_data(espnow_ctrl_data_cb);
//     esp_netif_ip_info_t ip_info = {
//         .ip.addr = ipaddr_addr("192.168.43.42"),
//         .netmask.addr = ipaddr_addr("255.255.255.0"),
//         .gw.addr      = ipaddr_addr("192.168.43.42"),
//     };
//     ESP_ERROR_CHECK(esp_netif_dhcps_stop(ap_netif));
//     ESP_ERROR_CHECK(esp_netif_set_ip_info(ap_netif, &ip_info));
//     ESP_ERROR_CHECK(esp_netif_dhcps_start(ap_netif));
//     DEBUG_PRINT_LOCAL("wifi_init_softap complete.SSID:%s password:%s", WIFI_SSID, WIFI_PWD);

//     if (udp_server_create(NULL) == ESP_FAIL) {
//         DEBUG_PRINT_LOCAL("UDP server create socket failed");
//     } else {
//         DEBUG_PRINT_LOCAL("UDP server create socket succeed");
//     }
//     xTaskCreate(udp_server_tx_task, UDP_TX_TASK_NAME, UDP_TX_TASK_STACKSIZE, NULL, UDP_TX_TASK_PRI, NULL);
//     xTaskCreate(udp_server_rx_task, UDP_RX_TASK_NAME, UDP_RX_TASK_STACKSIZE, NULL, UDP_RX_TASK_PRI, NULL);
//     isInit = true;
// }