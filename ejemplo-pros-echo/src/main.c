#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <nvs_flash.h>
#include <string.h>

#include "picoros.h"
#include "picoserdes.h"
#include "wifi_connection.h"

#define WIFI_SSID "OLIVERA2.4GHz"                   /* < SSID de la red WiFi */
#define WIFI_PASSWORD "31942649"                    /* < Contraseña WiFi */
#define ZENOH_ROUTER_ADDRESS "tcp/192.168.1.7:7447" /* < Dirección del router Zenoh */
#define ZENOH_NODE_NAME "pico_echo"                 /* < Nombre del nodo ROS */
#define ROS_DOMAIN_ID 0                             /* < Domain ID de ROS 2 */

static const char *TAG = "echo";
static picoros_publisher_t pub_echo = {
    .topic = {
        .name      = "echo_out",
        .type      = ROSTYPE_NAME(ros_String),
        .rihs_hash = ROSTYPE_HASH(ros_String),
    },
};

static uint8_t echo_pub_buf[1024];

static void
echo_callback (uint8_t *rx_data, size_t data_len)
{
    char *msg = NULL;
    ps_deserialize (rx_data, &msg, data_len);
    ESP_LOGI (TAG, "Recibido: %s", msg);

    size_t len = ps_serialize (echo_pub_buf, &msg, sizeof (echo_pub_buf));
    if (len > 0)
    {
        picoros_publish (&pub_echo, echo_pub_buf, len);
        ESP_LOGI (TAG, "Eco: %s", msg);
    }
    else
    {
        ESP_LOGE (TAG, "Error de serializacion en echo publish");
    }
}

static picoros_subscriber_t sub_echo = {
    .topic = {
        .name      = "echo_in",
        .type      = ROSTYPE_NAME(ros_String),
        .rihs_hash = ROSTYPE_HASH(ros_String),
    },
    .user_callback = echo_callback,
};

static picoros_node_t node = {
    .name      = ZENOH_NODE_NAME,
    .domain_id = ROS_DOMAIN_ID,
};

static void
init_nvs (void)
{
    esp_err_t ret = nvs_flash_init ();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK (nvs_flash_erase ());
        ret = nvs_flash_init ();
    }
    ESP_ERROR_CHECK (ret);
}

/* ── Punto de entrada ── */
void
app_main (void)
{
    ESP_LOGI (TAG, "Arrancando nodo echo...");

    /* 1. WiFi */
    init_nvs ();
    wifi_connect (WIFI_SSID, WIFI_PASSWORD);
    wifi_wait_connected ();
    ESP_LOGI (TAG, "WiFi conectado.");

    /* 2. Interfaz Zenoh */
    picoros_interface_t ifx = {
        .mode    = "client",
        .locator = ZENOH_ROUTER_ADDRESS,
    };
    ESP_LOGI (TAG, "Iniciando interfaz Pico-ROS [%s] en [%s]", ifx.mode, ifx.locator);
    while (picoros_interface_init (&ifx) == PICOROS_NOT_READY)
    {
        ESP_LOGD (TAG, "Esperando inicializacion RMW...");
        vTaskDelay (pdMS_TO_TICKS (500));
    }

    /* 3. Nodo Pico-ROS */
    ESP_LOGI (TAG, "Iniciando nodo [%s] dominio [%lu]", node.name, (unsigned long)node.domain_id);
    picoros_node_init (&node);

    /* 4. Publicador y suscriptor */
    picoros_publisher_declare (&node, &pub_echo);
    ESP_LOGI (TAG, "Publicador declarado en [%s]", pub_echo.topic.name);

    picoros_subscriber_declare (&node, &sub_echo);
    ESP_LOGI (TAG, "Suscriptor declarado en [%s]", sub_echo.topic.name);

    ESP_LOGI (TAG, "Nodo echo listo. Escuchando [echo_in], publicando [echo_out].");

    /* 5. busy loop — los topicos hacen todo el trabajo */
    for (;;)
    {
        vTaskDelay (pdMS_TO_TICKS (1000));
    }
}
