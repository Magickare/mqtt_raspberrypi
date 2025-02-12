#include <stdio.h>
#include <stdlib.h>
#include <mosquitto.h>

#define MQTT_BROKER "localhost"
#define MQTT_PORT 1883
#define MQTT_TOPIC "stm32/data"

// Callback function when a message is received
void on_message(struct mosquitto *mosq, void *userdata, const struct mosquitto_message *message) {
    if (message->payloadlen) {
        printf("Received message: %s\n", (char *)message->payload);
    } else {
        printf("Received empty message\n");
    }
}

int main() {
    mosquitto_lib_init();

    struct mosquitto *mosq = mosquitto_new(NULL, true, NULL);
    if (!mosq) {
        fprintf(stderr, "Failed to create MQTT client\n");
        return 1;
    }

    mosquitto_message_callback_set(mosq, on_message);

    if (mosquitto_connect(mosq, MQTT_BROKER, MQTT_PORT, 60) != MOSQ_ERR_SUCCESS) {
        fprintf(stderr, "Failed to connect to MQTT broker\n");
        mosquitto_destroy(mosq);
        return 1;
    }

    mosquitto_subscribe(mosq, NULL, MQTT_TOPIC, 0);
    printf("Subscribed to topic: %s\n", MQTT_TOPIC);

    mosquitto_loop_forever(mosq, -1, 1);

    mosquitto_destroy(mosq);
    mosquitto_lib_cleanup();
    return 0;
}