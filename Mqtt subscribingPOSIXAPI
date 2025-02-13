#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>

#define MQTT_BROKER "127.0.0.1"
#define MQTT_PORT 1883
#define MQTT_TOPIC "stm32/data"

int create_mqtt_connect_packet(char *packet) {
    int pos = 0;
    packet[pos++] = 0x10;
    packet[pos++] = 14;

    packet[pos++] = 0x00; packet[pos++] = 0x04;
    memcpy(&packet[pos], "MQTT", 4);
    pos += 4;

    packet[pos++] = 0x04;
    packet[pos++] = 0x02;

    packet[pos++] = 0x00; packet[pos++] = 0x3C;

    packet[pos++] = 0x00; packet[pos++] = 0x00;

    return pos;
}

int create_mqtt_subscribe_packet(char *packet) {
    int pos = 0;
    int topic_len = strlen(MQTT_TOPIC);

    packet[pos++] = 0x82;
    packet[pos++] = 2 + topic_len + 1;

    packet[pos++] = 0x00; packet[pos++] = 0x01;

    packet[pos++] = (topic_len >> 8) & 0xFF;
    packet[pos++] = topic_len & 0xFF;
    memcpy(&packet[pos], MQTT_TOPIC, topic_len);
    pos += topic_len;

    packet[pos++] = 0x00;

    return pos;
}

int main() {
    int sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock == -1) {
        perror("Socket creation failed");
        return 1;
    }

    struct sockaddr_in server_addr;
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(MQTT_PORT);
    server_addr.sin_addr.s_addr = inet_addr(MQTT_BROKER);

    if (connect(sock, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
        perror("MQTT connection failed");
        close(sock);
        return 1;
    }

    struct timeval timeout;
    timeout.tv_sec = 1;
    timeout.tv_usec = 0;
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, (const char *)&timeout, sizeof(timeout));

    char packet[256];
    send(sock, packet, create_mqtt_connect_packet(packet), 0);
    recv(sock, packet, sizeof(packet), 0);

    printf("Connected to MQTT broker\n");

    send(sock, packet, create_mqtt_subscribe_packet(packet), 0);
    recv(sock, packet, sizeof(packet), 0);
    printf("Subscribed to topic: %s\n", MQTT_TOPIC);

    while (1) {
        memset(packet, 0, sizeof(packet));
        int bytes_received = recv(sock, packet, sizeof(packet), 0);
        if (bytes_received > 0) {
            int topic_length = (packet[2] << 8) | packet[3];
            char *message = packet + 4 + topic_length;
            printf("Received: %s\n", message);
        }
        usleep(100000);
    }

    close(sock);
    return 0;
}
