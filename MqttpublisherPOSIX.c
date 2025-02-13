#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <arpa/inet.h>

#define SERIAL_PORT "/dev/ttyACM0"
#define BAUDRATE B115200
#define MQTT_BROKER "127.0.0.1"
#define MQTT_PORT 1883
#define MQTT_TOPIC "stm32/data"

int configure_serial(int serial_port) {
    struct termios tty;
    if (tcgetattr(serial_port, &tty) != 0) {
        perror("Error getting serial attributes");
        return -1;
    }

    cfsetospeed(&tty, BAUDRATE);
    cfsetispeed(&tty, BAUDRATE);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_oflag &= ~OPOST;

    tcsetattr(serial_port, TCSANOW, &tty);
    fcntl(serial_port, F_SETFL, O_NONBLOCK);
    return 0;
}

int create_mqtt_publish_packet(char *packet, const char *message) {
    int pos = 0;
    int topic_len = strlen(MQTT_TOPIC);
    int msg_len = strlen(message);
    
    packet[pos++] = 0x30;
    packet[pos++] = 2 + topic_len + msg_len;
    packet[pos++] = 0x00;
    packet[pos++] = topic_len;
    memcpy(&packet[pos], MQTT_TOPIC, topic_len);
    pos += topic_len;
    memcpy(&packet[pos], message, msg_len);
    pos += msg_len;

    return pos;
}

int main() {
    int serial_port = open(SERIAL_PORT, O_RDWR | O_NOCTTY | O_NDELAY);
    if (serial_port == -1) {
        perror("Error opening serial port");
        return 1;
    }
    if (configure_serial(serial_port) != 0) {
        close(serial_port);
        return 1;
    }

    int mqtt_sock;
    struct sockaddr_in server_addr;
    char packet[256];

    mqtt_sock = socket(AF_INET, SOCK_STREAM, 0);
    if (mqtt_sock == -1) {
        perror("Socket creation failed");
        return 1;
    }
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(MQTT_PORT);
    server_addr.sin_addr.s_addr = inet_addr(MQTT_BROKER);

    if (connect(mqtt_sock, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
        perror("MQTT connection failed");
        close(mqtt_sock);
        return 1;
    }

    struct timeval timeout;
    timeout.tv_sec = 1;
    timeout.tv_usec = 0;
    setsockopt(mqtt_sock, SOL_SOCKET, SO_RCVTIMEO, (const char *)&timeout, sizeof(timeout));

    packet[0] = 0x10;
    packet[1] = 14;
    memcpy(&packet[2], "\x00\x04MQTT\x04\x02\x00\x3C\x00\x00", 12);
    send(mqtt_sock, packet, 14, 0);
    recv(mqtt_sock, packet, sizeof(packet), 0);

    printf("Connected to MQTT broker\n");

    char buffer[256];
    int buffer_pos = 0;

    while (1) {
        char ch;
        int bytes_read = read(serial_port, &ch, 1);

        if (bytes_read > 0) {
            if (ch == '\n' || buffer_pos >= sizeof(buffer) - 1) {
                buffer[buffer_pos] = '\0';
                if (buffer_pos > 0) {
                    printf("Received: %s\n", buffer);
                    int len = create_mqtt_publish_packet(packet, buffer);
                    send(mqtt_sock, packet, len, 0);
                }
                buffer_pos = 0;
            } else {
                buffer[buffer_pos++] = ch;
            }
        }

        usleep(20000);
    }

    close(mqtt_sock);
    close(serial_port);
    return 0;
}
