#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <mosquitto.h>

#define SERIAL_PORT "/dev/ttyACM0"  // Adjust if needed
#define BAUDRATE B115200
#define MQTT_BROKER "localhost"
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
    tty.c_cflag &= ~PARENB; // No parity
    tty.c_cflag &= ~CSTOPB; // 1 stop bit
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8; // 8 bits per byte
    tty.c_cflag &= ~CRTSCTS; // No flow control

    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // Raw input
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // No software flow control
    tty.c_oflag &= ~OPOST; // Raw output

    tcsetattr(serial_port, TCSANOW, &tty);
    return 0;
}

void publish_mqtt(struct mosquitto *mosq, char *data) {
    int ret = mosquitto_publish(mosq, NULL, MQTT_TOPIC, strlen(data), data, 0, false);
    if (ret != MOSQ_ERR_SUCCESS) {
        fprintf(stderr, "Failed to publish MQTT message: %s\n", mosquitto_strerror(ret));
    }
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

    // Initialize MQTT
    mosquitto_lib_init();
    struct mosquitto *mosq = mosquitto_new(NULL, true, NULL);
    if (!mosq) {
        fprintf(stderr, "Failed to create MQTT client\n");
        close(serial_port);
        return 1;
    }

    if (mosquitto_connect(mosq, MQTT_BROKER, MQTT_PORT, 60) != MOSQ_ERR_SUCCESS) {
        fprintf(stderr, "Failed to connect to MQTT broker\n");
        mosquitto_destroy(mosq);
        close(serial_port);
        return 1;
    }

    char buffer[256];
    while (1) {
        int bytes_read = read(serial_port, buffer, sizeof(buffer) - 1);
        if (bytes_read > 0) {
            buffer[bytes_read] = '\0'; // Null-terminate
            printf("Received: %s\n", buffer);
            publish_mqtt(mosq, buffer);
        }
        usleep(100000); // Sleep 100ms
    }

    mosquitto_destroy(mosq);
    mosquitto_lib_cleanup();
    close(serial_port);
    return 0;
}