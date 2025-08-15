#include <iostream>
#include <array>
#include <cstring>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <cstdio>
#include "Packet.h"   // Your Packet struct
#include "CheckSum.h" // Your CRC functions

using namespace std;

int main()
{
    // ======== 1. Open and Configure UART ========
    int uart_fd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_SYNC);
    if (uart_fd < 0)
    {
        perror("Error: Failed to open UART port /dev/ttyUSB0");
        return 1;
    }

    struct termios tty{};
    if (tcgetattr(uart_fd, &tty) != 0)
    {
        perror("Error getting termios attributes");
        close(uart_fd);
        return 1;
    }

    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | PARODD | CSTOPB);
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_lflag = 0;
    tty.c_oflag = 0;
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 20; // 2 sec timeout

    if (tcsetattr(uart_fd, TCSANOW, &tty) != 0)
    {
        perror("Error setting termios attributes");
        close(uart_fd);
        return 1;
    }

    cout << "UART port configured. Waiting for STM32..." << endl;
    sleep(1);

    // ======== 2. Read Startup Message from STM32 ========
    char ready_buf[100] = {0};
    ssize_t bytes_read = read(uart_fd, ready_buf, sizeof(ready_buf) - 1);
    if (bytes_read > 0)
    {
        cout << "STM32 says: " << ready_buf;
    }
    else
    {
        cout << "Warning: No startup message from STM32." << endl;
    }

    // ======== 3. Create Packet ========
    // array<uint8_t, 4> payload = {0x01, -30, 0x01};
    // int16_t angle = -30; // signed value
    // array<uint8_t, 4> payload = {
    //     0x01,                                      // Motor ID
    //     static_cast<uint8_t>(angle & 0xFF),        // low byte
    //     static_cast<uint8_t>((angle >> 8) & 0xFF), // high byte
    //     0x01                                       // Direction
    // };
    array <uint8_t , 4> payload = {0x01 , 0x10 , 0x01, 0x00}; // Example payload for Motor
    Packet packet_to_send{};

    packet_to_send.start_packet = 0xAA55;
    packet_to_send.packetID = (uint8_t)(Motor_ID);
    packet_to_send.count = 1;
    memcpy(packet_to_send.payload, payload.data(), payload.size());

    // Use your checksum() from CheckSum.h
    uint8_t checksumData[6];
    memcpy(checksumData, payload.data(), 4);
    checksumData[4] = packet_to_send.packetID;
    checksumData[5] = packet_to_send.count;
    packet_to_send.checksum = checksum(checksumData, 6);
    packet_to_send.end_packet = 0x0D0A;

    // ======== 4. Send Packet ========
    uint8_t send_buffer[sizeof(Packet)];
    memcpy(send_buffer, &packet_to_send, sizeof(Packet));

    ssize_t bytes_written = write(uart_fd, send_buffer, sizeof(Packet));
    if (bytes_written != sizeof(Packet))
    {
        perror("Packet write error");
        close(uart_fd);
        return 1;
    }
    cout << "Packet sent (" << bytes_written << " bytes)!" << endl;

    // // ======== 5. Read Acknowledgment ========
    // char ack_buf[100] = {0};
    // bytes_read = read(uart_fd, ack_buf, sizeof(ack_buf) - 1);

    // if (bytes_read > 0) {
    //     cout << "Received acknowledgment: " << ack_buf << endl;
    // } else if (bytes_read == 0) {
    //     cout << "No acknowledgment received (timeout)." << endl;
    // } else {
    //     perror("Read error");
    // }

    close(uart_fd);
    return 0;
}
