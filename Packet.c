#include "Packet.h"
#include "CheckSum.h"
#include <string.h>

uint16_t FillData(const uint8_t payload[PAYLOAD_SIZE], PacketID packetID)
{
    struct Packet packet;
    packet.start_packet = 0xAA55;
    packet.end_packet = 0x0D0A;
    packet.count = 1;
    packet.packetID = (uint8_t)packetID;

    memcpy(packet.payload, payload, PAYLOAD_SIZE);

    uint8_t checksumData[6];
    memcpy(checksumData, packet.payload, PAYLOAD_SIZE);
    checksumData[4] = (uint8_t)packetID;
    checksumData[5] = packet.count;

    packet.checksum = checksum(checksumData, sizeof(checksumData));

    return packet.checksum;
}
uint16_t FillData_MotorAngle(uint8_t id, int16_t angle, uint8_t direction) {
    struct Packet packet;
    packet.start_packet = 0xAA55;
    packet.end_packet = 0x0D0A;
    packet.count = 1;
    packet.packetID = MotorAngle_ID;

    packet.payload[0] = id;
    packet.payload[1] = (uint8_t)(angle & 0xFF);
    packet.payload[2] = (uint8_t)((angle >> 8) & 0xFF);
    packet.payload[3] = direction;

    uint8_t checksumData[6];
    memcpy(checksumData, packet.payload, PAYLOAD_SIZE);
    checksumData[4] = packet.packetID;
    checksumData[5] = packet.count;

    packet.checksum = checksum(checksumData, sizeof(checksumData));

    return packet.checksum;
}
uint8_t SerializePacket(const struct Packet *packet)
{
    if (!packet)
        return 4; // Null pointer

    // Validate start/end markers
    if (packet->start_packet != 0xAA55 || packet->end_packet != 0x0D0A)
    {
        printf("Invalid start or end packet values.\r\n");
        return 1;
    }

    // Validate checksum
    uint8_t checksumData[6];
    memcpy(checksumData, packet->payload, PAYLOAD_SIZE);
    checksumData[4] = packet->packetID;
    checksumData[5] = packet->count;

    uint16_t checksumResult = checksum(checksumData, sizeof(checksumData));
    if (packet->checksum != checksumResult)
    {
        return 2; // Checksum mismatch
    }

    // Process packet by ID
    switch (packet->packetID)
    {
    case Motor_ID:
    {
        struct Motor motor = {
            .ID = packet->payload[0],
            .speed = packet->payload[1],
            .direction = packet->payload[2]};
        // TODO: Add motor control logic
        break;
    }

    case MotorAngle_ID:
    {
        struct MotorAngle motorAngle = {
            .ID = packet->payload[0],
            .angle = (int16_t)packet->payload[1],
            .direction = packet->payload[2]};

        printf("Encoder Data send:\r\n");
        printf("ID: %u\r\n", motorAngle.ID);
        printf("Angle: %u\r\n", motorAngle.angle);
        printf("Direction: %u\r\n", motorAngle.direction);

        break;
    }

    case CarHorn_ID:
    {
        struct CarHorn carHorn = {
            .ID = packet->payload[0],
            .duartion = packet->payload[1]};
        break;
    }

    case CarLight_ID:
    {
        struct CarLight carLight = {
            .ID = packet->payload[0],
            .lightStatus = packet->payload[1]};
        break;
    }

    case CarConfirmation_ID:
    {
        struct CarConfirmation carConfirmation = {
            .ID = packet->payload[0],
            .confirmationStatus = packet->payload[1],
            .value = packet->payload[2]};
        break;
    }

    default:
    {
        printf("Unknown packet ID\r\n");
        return 3;
    }
    }

    return 0; // Success
}
