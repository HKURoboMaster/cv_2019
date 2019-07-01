/****************************************************************************
 *  Copyright (C) 2019 RoboMaster
 *  Copyright (C) 2019 Brett Dong
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of 
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/

#include "protocol.h"
#include "crc.h"
#include <cstring>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#ifdef __linux__
#include <sys/select.h>
#endif
using namespace std;

namespace protocol
{

int serial_fd;

bool Connect(const char *serial_device)
{
    int stop_bits = 1;
    int data_bits = 8;
    char parity_bits = 'N';
    termios new_termios, old_termios;
    fd_set serial_fd_set;

#ifdef __arm__
    serial_fd = open(serial_device, O_RDWR | O_NONBLOCK);
#elif __x86_64__
    serial_fd = open(serial_device, O_RDWR | O_NOCTTY);
#else
    serial_fd = open(serial_device, O_RDWR | O_NOCTTY);
#endif

    if(serial_fd < 0)
    {
        return false;
    }

    /* save current port parameter */
    if (tcgetattr(serial_fd, &old_termios) != 0)
    {
        return false;
    }
    memset(&new_termios, 0, sizeof(new_termios));

    /* config the size of char */
    new_termios.c_cflag |= CLOCAL | CREAD;
    new_termios.c_cflag &= ~CSIZE;

    /* config data bit */
    switch (data_bits)
    {
    case 7:
        new_termios.c_cflag |= CS7;
        break;
    case 8:
        new_termios.c_cflag |= CS8;
        break;
    default:
        new_termios.c_cflag |= CS8;
        break; //8N1 default config
    }
    /* config the parity bit */
    switch (parity_bits)
    {
    /* odd */
    case 'O':
    case 'o':
        new_termios.c_cflag |= PARENB;
        new_termios.c_cflag |= PARODD;
        break;
    /* even */
    case 'E':
    case 'e':
        new_termios.c_cflag |= PARENB;
        new_termios.c_cflag &= ~PARODD;
        break;
    /* none */
    case 'N':
    case 'n':
        new_termios.c_cflag &= ~PARENB;
        break;
    default:
        new_termios.c_cflag &= ~PARENB;
        break; //8N1 default config
    }
    /* config baudrate */

    /* set standard baudrate */
#if defined(__linux__)
    cfsetispeed(&new_termios, B921600);
    cfsetospeed(&new_termios, B921600);
#elif defined(__APPLE__)
    cfsetispeed(&new_termios, B38400);
    cfsetospeed(&new_termios, B38400);
#endif

    /* config stop bit */
    if (stop_bits == 1)
        new_termios.c_cflag &= ~CSTOPB;
    else if (stop_bits == 2)
        new_termios.c_cflag |= CSTOPB;
    else
        new_termios.c_cflag &= ~CSTOPB; //8N1 default config

    /* config waiting time & min number of char */
    new_termios.c_cc[VTIME] = 1;
    new_termios.c_cc[VMIN] = 18;

    /* using the raw data mode */
    new_termios.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    new_termios.c_oflag &= ~OPOST;

    /* flush the hardware fifo */
    tcflush(serial_fd, TCIFLUSH);

    /* activite the configuration */
    if ((tcsetattr(serial_fd, TCSANOW, &new_termios)) != 0)
    {
        return false;
    }

    FD_ZERO(&serial_fd_set);
    FD_SET(serial_fd, &serial_fd_set);
    return true;
}

void Disconnect()
{
    close(serial_fd);
    serial_fd = -1;
}

#define GIMBAL_CMD_SET (0x03u)
#define MANIFOLD2_ADDRESS (0x00u)
#define GIMBAL_ADDRESS (0x02u)
#pragma pack(push, 1)
#define CMD_SET_GIMBAL_ANGLE (0x03u)
struct gimbal_ctrl
{
    union
    {
        uint8_t flag;
        struct
        {
            uint8_t yaw_mode: 1;
            uint8_t pitch_mode: 1;
        } bit;
    } ctrl;
    int16_t pitch;
    int16_t yaw;
};

struct header
{
    uint32_t sof : 8;
    uint32_t length : 10;
    uint32_t version : 6;
    uint32_t session_id : 5;
    uint32_t is_ack : 1;
    uint32_t reserved0 : 2; // Always 0
    uint32_t sender: 8;
    uint32_t receiver: 8;
    uint32_t reserved1 : 16;
    uint32_t seq_num : 16;
    uint32_t crc : 16;
    header()
    {
        sof = 0xAA;
        version = 0x00;
        session_id = 0;
        is_ack = 0;
        reserved0 = 0;
        sender = MANIFOLD2_ADDRESS;
        receiver = GIMBAL_ADDRESS;
        reserved1 = 0;
        seq_num = 0;
    }
};

#pragma pack(pop)

void SendGimbalAngle(const float yaw, const float pitch)
{
    static const uint8_t cmd_set_prefix[] = {CMD_SET_GIMBAL_ANGLE, GIMBAL_CMD_SET};
    static const uint32_t HEADER_LEN = sizeof(header), CMD_SET_PREFIX_LEN = 2*sizeof(uint8_t);
    static const uint32_t CRC_HEADER_LEN = sizeof(uint16_t), CRC_DATA_LEN = sizeof(uint32_t);
    static const uint32_t DATA_LEN = sizeof(gimbal_ctrl);
    static const uint16_t pack_length = HEADER_LEN + CMD_SET_PREFIX_LEN + DATA_LEN + CRC_DATA_LEN;
    static header header_data;
    static gimbal_ctrl gimbal_ctrl_data;
    static uint8_t buffer[1024];

    gimbal_ctrl_data.ctrl.bit.pitch_mode = 1;
    gimbal_ctrl_data.ctrl.bit.yaw_mode = 1;
    gimbal_ctrl_data.pitch = pitch;
    gimbal_ctrl_data.yaw = yaw;

    ++header_data.seq_num;
    header_data.length = pack_length;
    header_data.crc = CRC16Calc((uint8_t*)&header_data, HEADER_LEN - CRC_HEADER_LEN);
    memcpy(buffer, (uint8_t*)&header_data, HEADER_LEN);
    memcpy(buffer + HEADER_LEN, cmd_set_prefix, CMD_SET_PREFIX_LEN);
    memcpy(buffer + HEADER_LEN + CMD_SET_PREFIX_LEN, (uint8_t*)&gimbal_ctrl_data, DATA_LEN);
    uint32_t crc_data = CRC32Calc(buffer, pack_length - CRC_DATA_LEN);
    memcpy(buffer + pack_length - CRC_DATA_LEN, &crc_data, CRC_DATA_LEN);
    write(serial_fd, buffer, pack_length);
}

}
