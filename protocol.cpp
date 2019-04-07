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

#pragma pack(push, 1)
struct gimbal_ctrl
{
    uint32_t time;
    uint8_t ctrl_mode;
    float pit_ref, yaw_ref, tgt_dist, x, y, z;
    uint8_t visual_valid;
    gimbal_ctrl()
    {
        time = 0;
        ctrl_mode = 7;
        pit_ref = yaw_ref = tgt_dist = x = y = z = 0.0f;
        visual_valid = 0;
    }
};

struct header
{
    uint8_t sof;
    uint16_t data_length;
    uint8_t seq;
    uint8_t crc8;
    header()
    {
        sof = 0xa0u;
        seq = crc8 = 0;
        data_length = sizeof(gimbal_ctrl);
    }
};
#pragma pack(pop)

void Send(const float yaw, const float pitch)
{
    const uint16_t cmd_id = 0x00a1u;
    static header header_data;
    static gimbal_ctrl gimbal_ctrl_data;
    static uint8_t buffer[100];

    ++gimbal_ctrl_data.time;
    gimbal_ctrl_data.yaw_ref = yaw;
    gimbal_ctrl_data.pit_ref = pitch;

    ++header_data.seq;
    header_data.crc8 = CRC8Calc((uint8_t*)(&header_data), sizeof(header_data) - sizeof(header_data.crc8));
    
    memcpy(buffer, &header_data, sizeof(header_data));
    memcpy(buffer + sizeof(header_data), &cmd_id, sizeof(cmd_id));
    memcpy(buffer + sizeof(header_data) + sizeof(cmd_id), &gimbal_ctrl_data, sizeof(gimbal_ctrl_data));
    uint16_t crc16 = CRC16Calc(buffer, sizeof(header_data) + sizeof(cmd_id) + sizeof(gimbal_ctrl_data));
    memcpy(buffer + sizeof(header_data) + sizeof(cmd_id) + sizeof(gimbal_ctrl_data), &crc16, sizeof(crc16));

    write(serial_fd, buffer, sizeof(header_data) + sizeof(cmd_id) + sizeof(gimbal_ctrl_data) + sizeof(crc16));
}

}
