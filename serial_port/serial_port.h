/*---------------------------------------------------------*\
|  Cross Platform Serial COM Library for Windows and Linux  |
|  This library provides access to serial ports with a      |
|  common API for both Windows and Linux systems.  It       |
|  features read and write as well as tx/rx buffer flush.   |
|                                                           |
|  Adam Honse (calcprogrammer1@gmail.com), 1/21/2013        |
\*---------------------------------------------------------*/

#ifndef SERIAL_PORT_H
#define SERIAL_PORT_H

#include <QSerialPort>
#include <string.h>
#include <stdio.h>

/*-------------------------------------------------------------------------*\
|  Serial Port Enums                                                        |
\*-------------------------------------------------------------------------*/
typedef unsigned int serial_port_parity;
enum
{
    SERIAL_PORT_PARITY_NONE = 0,    /* No parity                           */
    SERIAL_PORT_PARITY_ODD  = 1,    /* Odd parity                          */
    SERIAL_PORT_PARITY_EVEN = 2,    /* Even parity                         */
};

typedef unsigned int serial_port_size;
enum
{
    SERIAL_PORT_SIZE_8      = 0,    /* 8 bits per byte                     */
    SERIAL_PORT_SIZE_7      = 1,    /* 7 bits per byte                     */
    SERIAL_PORT_SIZE_6      = 2,    /* 6 bits per byte                     */
    SERIAL_PORT_SIZE_5      = 3,    /* 5 bits per byte                     */
};

typedef unsigned int serial_port_stop_bits;
enum
{
    SERIAL_PORT_STOP_BITS_1 = 0,    /* 1 stop bit                          */
    SERIAL_PORT_STOP_BITS_2 = 1,    /* 2 stop bits                         */
};

/*-------------------------------------------------------------------------*\
|  Serial Port Class                                                        |
|    The reason for this class is that serial ports are treated differently |
|    on Windows and Linux.  By creating a class, those differences can be   |
|    made invisible to the program and make cross-platform usage easy       |
\*-------------------------------------------------------------------------*/
class serial_port
{
public:
    serial_port();
    serial_port(const char * name, unsigned int baud);
    serial_port(const char *            name,
                unsigned int            baud,
                serial_port_parity      parity,
                serial_port_size        size,
                serial_port_stop_bits   stop_bits,
                bool                    flow_control);

    ~serial_port();

    bool serial_open();
    bool serial_open(const char* name);
    bool serial_open(const char* name, unsigned int baud);

    void serial_close();

    void serial_set_baud(unsigned int baud);
    int serial_get_baud();

    int serial_read(char * buffer, int length);

    int serial_write(char * buffer, int length);

    void serial_flush_rx();
    void serial_flush_tx();
    void serial_break();

    void serial_set_rts(bool rts);

    int serial_available();

private:
    QSerialPort serial;
};

#endif  /* SERIAL_PORT_H */