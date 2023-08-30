/*---------------------------------------------------------*\
|  Cross Platform Serial COM Library for Windows and Linux  |
|  This library provides access to serial ports with a      |
|  common API for both Windows and Linux systems.  It       |
|  features read and write as well as tx/rx buffer flush.   |
|                                                           |
|  Adam Honse (calcprogrammer1@gmail.com), 1/21/2013        |
\*---------------------------------------------------------*/

#include "serial_port.h"
#include <QString>
#include <thread>

using namespace std::chrono_literals;

QSerialPort::Parity qparity[] {
    [SERIAL_PORT_PARITY_NONE] = QSerialPort::NoParity,
    [SERIAL_PORT_PARITY_ODD] = QSerialPort::OddParity,
    [SERIAL_PORT_PARITY_EVEN] = QSerialPort::EvenParity
};

QSerialPort::DataBits qdatabits[] {
    [SERIAL_PORT_SIZE_8] = QSerialPort::Data8,
    [SERIAL_PORT_SIZE_7] = QSerialPort::Data7,
    [SERIAL_PORT_SIZE_6] = QSerialPort::Data6,
    [SERIAL_PORT_SIZE_5] = QSerialPort::Data5
};

QSerialPort::StopBits qstopbits[] {
    [SERIAL_PORT_STOP_BITS_1] = QSerialPort::OneStop,
    [SERIAL_PORT_STOP_BITS_2] = QSerialPort::TwoStop
};

/*---------------------------------------------------------*\
|  serial_port (constructor)                                |
|    The default constructor does not initialize the serial |
|    port                                                   |
\*---------------------------------------------------------*/
serial_port::serial_port()
{
    /*-----------------------------------------------------*\
    | Set default port configuration but do not open        |
    \*-----------------------------------------------------*/
    serial.setBaudRate(9600);
    serial.setParity(QSerialPort::NoParity);
    serial.setDataBits(QSerialPort::Data8);
    serial.setStopBits(QSerialPort::OneStop);
    serial.setFlowControl(QSerialPort::HardwareControl);
}

/*---------------------------------------------------------*\
|  serial_port (constructor)                                |
|    When created with port information, the constructor    |
|    will automatically open port <name> at baud rate <baud>|
\*---------------------------------------------------------*/
serial_port::serial_port(const char * name, unsigned int baud)
{
    serial_port(name, baud, SERIAL_PORT_PARITY_NONE, SERIAL_PORT_SIZE_8, SERIAL_PORT_STOP_BITS_1, true);
}

/*---------------------------------------------------------*\
|  serial_port (constructor)                                |
|    When created with port information, the constructor    |
|    will automatically open port <name> at baud rate <baud>|
|    with the given port configuration                      |
\*---------------------------------------------------------*/
serial_port::serial_port
    (
    const char *            name,
    unsigned int            baud,
    serial_port_parity      parity,
    serial_port_size        size,
    serial_port_stop_bits   stop_bits,
    bool                    flow_control
    )
{
    /*-----------------------------------------------------*\
    | Set default port configuration and open               |
    \*-----------------------------------------------------*/
    serial.setBaudRate(baud);
    serial.setParity(qparity[parity]);
    serial.setDataBits(qdatabits[size]);
    serial.setStopBits(qstopbits[stop_bits]);

    if (flow_control) {
        serial.setFlowControl(QSerialPort::HardwareControl);
    }

    serial_open(name);
}

/*---------------------------------------------------------*\
|  ~serial_port (destructor)                                |
|    Closes the port before destroying the object           |
\*---------------------------------------------------------*/
serial_port::~serial_port()
{
    serial_close();
}

/*---------------------------------------------------------*\
|  serial_open                                              |
|    Opens the serial port using stored information         |
|    Sets the baud rate to the stored baud rate             |
|    8 data bits, no parity, one stop bit                   |
\*---------------------------------------------------------*/
bool serial_port::serial_open()
{
    return serial.open(QIODevice::ReadWrite);
}

/*---------------------------------------------------------*\
|  serial_open                                              |
|    Opens the serial port <name> without changing stored   |
|    baud rate                                              |
\*---------------------------------------------------------*/
bool serial_port::serial_open(const char * name)
{
    serial.setPortName(QString(name));

    return serial_open();
}

/*---------------------------------------------------------*\
|  serial_open                                              |
|    Opens the serial port <name> at baud rate <baud>       |
\*---------------------------------------------------------*/
bool serial_port::serial_open(const char* name, unsigned int baud)
{
    serial.setBaudRate(baud);

    return serial_open(name);
}

/*---------------------------------------------------------*\
|  serial_close                                             |
|    Closes the serial port                                 |
\*---------------------------------------------------------*/
void serial_port::serial_close()
{
    serial.close();
}

/*---------------------------------------------------------*\
|  serial_read                                              |
|    Reads <length> bytes from the serial port into <buffer>|
|    Returns the number of bytes actually read              |
|    If less than <length> bytes are available, it will read|
|    all available bytes                                    |
\*---------------------------------------------------------*/
int serial_port::serial_read(char * buffer, int length)
{
    return (int)serial.read(buffer, length);
}

/*---------------------------------------------------------*\
|  serial_write                                             |
|    Writes <length> bytes to the serial port from <buffer> |
|    Returns the number of bytes actually written           |
|    Does not check for null-termination, so if <length> is |
|    greater than the number of bytes in <buffer>, it will  |
|    read past <buffer> and may cause a segfault            |
\*---------------------------------------------------------*/
int serial_port::serial_write(char * buffer, int length)
{
    return (int)serial.write(buffer, length);
}

/*---------------------------------------------------------*\
|  serial_flush                                             |
\*---------------------------------------------------------*/
void serial_port::serial_flush_rx()
{
    serial.clear(QSerialPort::Input);
}

/*---------------------------------------------------------*\
|  serial_flush_tx                                          |
\*---------------------------------------------------------*/
void serial_port::serial_flush_tx()
{
    serial.clear(QSerialPort::Output);
}

/*---------------------------------------------------------*\
|  serial_break                                             |
\*---------------------------------------------------------*/
void serial_port::serial_break()
{
    serial.setBreakEnabled(true);
    std::this_thread::sleep_for(1ms);
    serial.setBreakEnabled(false);
}

void serial_port::serial_set_rts(bool rts)
{
    serial.setRequestToSend(rts);
}
