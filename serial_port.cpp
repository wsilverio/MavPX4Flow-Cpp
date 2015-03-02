/****************************************************************************
 *
 *        Baseado em github.com/mavlink/c_uart_interface_example,
 *            dos autores MAVlink Development Team:
 *                Trent Lukaczyk, <aerialhedgehog@gmail.com>
 *                Jaycee Lock,    <jaycee.lock@gmail.com>
 *                Lorenz Meier,   <lm@inf.ethz.ch>
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

// ------------------------------------------------------------------------------
//   Includes
// ------------------------------------------------------------------------------

#include "serial_port.h"


// ----------------------------------------------------------------------------------
//   Serial Port Manager Class
// ----------------------------------------------------------------------------------

// ------------------------------------------------------------------------------
//   Con/De structors
// ------------------------------------------------------------------------------
Serial_Port::
Serial_Port(char *&uart_name_ , int &baudrate_)
{
    initialize_defaults();
    uart_name = uart_name_;
    baudrate  = baudrate_;
}

Serial_Port::
Serial_Port()
{
    initialize_defaults();
}

Serial_Port::
~Serial_Port()
{
    // destroy mutex
    pthread_mutex_destroy(&lock);
}

void
Serial_Port::
initialize_defaults()
{
    // Initialize attributes
    debug  = false;
    fd     = -1;
    status = SERIAL_PORT_CLOSED;

    uart_name = (char*)"/dev/ttyACM0"; // ttyUSB0
    baudrate  = 57600;

    // Start mutex
    int result = pthread_mutex_init(&lock, NULL);
    if ( result != 0 )
    {
        printf("\n mutex init failed\n");
        throw 1;
    }
}


// ------------------------------------------------------------------------------
//   Read from Serial
// ------------------------------------------------------------------------------
int
Serial_Port::
read_message(mavlink_message_t &message)
{
    uint8_t          cp;
    mavlink_status_t status;
    uint8_t          msgReceived = false;

    // --------------------------------------------------------------------------
    //   READ FROM PORT
    // --------------------------------------------------------------------------

    // this function locks the port during read
    int result = _read_port(cp);


    // --------------------------------------------------------------------------
    //   PARSE MESSAGE
    // --------------------------------------------------------------------------
    if (result > 0)
    {
        // the parsing
        msgReceived = mavlink_parse_char(MAVLINK_COMM_1, cp, &message, &status);

        // check for dropped packets
        if ( (lastStatus.packet_rx_drop_count != status.packet_rx_drop_count) && debug )
        {
            printf("ERROR: DROPPED %d PACKETS\n", status.packet_rx_drop_count);
            unsigned char v=cp;
            fprintf(stderr,"%02x ", v);
        }
        lastStatus = status;
    }

    // Couldn't read from port
    else
    {
        fprintf(stderr, "ERROR: Could not read from fd %d\n", fd);
    }

    // --------------------------------------------------------------------------
    //   DEBUGGING REPORTS
    // --------------------------------------------------------------------------
    if(msgReceived && debug)
    {
        // Report info
        printf("Received message from serial with ID #%d (sys:%d|comp:%d):\n", message.msgid, message.sysid, message.compid);

        fprintf(stderr,"Received serial data: ");
        unsigned int i;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];

        // check message is write length
        unsigned int messageLength = mavlink_msg_to_send_buffer(buffer, &message);

        // message length error
        if (messageLength > MAVLINK_MAX_PACKET_LEN)
        {
            fprintf(stderr, "\nFATAL ERROR: MESSAGE LENGTH IS LARGER THAN BUFFER SIZE\n");
        }

        // print out the buffer
        else
        {
            for (i=0; i<messageLength; i++)
            {
                unsigned char v=buffer[i];
                fprintf(stderr,"%02x ", v);
            }
            fprintf(stderr,"\n");
        }
    }

    // Done!
    return msgReceived;
}

// ------------------------------------------------------------------------------
//   Write to Serial
// ------------------------------------------------------------------------------
int
Serial_Port::
write_message(mavlink_message_t &message)
{
    char buf[300];

    // Translate message to buffer
    unsigned len = mavlink_msg_to_send_buffer((uint8_t*)buf, &message);

    // Write buffer to serial port, locks port while writing
    _write_port(buf,len);

    return len;
}


// ------------------------------------------------------------------------------
//   Open Serial Port
// ------------------------------------------------------------------------------
/**
 * throws EXIT_FAILURE if could not open the port
 */
void
Serial_Port::
open_serial()
{

    // --------------------------------------------------------------------------
    //   OPEN PORT
    // --------------------------------------------------------------------------
    printf("OPEN PORT\n");

    fd = _open_port(uart_name);

    // Check success
    if (fd == -1)
    {
        printf("failure, could not open port.\n");
        throw EXIT_FAILURE;
    }

    // --------------------------------------------------------------------------
    //   SETUP PORT
    // --------------------------------------------------------------------------
    bool success = _setup_port(baudrate, 8, 1, false, false);

    // --------------------------------------------------------------------------
    //   CHECK STATUS
    // --------------------------------------------------------------------------
    if (!success)
    {
        printf("failure, could not configure port.\n");
        throw EXIT_FAILURE;
    }
    if (fd <= 0)
    {
        printf("Connection attempt to port %s with %d baud, 8N1 failed, exiting.\n", uart_name, baudrate);
        throw EXIT_FAILURE;
    }

    // --------------------------------------------------------------------------
    //   CONNECTED!
    // --------------------------------------------------------------------------
    printf("Connected to %s with %d baud, 8 data bits, no parity, 1 stop bit (8N1)\n", uart_name, baudrate);
    lastStatus.packet_rx_drop_count = 0;

    status = true;

    printf("\n");

    return;

}


// ------------------------------------------------------------------------------
//   Close Serial Port
// ------------------------------------------------------------------------------
void
Serial_Port::
close_serial()
{
    printf("CLOSE PORT\n");

    int result = close(fd);

    if ( result )
    {
        fprintf(stderr,"WARNING: Error on port close (%i)\n", result );
    }

    status = false;

    printf("\n");

}


// ------------------------------------------------------------------------------
//   Convenience Functions
// ------------------------------------------------------------------------------
void
Serial_Port::
start()
{
    open_serial();
}

void
Serial_Port::
stop()
{
    close_serial();
}


// ------------------------------------------------------------------------------
//   Quit Handler
// ------------------------------------------------------------------------------
void
Serial_Port::
handle_quit( int sig )
{
    try {
        stop();
    }
    catch (int error) {
        fprintf(stderr,"Warning, could not stop serial port\n");
    }
}


// ------------------------------------------------------------------------------
//   Helper Function - Open Serial Port File Descriptor
// ------------------------------------------------------------------------------
// Where the actual port opening happens, returns file descriptor 'fd'
int
Serial_Port::
_open_port(const char* port)
{
    // Open serial port
    // O_RDWR - Read and write
    // O_NOCTTY - Ignore special chars like CTRL-C
    // O_NDELAY - The open will return without waiting for the device to be ready or available; subsequent behavior of the device is device specific.
    fd = open(port, O_RDWR | O_NOCTTY | O_NDELAY);

    // Check for Errors
    // On failure, open returns -1 and sets errno to identify the error. 
    if (fd == -1)
    {
        /* Could not open the port. */
        return(-1);
    }

    // Finalize
    else
    {   
        // fcntl - manipulate file descriptor
        // F_SETFL - Set the file status flags to the value specified by arg.
        fcntl(fd, F_SETFL, 0);
    }

    // Done!
    return fd;
}

// ------------------------------------------------------------------------------
//   Helper Function - Setup Serial Port
// ------------------------------------------------------------------------------
// Sets configuration, flags, and baud rate
bool
Serial_Port::
_setup_port(int baud, int data_bits, int stop_bits, bool parity, bool hardware_control)
{
    // Check file descriptor
    // isatty - test whether a file descriptor refers to a terminal 
    if(!isatty(fd))
    {
        fprintf(stderr, "\nERROR: file descriptor %d is NOT a serial port\n", fd);
        return false;
    }

    // Read file descritor configuration
    // tcgetattr() gets the parameters associated with the object referred by fd and 
    // stores them in the termios structure referenced by termios_p.
    // Upon successful completion, 0 shall be returned. Otherwise, -1 shall be returned and errno set to indicate the error.
    struct termios  config;
    if(tcgetattr(fd, &config) < 0)
    {
        fprintf(stderr, "\nERROR: could not read configuration of fd %d\n", fd);
        return false;
    }

    // Input flags - Turn off input processing
    // convert break to null byte, no CR to NL translation,
    // no NL to CR translation, don't mark parity errors or breaks
    // no input parity check, don't strip high bit off,
    // no XON/XOFF software flow control
    // & ~BIT ---> 0
    // IGNBRK - Ignore break condition. 
    // BRKINT - Signal interrupt on break. 
    // ICRNL - Map CR to NL on input. 
    // INLCR - Map NL to CR on input. 
    // PARMRK - Mark parity errors. 
    // INPCK - Enable input parity check. 
    // ISTRIP - Strip character 
    // IXON - Enable start/stop output control. 
    config.c_iflag &= ~(IGNBRK | BRKINT | ICRNL |
                        INLCR | PARMRK | INPCK | ISTRIP | IXON);

    // Output flags - Turn off output processing
    // no CR to NL translation, no NL to CR-NL translation,
    // no NL to CR translation, no column 0 CR suppression,
    // no Ctrl-D suppression, no fill characters, no case mapping,
    // no local output processing
    // & ~BIT ---> 0
    // OCRNL - Map CR to NL on output.
    // ONLCR - Map NL to CR-NL on output. 
    // ONLRET - NL performs CR function. 
    // ONOCR - No CR output at column 0.
    // OFILL - Use fill characters for delay. 
    // OPOST - Post-process output 
    config.c_oflag &= ~(OCRNL | ONLCR | ONLRET |
                         ONOCR | OFILL | OPOST);

    #ifdef OLCUC
        // & ~BIT ---> 0
        // If this bit is set, map lower-case to upper-case on output (LEGACY). 
        config.c_oflag &= ~OLCUC;
    #endif

    #ifdef ONOEOT
        // & ~BIT ---> 0
        // If this bit is set, discard C-d characters (code 004) on output.
        // These characters cause many dial-up terminals to disconnect.
        // This bit exists only on BSD systems and GNU/Hurd systems. 
        config.c_oflag &= ~ONOEOT;
    #endif

    // No line processing:
    // echo off, echo newline off, canonical mode off,
    // extended input processing off, signal chars off
    // & ~BIT ---> 0
    // ECHO - Enable echo.  
    // ECHONL - Echo NL.  
    // ICANON - Canonical input (erase and kill processing).  
    // IEXTEN - Enable extended input character processing.  
    // ISIG - Enable signals. 
    config.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);

    // Turn off character processing
    // clear current char size mask, no parity checking,
    // no output processing, force 8 bit input
    config.c_cflag &= ~(CSIZE | PARENB);
    config.c_cflag |= CS8;

    // One input byte is enough to return from read()
    // Inter-character timer off
    // c_cc - control chars
    // VMIN - MIN value 
    // VTIME - TIME value 
    config.c_cc[VMIN]  = 1;
    config.c_cc[VTIME] = 10; // was 0

    // Get the current options for the port
    ////struct termios options;
    ////tcgetattr(fd, &options);

    // Apply baudrate
    switch (baud)
    {
        // The baud rate functions are provided for getting and setting the values of the input and output baud rates in the termios structure.
        // The new values do not take effect until tcsetattr() is successfully called. 
        //
        // cfsetispeed() sets the input baud rate stored in the termios structure to speed,
        // which must be specified as one of the Bnnn constants listed above for cfsetospeed().
        // If the input baud rate is set to zero, the input baud rate will be equal to the output baud rate.
        // Returns the input baud rate stored in the termios structure.
        //
        // cfsetospeed() sets the output baud rate stored in the termios structure pointed to by termios_p to speed.

        case 1200:
            if (cfsetispeed(&config, B1200) < 0 || cfsetospeed(&config, B1200) < 0)
            {
                fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
                return false;
            }
            break;
        case 1800:
            cfsetispeed(&config, B1800);
            cfsetospeed(&config, B1800);
            break;
        case 9600:
            cfsetispeed(&config, B9600);
            cfsetospeed(&config, B9600);
            break;
        case 19200:
            cfsetispeed(&config, B19200);
            cfsetospeed(&config, B19200);
            break;
        case 38400:
            if (cfsetispeed(&config, B38400) < 0 || cfsetospeed(&config, B38400) < 0)
            {
                fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
                return false;
            }
            break;
        case 57600:
            if (cfsetispeed(&config, B57600) < 0 || cfsetospeed(&config, B57600) < 0)
            {
                fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
                return false;
            }
            break;
        case 115200:
            if (cfsetispeed(&config, B115200) < 0 || cfsetospeed(&config, B115200) < 0)
            {
                fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
                return false;
            }
            break;

        // These two non-standard (by the 70'ties ) rates are fully supported on
        // current Debian and Mac OS versions (tested since 2010).
        case 460800:
            if (cfsetispeed(&config, B460800) < 0 || cfsetospeed(&config, B460800) < 0)
            {
                fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
                return false;
            }
            break;
        case 921600:
            if (cfsetispeed(&config, B921600) < 0 || cfsetospeed(&config, B921600) < 0)
            {
                fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
                return false;
            }
            break;
        default:
            fprintf(stderr, "ERROR: Desired baud rate %d could not be set, aborting.\n", baud);
            return false;

            break;
    }

    // Finally, apply the configuration
    // tcsetattr() sets the parameters associated with the terminal (unless support is required from the
    // underlying hardware that is not available) from the termios structure referred
    // to by termios_p. optional_actions specifies when the changes take effect:
    // TCSAFLUSH
    // the change occurs after all output written to the object referred by fd has been transmitted,
    // and all input that has been received but not read will be discarded before the change is made. 
    if(tcsetattr(fd, TCSAFLUSH, &config) < 0)
    {
        fprintf(stderr, "\nERROR: could not set configuration of fd %d\n", fd);
        return false;
    }

    // Done!
    return true;
}



// ------------------------------------------------------------------------------
//   Read Port with Lock
// ------------------------------------------------------------------------------
int
Serial_Port::
_read_port(uint8_t &cp)
{

    // Lock
    pthread_mutex_lock(&lock);

    // read() attempts to read up to count bytes from file descriptor fd into the buffer starting at buf. 
    // ssize_t read(int fd, void *buf, size_t count);
    int result = read(fd, &cp, 1);

    // Unlock
    pthread_mutex_unlock(&lock);

    return result;
}


// ------------------------------------------------------------------------------
//   Write Port with Lock
// ------------------------------------------------------------------------------
void
Serial_Port::
_write_port(char *buf, unsigned &len)
{

    // Lock
    pthread_mutex_lock(&lock);

    // Write packet via serial link
    write(fd, buf, len);

    // Wait until all data has been written
    tcdrain(fd);

    // Unlock
    pthread_mutex_unlock(&lock);

    return;
}