/* run $ gcc -shared -Wl,-soname,control -o control.so -fPIC control.c */
#include <stdio.h>
#include <fcntl.h>   /* File Control Definitions           */
#include <termios.h> /* POSIX Terminal Control Definitions */
#include <unistd.h>  /* UNIX Standard Definitions 	   */ 
#include <errno.h>   /* ERROR Number Definitions           */
#include <string.h>
// #include <termbits.h>
char read_buffer[100];
float plane_info[10];

// write to serial
void serial_write(unsigned char* TxPack_Buffer, int length)
{
    int fd;  /*File Descriptor*/

    printf("\n +----------------------------------+");
    printf("\n |        Serial Port Write         |");
    printf("\n +----------------------------------+");

    /*------------------------------- Opening the Serial Port -------------------------------*/

    /* Change /dev/ttyUSB0 to the one corresponding to your system */

        fd = open("/dev/ttyUSB0",O_RDWR | O_NOCTTY | O_NDELAY | O_NONBLOCK);	/* ttyUSB0 is the FT232 based USB2SERIAL Converter   */
                                /* O_RDWR Read/Write access to serial port           */
                                /* O_NOCTTY - No terminal will control the process   */
                                /* O_NDELAY -Non Blocking Mode,Does not care about-  */
                                /* -the status of DCD line,Open() returns immediatly */                                        
                                
        if(fd == -1)						/* Error Checking */
                printf("\n  Error! in Opening ttyUSB0  ");
        else
                printf("\n  ttyUSB0 Opened Successfully ");


    /*---------- Setting the Attributes of the serial port using termios structure --------- */

    struct termios SerialPortSettings;	/* Create the structure                          */

    tcgetattr(fd, &SerialPortSettings);	/* Get the current attributes of the Serial port */

    cfsetispeed(&SerialPortSettings,B57600); /* Set Read  Speed as 9600                       */
    cfsetospeed(&SerialPortSettings,B57600); /* Set Write Speed as 9600                       */

    SerialPortSettings.c_cflag &= ~PARENB;   /* Disables the Parity Enable bit(PARENB),So No Parity   */
    SerialPortSettings.c_cflag &= ~CSTOPB;   /* CSTOPB = 2 Stop bits,here it is cleared so 1 Stop bit */
    SerialPortSettings.c_cflag &= ~CSIZE;	 /* Clears the mask for setting the data size             */
    SerialPortSettings.c_cflag |=  CS8;      /* Set the data bits = 8                                 */

    SerialPortSettings.c_cflag &= ~CRTSCTS;       /* No Hardware flow Control                         */
    SerialPortSettings.c_cflag |= CREAD | CLOCAL; /* Enable receiver,Ignore Modem Control lines       */ 


    SerialPortSettings.c_iflag &= ~(IXON | IXOFF | IXANY);          /* Disable XON/XOFF flow control both i/p and o/p */
    SerialPortSettings.c_iflag &= ~(ICANON | ECHO | ECHOE | ISIG);  /* Non Cannonical mode                            */

    SerialPortSettings.c_oflag &= ~OPOST;/*No Output Processing*/

    if((tcsetattr(fd,TCSANOW,&SerialPortSettings)) != 0) /* Set the attributes to the termios structure*/
        printf("\n  ERROR ! in Setting attributes\n");
    else
        printf("\n  BaudRate = 57600 \n  StopBits = 1 \n  Parity   = none");
        
        /*------------------------------- Write data to serial port -----------------------------*/

    // char write_buffer[] = "ABCDEFG HIJK";	/* Buffer containing characters to write into port	     */	
    int  bytes_written  = 0;  	/* Value for storing the number of bytes written to the port */ 

    bytes_written = write(fd, TxPack_Buffer, length);/* use write() to send data to port                                            */
                                        /* "fd"                   - file descriptor pointing to the opened serial port */
                                        /*	"write_buffer"         - address of the buffer containing data	           */
                                        /* "sizeof(write_buffer)" - No of bytes to write                               */

    // printf("\n  %s written to ttyUSB0",write_buffer);
    // printf("\n  %d Bytes written to ttyUSB0", bytes_written);
    // printf("\n +----------------------------------+\n\n");

    close(fd);  /* Close the Serial port */
}

// read from serial
void serial_read()
{
    int fd;/*File Descriptor*/

    printf("\n +----------------------------------+");
    printf("\n |        Serial Port Read          |");
    printf("\n +----------------------------------+");

    /*------------------------------- Opening the Serial Port -------------------------------*/

    /* Change /dev/ttyUSB0 to the one corresponding to your system */

        fd = open("/dev/ttyUSB0",O_RDWR | O_NOCTTY);	/* ttyUSB0 is the FT232 based USB2SERIAL Converter   */
                            /* O_RDWR   - Read/Write access to serial port       */
                            /* O_NOCTTY - No terminal will control the process   */
                            /* Open in blocking mode,read will wait              */



        if(fd == -1)						/* Error Checking */
                printf("\n  Error! in Opening ttyUSB0  ");
        else
                printf("\n  ttyUSB0 Opened Successfully ");


    /*---------- Setting the Attributes of the serial port using termios structure --------- */

    struct termios SerialPortSettings;	/* Create the structure                          */

    tcgetattr(fd, &SerialPortSettings);	/* Get the current attributes of the Serial port */

    /* Setting the Baud rate */
    cfsetispeed(&SerialPortSettings,B57600); /* Set Read  Speed as 57600                      */
    cfsetospeed(&SerialPortSettings,B57600); /* Set Write Speed as 57600                       */

    /* 8N1 Mode */
    SerialPortSettings.c_cflag &= ~PARENB;   /* Disables the Parity Enable bit(PARENB),So No Parity   */
    SerialPortSettings.c_cflag &= ~CSTOPB;   /* CSTOPB = 2 Stop bits,here it is cleared so 1 Stop bit */
    SerialPortSettings.c_cflag &= ~CSIZE;	 /* Clears the mask for setting the data size             */
    SerialPortSettings.c_cflag |=  CS8;      /* Set the data bits = 8                                 */

    SerialPortSettings.c_cflag &= ~CRTSCTS;       /* No Hardware flow Control                         */
    SerialPortSettings.c_cflag |= CREAD | CLOCAL; /* Enable receiver,Ignore Modem Control lines       */


    SerialPortSettings.c_iflag &= ~(IXON | IXOFF | IXANY);          /* Disable XON/XOFF flow control both i/p and o/p */
    SerialPortSettings.c_iflag &= ~(ICANON | ECHO | ECHOE | ISIG);  /* Non Cannonical mode                            */

    SerialPortSettings.c_oflag &= ~OPOST;/*No Output Processing*/

    /* Setting Time outs */
    SerialPortSettings.c_cc[VMIN] = 10; /* Read at least 10 characters */
    SerialPortSettings.c_cc[VTIME] = 0; /* Wait indefinetly   */


    if((tcsetattr(fd,TCSANOW,&SerialPortSettings)) != 0) /* Set the attributes to the termios structure*/
        printf("\n  ERROR ! in Setting attributes");
    else
                printf("\n  BaudRate = 57600 \n  StopBits = 1 \n  Parity   = none");

        /*------------------------------- Read data from serial port -----------------------------*/

    tcflush(fd, TCIFLUSH);   /* Discards old data in the rx buffer            */

    // char read_buffer[100];   /* Buffer to store the data received              */
    int  bytes_read = 0;    /* Number of bytes read by the read() system call */
    int i = 0;

    bytes_read = read(fd, &read_buffer, 100); /* Read the data                   */

    printf("\n  Bytes Rxed %d", bytes_read); /* Print the number of bytes read */
    printf("\n  ");

    for(i = 0; i < bytes_read; i++)	 /*printing only the received characters*/
        printf("%c\t", read_buffer[i]);

    printf("\n +----------------------------------+\n");

    close(fd); /* Close the serial port */
    // return read_buffer;
}

// double -> bytes
void double_to_bytes(unsigned char* Buffer, double tag, int index)
{
	unsigned char a[8] = {0};
	int i = 0;
	memcpy(&a, &tag, 8);
	for(i; i < 8; i++)
	{
		Buffer[i + index] = a[7 - i];
	}
}

// float -> bytes
void float_to_bytes(unsigned char* Buffer, float tag, int index)
{
	unsigned char a[4] = {0};
	int i = 0;
	memcpy(&a, &tag, 4);
	for(i; i < 4; i++)
	{
		Buffer[i + index] = a[3 - i];
	}
}

// bytes -> float
void bytes_to_float(unsigned char* Buffer, float* plane_info)
{
    unsigned char tmp[4];
    int index = 0;
    for (int i = 4; i <= 40; i+=4)
    {
        float f;
        for (int j = 0; j < 4; j++)
        {
            tmp[j] = Buffer[i + j];
        }
        memcpy(&f, &tmp, 4);
        // printf("%.2f\t", f);
        plane_info[index++] = f;
    }
}

// function id: 33 (main control)
void function_33(unsigned short Pkg_Num, unsigned char Control_Mode, 
                 float Offset_PosX, float Offset_PosY, float Height, float Yaw)
{
	unsigned char TxPack_Buffer[51] = {0};
    unsigned short Serial_Num = 0;
	// unsigned char Control_Mode = 4, Nav_Mode = 1;
	unsigned char Nav_Mode = 1;
	double APIC_Lat = 0, APIC_Lon = 0;
	float APIC_PosX = 1000000, APIC_PosY = 1000000;
    unsigned char CK_A, CK_B;
    TxPack_Buffer[0] = 0xAA; // Frame header
    TxPack_Buffer[1] = 44; // Load length
    TxPack_Buffer[2] = Pkg_Num  ; // Packet sequence
    TxPack_Buffer[3] = 33; // Function word

    TxPack_Buffer[4] = Serial_Num >> 8; // Controlled object serial number
    TxPack_Buffer[5] = Serial_Num & 0xff;

    TxPack_Buffer[6] = Control_Mode; // Control mode
    TxPack_Buffer[7] = Nav_Mode; // Navigation mode
    double_to_bytes(TxPack_Buffer, APIC_Lat, 8); // Target latitude
    double_to_bytes(TxPack_Buffer, APIC_Lon, 16); // Target longitude
    float_to_bytes(TxPack_Buffer, APIC_PosX, 24); // Target location(North)
    float_to_bytes(TxPack_Buffer, APIC_PosY, 28); // Target location(East)
    float_to_bytes(TxPack_Buffer, Offset_PosX, 32); // Position offset(Roll)
    float_to_bytes(TxPack_Buffer, Offset_PosY, 36); // Position offset(Pitch)
    float_to_bytes(TxPack_Buffer, Height, 40); // Target height
    float_to_bytes(TxPack_Buffer, Yaw, 44); // Target heading
    // Calculate the 16 bit checksum
    CK_A = 0, CK_B = 0;
    unsigned short i = 0;
    for(i; i < 48; i++)
    {
        CK_A = CK_A + TxPack_Buffer[i];
        CK_B = CK_B + CK_A;
    }
    TxPack_Buffer[48] = CK_A;
    TxPack_Buffer[49] = CK_B; // Serial port sending
    serial_write(TxPack_Buffer, 51);

    for(i = 0; i < 51; i++)
	{
        printf("%02x ", TxPack_Buffer[i]);
        if ((i + 1) % 10 == 0)
            printf("\n");
    }	
}

// function id: 32 (get plane parameters)
float *function_32(unsigned short Pkg_Num, unsigned char Read_Mode)
{
	unsigned char TxPack_Buffer[8] = {0};

    unsigned char CK_A, CK_B;
    TxPack_Buffer[0] = 0xAA; // Frame header
    TxPack_Buffer[1] = 1; // Load length
    TxPack_Buffer[2] = Pkg_Num; // Packet sequence
    TxPack_Buffer[3] = 32; // Function word
    TxPack_Buffer[4] = Read_Mode; //return time

    CK_A = 0, CK_B = 0;
    unsigned short i = 0;
    for(i; i < 5; i++)
    {
        CK_A = CK_A + TxPack_Buffer[i];
        CK_B = CK_B + CK_A;
    }
    TxPack_Buffer[5] = CK_A;
    TxPack_Buffer[6] = CK_B; // Serial port sending
    serial_write(TxPack_Buffer, 8);

    for(i = 0; i < 8; i++)
		printf("%02x ", TxPack_Buffer[i]);
	serial_read();
    bytes_to_float(read_buffer, plane_info);
    return plane_info;
}

// function id: 31 (gestures control)
void function_31(unsigned short Pkg_Num, short Pitch, short Roll, 
                 short Yaw, short Thrust, short X1, short X2)
{
	unsigned char TxPack_Buffer[19] = {0};
	// short Pitch=0, Roll=0, Yaw = 0, Thrust=1000, X1=0, X2=0;
    unsigned char CK_A, CK_B;
    TxPack_Buffer[0] = 0xAA; // Frame header
    TxPack_Buffer[1] = 12; // Load length
    TxPack_Buffer[2] = Pkg_Num; // Packet sequence
    TxPack_Buffer[3] = 31; // Function word

    TxPack_Buffer[4] = ((unsigned short) Pitch) >> 8;
    TxPack_Buffer[5] = ((unsigned short) Pitch) & 0xff;
    TxPack_Buffer[6] = ((unsigned short) Roll) >> 8;
    TxPack_Buffer[7] = ((unsigned short) Roll) & 0xff;
    TxPack_Buffer[8] = ((unsigned short) Yaw) >> 8;
    TxPack_Buffer[9] = ((unsigned short) Yaw) & 0xff;
    TxPack_Buffer[10] = ((unsigned short) Thrust) >> 8;
    TxPack_Buffer[11] = ((unsigned short) Thrust) & 0xff;
    TxPack_Buffer[12] = ((unsigned short) X1) >> 8;
    TxPack_Buffer[13] = ((unsigned short) X1) & 0xff;
    TxPack_Buffer[14] = ((unsigned short) X2) >> 8;
    TxPack_Buffer[15] = ((unsigned short) X2) & 0xff;

    // Calculate the 16 bit checksum
    CK_A = 0, CK_B = 0;
    unsigned short i = 0;
    for(i; i < 16; i++)
    {
        CK_A = CK_A + TxPack_Buffer[i];
        CK_B = CK_B + CK_A;
    }
    TxPack_Buffer[16] = CK_A;
    TxPack_Buffer[17] = CK_B; // Serial port sending
    serial_write(TxPack_Buffer, 19);

    for(i = 0; i < 19; i++)
    {
        printf("%02x ", TxPack_Buffer[i]);
        if ((i + 1) % 10 == 0)
            printf("\n");
    }
}
