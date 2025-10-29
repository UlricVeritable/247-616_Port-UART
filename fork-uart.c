/**
 * @file    SerialPort_write.c
 * 
 * @brief Serial Port Programming in C (Serial Port Write)  
 * Non Cannonical mode 
 * Sellecting the Serial port Number on Linux   
 * /dev/ttyUSBx - when using USB  to Serial Converter, where x can be 0,1,2...etc  
 * /dev/ttySx   - for PC hardware based Serial ports, where x can be 0,1,2...etc 
 * termios structure -  /usr/include/asm-generic/termbits.h  
 * use "man termios" to get more info about  termios structure 
 * @author  Kevin Cotton
 * @date    2024-08-02
 */
#define _GNU_SOURCE

#include <stdio.h>
#include <fcntl.h>   // File Control Definitions
#include <termios.h> // POSIX Terminal Control Definitions
#include <unistd.h>  // UNIX Standard Definitions
#include <errno.h>   // ERROR Number Definitions

// device port série à utiliser 
//const char *portTTY = "/dev/ttyGS0"; 
//const char *portTTY = "/dev/ttyS0";
//const char *portTTY = "/dev/ttyS1";
//const char *portTTY = "/dev/ttyS2";
//const char *portTTY = "/dev/ttyS3";
//const char *portTTY = "/dev/ttyS4";
//const char *portTTY = "/dev/ttyS5";
const char *portTTY = "/dev/ttyUSB0"; // ttyUSB0 is the FT232 based USB2SERIAL Converter

void SerialPortInit(int fd)
{

	// Opening the Serial Port 
	fd = open(portTTY, O_RDWR | O_NOCTTY | O_NDELAY);
								// O_RDWR Read/Write access to serial port           
								// O_NOCTTY - No terminal will control the process   
								// O_NDELAY -Non Blocking Mode,Does not care about-  
								// -the status of DCD line, Open() returns immediatly                                        
	if(fd == -1) // Error Checking
		printf("\n Erreur! ouverture de %s ", portTTY);
	else
		printf("\n Ouverture de %s reussit ", portTTY);

	// Setting the Attributes of the serial port using termios structure 
	struct termios SerialPortSettings;	// Create the structure 
	tcgetattr(fd, &SerialPortSettings);	// Get the current attributes of the Serial port
	// Setting the Baud rate
	cfsetispeed(&SerialPortSettings, B115200); // Set Read  Speed   
	cfsetospeed(&SerialPortSettings, B115200); // Set Write Speed  
	// 8N1 Mode 
	SerialPortSettings.c_cflag &= ~PARENB;   // Disables the Parity Enable bit(PARENB),So No Parity
	SerialPortSettings.c_cflag &= ~CSTOPB;   // CSTOPB = 2 Stop bits,here it is cleared so 1 Stop bit
	SerialPortSettings.c_cflag &= ~CSIZE;	 // Clears the mask for setting the data size
	SerialPortSettings.c_cflag |=  CS8;      //Set the data bits = 8
	SerialPortSettings.c_cflag &= ~CRTSCTS;       // No Hardware flow Control
	SerialPortSettings.c_cflag |= CREAD | CLOCAL; // Enable receiver, Ignore Modem Control lines 

	SerialPortSettings.c_iflag &= ~(IXON | IXOFF | IXANY);	// Disable XON/XOFF flow control both i/p and o/p

	SerialPortSettings.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);  // Non Cannonical mode, Disable echo, Disable signal  

	SerialPortSettings.c_oflag &= ~OPOST;	//No Output Processing

	// Setting Time outs 
	SerialPortSettings.c_cc[VMIN] = 1; // Read at least X character(s) 
	SerialPortSettings.c_cc[VTIME] = 0; // Wait 3sec (0 for indefinetly) 


	if((tcsetattr(fd, TCSANOW, &SerialPortSettings)) != 0) // Set the attributes to the termios structure
		printf("\n  Erreur! configuration des attributs du port serie");

}

void main(void)
	{
	int fd; //File Descriptor
	pid_t pid;
	
	printf("\n Ecriture Port Serie");
	SerialPortInit(fd);
	
	pid = fork();	// On fork{}
	if (pid == 0)	//Si 0, alors processus fils
	{
		//Processus fils ici, lecture terminal et ecriture sur SP
		printf("Je suis le processus Fils, j'ecris sur le port serie ce que j'entends ur la console (Terminal)...");
		// Write data to serial port 
		char write_buffer[] = "ABCDE12345";	// Buffer containing characters to write into port
		int  bytes_written  = 0;  	// Value for storing the number of bytes written to the port 

		bytes_written = write(fd, write_buffer, sizeof(write_buffer)); // use write() to send data to port 
											// "fd"                   - file descriptor pointing to the opened serial port
											//	"write_buffer"         - address of the buffer containing data
											// "sizeof(write_buffer)" - No of bytes to write 
		printf("\n Ecriture de %d octets : %s ecrit sur le port %s", bytes_written, write_buffer, portTTY);
		printf("\n");

		close(fd); // Close the Serial port 
	}
	else if (pid>0)	//Si retour processus parent
	{
	//Processus parent, lecture du SP et l<envoyer sur terminal
		printf("Je suis le programme Pere, j'ecris sur la console (terminal) ce que j'entend sur le port serie...");

		// Read data from serial port 
		tcflush(fd, TCIFLUSH);  // Discards old data in the rx buffer
		char read_buffer[32];   // Buffer to store the data received 
		int  bytes_read = 0;    // Number of bytes read by the read() system call 
		int i = 0;

		bytes_read = read(fd, &read_buffer, 32); // Read the data 
		//Ici pour voir si recu  caract interdit, car VMIN 1 alors juste bufer 1

		printf(" Bytes Recus : %d --> ", bytes_read); // Print the number of bytes read
		for(i=0; i<bytes_read; i++)	 // printing only the received characters
			printf("%c", read_buffer[i]);

		printf("\n");

		close(fd);  //Close the serial port
		wait(NULL);	//Attend deuxieme fils

	}
	else
	{	//Erreur au premier fork
		perror("Erreur au premeir fork");
	}

	return 0;
	}
