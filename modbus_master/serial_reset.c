#include <stdio.h>
#include <fcntl.h>       /* File Control Definitions           */
#include <termios.h>     /* POSIX Terminal Control Definitions */
#include <unistd.h>      /* UNIX Standard Definitions          */
#include <errno.h>       /* ERROR Number Definitions           */
#include <sys/ioctl.h>   /* ioctl()                            */

int main(int argc, char* argv[])
{
  int fd;     /*File Descriptor*/
  int status;

  if(argc > 1)
  {
    if((fd = open(argv[1],O_RDWR | O_NOCTTY ))!= -1) //Opening the serial port
    {
      ioctl(fd,TIOCMGET,&status); /* GET the State of MODEM bits in Status */
      status |= TIOCM_DTR;        // Set the RTS pin
      status |= TIOCM_RTS;        // Set the RTS pin
      ioctl(fd, TIOCMBIS, &status);
      ioctl(fd, TIOCMBIC, &status);
      close(fd);
    }else{
      printf("File %s couldn't be opened\n",argv[1]);
    }
  }else {
    printf("Usage: serial_reset <tty>\n");
    printf("  Example: serial_reset /dev/ttyUSB0\n");
  }
  return 0;
}

