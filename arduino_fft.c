#include <errno.h>
#include <fcntl.h> 
#include <string.h>
#include <termios.h>
#include <termios.h>
#include <unistd.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <complex.h>
#include <fftw3.h>

int
set_interface_attribs (int fd, int speed, int parity)
{
        struct termios tty;
        memset (&tty, 0, sizeof tty);
        if (tcgetattr (fd, &tty) != 0)
        {
                printf ("error %d from tcgetattr", errno);
                return -1;
        }

        cfsetospeed (&tty, speed);
        cfsetispeed (&tty, speed);

        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
        // disable IGNBRK for mismatched speed tests; otherwise receive break
        // as \000 chars
        tty.c_iflag &= ~IGNBRK;         // disable break processing
        tty.c_lflag = 0;                // no signaling chars, no echo,
                                        // no canonical processing
        tty.c_oflag = 0;                // no remapping, no delays
        tty.c_cc[VMIN]  = 0;            // read doesn't block
        tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

        tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

        tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
                                        // enable reading
        tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
        tty.c_cflag |= parity;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;

        if (tcsetattr (fd, TCSANOW, &tty) != 0)
        {
                printf ("error %d from tcsetattr", errno);
                return -1;
        }
        return 0;
}

void
set_blocking (int fd, int should_block)
{
        struct termios tty;
        memset (&tty, 0, sizeof tty);
        if (tcgetattr (fd, &tty) != 0)
        {
                printf ("error %d from tggetattr", errno);
                return;
        }

        tty.c_cc[VMIN]  = should_block ? 1 : 0;
        tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

        if (tcsetattr (fd, TCSANOW, &tty) != 0)
                printf ("error %d setting term attributes", errno);
}

#define FFT_SIZE 131072
#define sq(a) ((a)*(a))
double fft_input[FFT_SIZE*2];
int main()
{
	char *portname = "/dev/ttyACM0";
	int fd = open (portname, O_RDWR | O_NOCTTY | O_SYNC);
	if (fd < 0)
	{
		    printf ("error %d opening %s: %s", errno, portname, strerror (errno));
		    return 0;
	}

	set_interface_attribs (fd, B115200, 0);  // set speed to 115,200 bps, 8n1 (no parity)
	set_blocking (fd, 0);                // set no blocking

	uint16_t buf [FFT_SIZE];
	
	fftw_complex *out, *in;
	out = (fftw_complex *)fftw_malloc(sizeof(fftw_complex) * FFT_SIZE);
	in = (fftw_complex *)fftw_malloc(sizeof(fftw_complex) * FFT_SIZE);
	 
	// Initialize 'in' with N complex entries
	
	fftw_plan my_plan;
	my_plan = fftw_plan_dft_1d(FFT_SIZE, in, out, FFTW_FORWARD, FFTW_ESTIMATE);
	
	while(1){
	int n = read (fd, buf, sizeof buf);  // read up to 100 characters if ready to read
	for(int i = 0;i<FFT_SIZE;i++)
	{
		fft_input[i] = (double)buf[i];
		in[i] = buf[i];
	}
	fftw_execute(my_plan);
	for(int i = 0;i<FFT_SIZE;i++)
	{
		printf("%.0lf\n", FFT_SIZE-i-1, sqrt(sq(creal(out[i])) + sq(cimag(out[i]))) / FFT_SIZE);
	}
	}
	// Use 'out' for something
	 
	fftw_destroy_plan(my_plan);
	fftw_free(out);
	return 0;
}
