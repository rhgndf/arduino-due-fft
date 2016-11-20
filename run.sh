gcc arduino_fft.c -o arduino_fft -lfftw3 -lm -g -O2
./arduino_fft | python plot.py
