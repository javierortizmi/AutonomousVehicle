// Biblioteca de funciones especiales

void espera(int tiempo) {
    int i;
    for(i=0; i<tiempo; i++);
}

void Bin2Ascii(unsigned short numero, unsigned char* cadena) {
    unsigned short parcial, j, cociente, divisor;

    parcial = numero;
    divisor = 10000;
    *(cadena) = ' ';
    for (j = 1; j < 6; j++)
    {
        cociente = parcial/divisor;
        *(cadena+j) = '0'+ (unsigned char)cociente;
        parcial -= (cociente * divisor);
        divisor /= 10;
    }

    *(cadena+6) = 0;

}

long map(long val, long in_min, long in_max, long out_min, long out_max)
{
  return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}