// Biblioteca de funciones auxiliares

#include <stdio.h>
#include <string.h>
extern UART_HandleTypeDef huart2;

void espera(int tiempo);
void Bin2Ascii(unsigned short numero, unsigned char* cadena);
//int _write(int file, char *ptr, int len);

int _write(int file, char *ptr, int len) {
    int i = 0;
    for ( i = 0; i < len; i++)
        HAL_UART_Transmit(&huart2,ptr++,1,1000);
    return len;
}

long map(long val, long in_min, long in_max, long out_min, long out_max);
