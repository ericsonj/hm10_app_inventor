/* Copyright 2018, Ericson Joseph
 * All rights reserved.
 *
 * This file is part of sAPI Library.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*==================[inlcusiones]============================================*/

#include "sapi.h" // <= Biblioteca sAPI
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

/*==================[definiciones y macros]==================================*/

#define UART_PC UART_USB
#define UART_BLUETOOTH UART_232

/*==================[definiciones de datos internos]=========================*/

// MPU9250 Address
// If MPU9250 AD0 pin is connected to GND
MPU9250_address_t addr = MPU9250_ADDRESS_0;

/*==================[definiciones de datos externos]=========================*/

/*==================[declaraciones de funciones internas]====================*/

/*==================[declaraciones de funciones externas]====================*/

bool_t hm10bleTest(int32_t uart);

/*==================[funcion principal]======================================*/

// FUNCION PRINCIPAL, PUNTO DE ENTRADA AL PROGRAMA LUEGO DE ENCENDIDO O RESET.
int main(void) {
    // ---------- CONFIGURACIONES ------------------------------

    // Inicializar y configurar la plataforma
    boardConfig();

    // Inicializar UART_USB para conectar a la PC
    uartConfig(UART_PC, 115200);
    uartWriteString(UART_PC, "UART_PC configurada.\r\n");

    // Inicializar la IMU
    printf("Inicializando IMU MPU9250...\r\n");
    int8_t status;
    status = mpu9250Init(addr);

    if (status < 0) {
        printf("IMU MPU9250 no inicializado, chequee las conexiones:\r\n\r\n");
        printf("MPU9250 ---- EDU-CIAA-NXP\r\n\r\n");
        printf("    VCC ---- 3.3V\r\n");
        printf("    GND ---- GND\r\n");
        printf("    SCL ---- SCL\r\n");
        printf("    SDA ---- SDA\r\n");
        printf("    AD0 ---- GND\r\n\r\n");
        printf("Se detiene el programa.\r\n");
        while (1)
            ;
    }
    printf("IMU MPU9250 inicializado correctamente.\r\n\r\n");

    // Inicializar UART_232 para conectar al modulo bluetooth
    uartConfig(UART_BLUETOOTH, 115200);
    uartWriteString(UART_PC,
                    "UART_BLUETOOTH para modulo Bluetooth configurada.\r\n");

    uint8_t data = 0;

    uartWriteString(UART_PC,
                    "Testeto si el modulo esta conectado enviando: AT\r\n");
    if (hm10bleTest(UART_BLUETOOTH)) {
        uartWriteString(UART_PC, "Modulo conectado correctamente.\r\n");
    }

    // ---------- REPETIR POR SIEMPRE --------------------------
    while (TRUE) {

        // Leer el sensor y guardar en estructura de control

        mpu9250Read();

        char bufferSend[256];

        float gyroX = mpu9250GetGyroX_rads();
        float gyroY = mpu9250GetGyroY_rads();
        float gyroZ = mpu9250GetGyroZ_rads();

        // Imprimir resultados
        sprintf(bufferSend, "Giroscopo:      (%f|f|%f)   [rad/s]\r\n",
                          gyroX, gyroY, gyroZ); 
        uartWriteString(UART_BLUETOOTH, bufferSend);

        sprintf(bufferSend, "Acelerometro:   (%f|%f|%f)   [m/s2]\r\n",
               mpu9250GetAccelX_mss(),
               mpu9250GetAccelY_mss(),
               mpu9250GetAccelZ_mss());
        uartWriteString(UART_BLUETOOTH, bufferSend);
        
        sprintf(bufferSend, "Magnetometro:   (%f|%f|%f)   [uT]\r\n",
               mpu9250GetMagX_uT(),
               mpu9250GetMagY_uT(),
               mpu9250GetMagZ_uT());
        uartWriteString(UART_BLUETOOTH, bufferSend);
        
        sprintf(bufferSend, "Temperatura:    %f   [C]\r\n\r\n", mpu9250GetTemperature_C());
        uartWriteString(UART_BLUETOOTH, bufferSend);
        
        delay(2000);
    }

    // NO DEBE LLEGAR NUNCA AQUI, debido a que a este programa se ejecuta
    // directamenteno sobre un microcontroladore y no es llamado por ningun
    // Sistema Operativo, como en el caso de un programa para PC.
    return 0;
}

/*==================[definiciones de funciones internas]=====================*/

/*==================[definiciones de funciones externas]=====================*/

bool_t hm10bleTest(int32_t uart) {
    uartWriteString(uart, "AT\r\n");
    return waitForReceiveStringOrTimeoutBlocking(uart, "OK\r\n",
                                                 strlen("OK\r\n"), 50);
}

/*==================[fin del archivo]========================================*/
