#include "DigitalOut.h"
#include "PinNames.h"
#include "mbed.h"
#include <math.h>
#include "mbed_error.h"
#include "mbed_wait_api.h"
#include "hcsr04.h"
#include "nRF24L01P.h"

Serial pc(USBTX, USBRX);


nRF24L01P my_nrf24l01p(PTD2, PTD3, PTC5, PTD0, PTD5, PTA13);     // mosi, miso, sck, csn, ce, irq

DigitalOut myled1(LED_GREEN);
DigitalOut myled2(LED_RED);


//Motor ponte H
PwmOut right_motor_backward(PTE20);
PwmOut right_motor_forward(PTE21); 
PwmOut left_motor_backward(PTE29); 
PwmOut left_motor_forward(PTE31); 

//HCSR04
DigitalOut echo(PTA4);
DigitalOut trigger(PTA5);
HCSR04 sonar(PTA5, PTA4);
unsigned int dist;

//Encoder; 20 furos no disco
//Cada volta completa dá 20 pulsos (um pulso por furo)(pulso: 0->1)
InterruptIn encoder_right(PTA12);
InterruptIn encoder_left(PTD4);
volatile int right_pulses = 0;
volatile int left_pulses = 0;
void reset_pulses() {
    right_pulses = 0; left_pulses = 0;
}
//as funções para os contadores são necessárias por causa da lógica de acionamento dos pinos de interrupt
void count_right() {
    right_pulses++;
}
void count_left() {
    left_pulses++;
}
const long double pi = atan(1)*4;
const long double dist_pulse = (2*pi*3.15)/20;


void setMotor(float left_forward, float left_backward, float right_forward, float right_backward) {
    right_motor_backward = right_backward;
    pc.printf("Motor direito pra trás: %f \r\n", right_backward);
    right_motor_forward = right_forward;   
    pc.printf("Motor direito pra frente: %f \r\n", right_forward);
    left_motor_backward = left_backward;
    pc.printf("Motor esquerdo pra trás: %f \r\n", left_backward);   
    left_motor_forward = left_forward;
    pc.printf("Motor esquerdo pra frente: %f \r\n", left_forward);
    right_motor_backward.period_ms(25);
    right_motor_forward.period_ms(25);
    left_motor_backward.period_ms(25);
    left_motor_forward.period_ms(25);   
}

void move_forward(int dist_cm){
    int pulses = floor(dist_cm/(1/dist_pulse));
    reset_pulses();
    while (right_pulses <= pulses && left_pulses <= pulses) {
        setMotor(0, 1, 0, 1);
        pc.printf("Andando %d cm pra frente", dist_cm);
        pc.printf("\r\n");
        pc.printf("Pulsos: %d", pulses);
        pc.printf("\r\n");
        pc.printf("Encoder direita: %d", right_pulses);
        pc.printf("\r\n");
        pc.printf("Encoder esquerda: %d", left_pulses);
        pc.printf("\r\n");
    }
    setMotor(0, 0, 0, 0);
    reset_pulses();
}

void move_left() { //90 graus
    int pulses = 6; // 8 pulsos para 90 graus
    reset_pulses();
    while (left_pulses < pulses) {
        setMotor(1, 0, 0, 1);
        pc.printf("Virando pra esquerda");
        pc.printf("\r\n");
        pc.printf("Arco: %d", pulses);
        pc.printf("\r\n");
        pc.printf("Encoder direita: %d", right_pulses);
        pc.printf("\r\n");
        pc.printf("Encoder esquerda: %d", left_pulses);
        pc.printf("\r\n");
    }
    setMotor(0, 0, 0, 0);
    reset_pulses();
}

void move_right() { //90 graus
    int pulses = 6; // 8 pulsos para 90 graus
    reset_pulses();
    while (left_pulses < pulses) {
        setMotor(0, 1, 1, 0);
        pc.printf("Virando pra direita");
        pc.printf("\r\n");
        pc.printf("Arco: %d", pulses);
        pc.printf("\r\n");
        pc.printf("Encoder direita: %d", right_pulses);
        pc.printf("\r\n");
        pc.printf("Encoder esquerda: %d", left_pulses);
        pc.printf("\r\n");
    }
    setMotor(0, 0, 0, 0);
    reset_pulses();
}

void move_backwards() { //5cm pra trás
    int pulses = floor(5/(1/dist_pulse));
    reset_pulses();
    setMotor(1, 0, 1, 0);
    while (right_pulses <= pulses || left_pulses <= pulses) {
        pc.printf("Andando pra trás");
        pc.printf("\r\n");
        pc.printf("Pulsos: %d", pulses);
        pc.printf("\r\n");
        pc.printf("Encoder direita: %d", right_pulses);
        pc.printf("\r\n");
        pc.printf("Encoder esquerda: %d", left_pulses);
        pc.printf("\r\n");
    }
    setMotor(0, 0, 0, 0);
    reset_pulses();
}

void destination(int x, int y, bool positive) {
    int y_tvl = y; int x_tvl = x;
    while (y_tvl >= 0 && x_tvl >= 0) {
        pc.printf("Entrando no loop base \r\n");
        while (y_tvl >= 0) {
            pc.printf("Entrando no loop Y \r\n");
            dist = sonar.get_dist_cm();
            pc.printf("Distancia: %d", dist);
            if (dist > 2 || dist <= 0) { //CHECAR SE O CARRINHO DE FATO ANDA SÓ 1CM
                pc.printf("Andando pra frende 1 cm \r\n");
                move_forward(1); //anda de um em um cm
                y_tvl--;
                pc.printf("Faltam %d cms em Y \r\n", y_tvl);
            } else {
                pc.printf("Oh não, achamos um obstáculo!!! \r\n");
                setMotor(0, 0, 0, 0);
                wait_us(1000000);
                if (positive) {
                    pc.printf("Girando para a direita e andando 1 cm");
                    move_right();
                    wait_us(1000000);
                    move_forward(1);
                    x_tvl--;
                    wait_us(1000000);
                    move_left();
                    wait_us(1000000);
                } else if (!positive) {
                    pc.printf("Girando para a esquerda e andando 1 cm");
                    move_left();
                    wait_us(1000000);
                    move_forward(1);
                    x_tvl--;
                    wait_us(1000000);
                    move_right();
                    wait_us(1000000);
                }
            }
        }
        wait_us(1000000);
        if (positive) {
            pc.printf("Girando para a direita e preparando para percorrer X");
            move_right();
        } else if (!positive) {
            move_left();
        }
        wait_us(1000000);
        while (x_tvl >= 0) {
            pc.printf("Entrando no loop X");
            dist = sonar.get_dist_cm();
            if (dist > 2 || dist <= 0) {
                pc.printf("Andando pra frente 1 cm");
                move_forward(1); //anda de um em um cm
                x_tvl--;
                pc.printf("Faltam %d cm em X", x_tvl);
            } else {
                setMotor(0, 0, 0, 0);
                wait_us(1000000);
                if (positive) {
                    pc.printf("Girando para a direita e andando 1 cm");
                    move_right();
                    wait_us(1000000);
                    move_forward(1);
                    y_tvl++;
                    wait_us(1000000);
                    move_left();
                    wait_us(1000000);
                } else if (!positive) {
                    pc.printf("Girando para a esquerda e andando 1 cm");
                    move_left();
                    wait_us(1000000);
                    move_forward(1);
                    y_tvl++;
                    wait_us(1000000);
                    move_right();
                    wait_us(1000000);
                }
            }
        }
    }
}

// main() runs in its own thread in the OS
int main()
{
    #define TRANSFER_SIZE   5 //XX YY Positivo

    myled1 = 0;
    myled2 = 1;

    encoder_right.fall(&count_right);
    encoder_left.fall(&count_left);

    char txData[TRANSFER_SIZE], rxData[TRANSFER_SIZE];
    int txDataCnt = 0;
    int rxDataCnt = 0;

    my_nrf24l01p.powerUp();

    // Display the (default) setup of the nRF24L01+ chip
    pc.printf( "nRF24L01+ Frequency    : %d MHz\r\n",  my_nrf24l01p.getRfFrequency() );
    pc.printf( "nRF24L01+ Output power : %d dBm\r\n",  my_nrf24l01p.getRfOutputPower() );
    pc.printf( "nRF24L01+ Data Rate    : %d kbps\r\n", my_nrf24l01p.getAirDataRate() );
    pc.printf( "nRF24L01+ TX Address   : 0x%010llX\r\n", my_nrf24l01p.getTxAddress() );
    pc.printf( "nRF24L01+ RX Address   : 0x%010llX\r\n", my_nrf24l01p.getRxAddress() );

    pc.printf( "Type keys to test transfers:\r\n  (transfers are grouped into %d characters)\r\n", TRANSFER_SIZE );

    my_nrf24l01p.setTransferSize(TRANSFER_SIZE);

    my_nrf24l01p.setReceiveMode();
    my_nrf24l01p.enable();

    pc.printf("Ready to roll... \r\n");

    while (1) {

        // If we've received anything over the host serial link...
        if ( pc.readable() ) {

            // ...add it to the transmit buffer
            txData[txDataCnt++] = pc.getc();

            // If the transmit buffer is full
            if ( txDataCnt >= sizeof( txData ) ) {

                // Send the transmitbuffer via the nRF24L01+
                my_nrf24l01p.write( NRF24L01P_PIPE_P0, txData, txDataCnt );

                txDataCnt = 0;
            }

            // Toggle LED1 (to help debug Host -> nRF24L01+ communication)
           myled2 = !myled2;
        }

        // If we've received anything in the nRF24L01+...
        if ( my_nrf24l01p.readable() ) {

            // ...read the data into the receive buffer
            rxDataCnt = my_nrf24l01p.read( NRF24L01P_PIPE_P0, rxData, sizeof( rxData ) );
            
            for ( int i = 0; rxDataCnt > 0; rxDataCnt--, i++ ) {

                pc.putc( rxData[i] );
            }

            pc.printf("\r\n");

            myled1 = !myled1;

            int x_coord = 0; int y_coord = 0; bool positive = true;
            if (rxData[0] > 47 && rxData[0] < 58 && rxData[1] > 47 && rxData[1] < 58) {
                x_coord = (rxData[0]-48)*10 + (rxData[1]-48);
                pc.printf("Coordenada X: %d \r\n", x_coord);
            }
            if (rxData[2] > 47 && rxData[2] < 58 && rxData[3] > 47 && rxData[3] < 58) {
                y_coord = (rxData[2]-48)*10 + (rxData[3]-48);
                pc.printf("Coordenada Y: %d \r\n", y_coord);
            }
            switch (rxData[4]) {
                case 'p': positive = true; break;
                case 'n': positive = false; break;
            }
            pc.printf("Positivo: %d \r\n", positive);
            /*move_forward(10);
            wait_us(1000000);
            move_left();
            wait_us(1000000);
            move_right();*/
            destination(x_coord, y_coord, positive);

            /*switch (rxData[0]) {
                case 'e': //pra esquerda
                    move_left();
                    break;
                case 'd': //pra direita
                    move_right();
                    break;
                case 'f': //pra frente
                    move_forward(rxData[1]);
                    break;
            }*/
            // Display the receive buffer contents via the host serial link

            // Toggle LED2 (to help debug nRF24L01+ -> Host communication)
        }
    }
}

