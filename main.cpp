#include "DigitalOut.h"
#include "PinNames.h"
#include "mbed.h"
#include <math.h>
#include "mbed_wait_api.h"
#include "nRF24L01P.h"
#include "HCSR04.h"

float list[4] = {0.06f, 0.06f, 0.06f, 0.06f};

Serial pc(USBTX, USBRX);

nRF24L01P my_nrf24l01p(PTD2, PTD3, PTC5, PTD0, PTD5, PTA13);    // mosi, miso, sck, csn, ce, irq

DigitalOut myled1(LED1);
DigitalOut myled2(LED2);

//Motor ponte H
DigitalOut right_motor_backward(PTB0);
DigitalOut right_motor_forward(PTB1); 
DigitalOut left_motor_backward(PTC2); 
DigitalOut left_motor_forward(PTC1); 

//HCSR04
DigitalOut echo(PTA4);
DigitalOut trigger(PTA5);
HCSR04 sonar(PTA5, PTA4);
float dist;

//Encoder; 20 furos no disco
//Cada volta completa dá 20 pulsos (um pulso por furo)(pulso: 0->1)
InterruptIn encoder_right(PTA5);
InterruptIn encoder_left(PTA4);
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
const long double dist_pulse = (2*pi*2*6.8)/20;


void setMotor(float left_forward, float left_backward, float right_forward, float right_backward) {
    right_motor_backward = right_backward;
    right_motor_forward = right_forward;   
    left_motor_backward = left_backward;   
    left_motor_forward = left_forward;   
}

void move_forward(int dist_cm){
    int pulses = floor(dist_cm/(1/dist_pulse));
    reset_pulses();
    setMotor(1, 0, 1, 0);
    while (right_pulses <= pulses || left_pulses <= pulses) {
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
    int pulses = 18; // 18 pulsos para 90 graus
    reset_pulses();
    setMotor(0, 1, 1, 0);
    while (left_pulses < pulses) {
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
    int pulses = 18; // 18 pulsos para 90 graus
    reset_pulses();
    setMotor(1, 0, 0, 1);
    while (right_pulses < pulses) {
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
    setMotor(0, 1, 0, 1);
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
    while (y_tvl >= 0) {
        dist = sonar.distance(CM);
        if (dist > 10 || dist <= 0) {
            move_forward(1); //anda de um em um cm
            y_tvl--;
        } else {
            setMotor(0, 0, 0, 0);
            wait_us(1000000);
            if (positive) {
                move_left();
                wait_us(1000000);
                move_forward(1);
                x_tvl--;
                wait_us(1000000);
                move_right();
                wait_us(1000000);
            } else if (!positive) {
                move_right();
                wait_us(1000000);
                move_forward(1);
                x_tvl--;
                wait_us(1000000);
                move_left();
                wait_us(1000000);
            }
        }
    }
}

// main() runs in its own thread in the OS
int main()
{
    #define TRANSFER_SIZE   4 //originalmente 4

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

    my_nrf24l01p.setTransferSize( TRANSFER_SIZE, TRANSFER_SIZE );

    my_nrf24l01p.setReceiveMode();
    my_nrf24l01p.enable();

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
           myled1 = !myled1;
        }

        // If we've received anything in the nRF24L01+...
        if ( my_nrf24l01p.readable() ) {

            // ...read the data into the receive buffer
            rxDataCnt = my_nrf24l01p.read( NRF24L01P_PIPE_P0, rxData, sizeof( rxData ) );
            
            for ( int i = 0; rxDataCnt > 0; rxDataCnt--, i++ ) {

                pc.putc( rxData[i] );
            }

            myled2 = !myled2;

            if (rxData[0] == 'e'){
                move_left();
            } else if (rxData[0] == 'd') {
                move_right();
            } else if (rxData[0] == 'f') {
                move_forward((rxData[1] - '0'));
            }

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

