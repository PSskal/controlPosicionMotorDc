#include "mcc_generated_files/system/system.h"
#include <stdbool.h>  // 
#include <math.h>
#include <string.h>  // Para usar memset

#define Forward 1
#define Backward 0
#define PWM_MINIMO 0   // Valor mínimo necesario para romper la inercia (ajustar según pruebas)
#define PWM_MIN_DEADZONE 150  // Ajusta según tu motor
/*
    Main application
*/
volatile int16_t theta = 0;
uint8_t pulsadorAnterior; 
uint8_t dir;



int16_t adcValue = 0;
float pwmDuty;

typedef struct {
    float kp;
    float ki;
    float kd;
    float error;
    float errorLast;
    float integral;
    float derivative;
    float output;
    float maxOutput;
    float dt; // Tiempo de muestreo
} PIDController;

PIDController pid;

// Inicializa el controlador PID
void PID_Init(PIDController* pid, float kp, float ki, float kd, float maxOutput, float dt) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->error = 0;
    pid->errorLast = 0;
    pid->integral = 0;
    pid->derivative = 0;
    pid->output = 0;
    pid->maxOutput = maxOutput;
    pid->dt = dt;
}

// Calcula la salida del PID
void PID_Calculate(PIDController* pid, float setpoint, float current) {
    // Calcular el error
    pid->errorLast = pid->error;
    pid->error = setpoint - current;

    // Calcular términos proporcional, integral y derivativo
    float proportional = pid->kp * pid->error;
    pid->integral += pid->ki * pid->error * pid->dt;
    pid->derivative = pid->kd * (pid->error - pid->errorLast) / pid->dt;

    // Limitar la integral
    if (pid->integral > pid->maxOutput) pid->integral = pid->maxOutput;
    else if (pid->integral < -pid->maxOutput) pid->integral = -pid->maxOutput;

    // Calcular la salida total
    pid->output = proportional + pid->integral + pid->derivative;

    // Limitar la salida
    if (pid->output > pid->maxOutput) pid->output = pid->maxOutput;
    else if (pid->output < -pid->maxOutput) pid->output = -pid->maxOutput;
}


void IOC_Initialize(void) {
    // Habilitar interrupciones globales y de periféricos
    INTCON0bits.GIE = 1;

    // Habilitar SOLO flanco de subida para RA1 (Canal A)
    IOCAPbits.IOCAP1 = 1; // Flanco ascendente RA1
    IOCANbits.IOCAN1 = 0; // No flanco descendente RA1

    // Limpiar banderas previas
    IOCAFbits.IOCAF1 = 0;

    // Habilitar interrupción de IOC
    PIE0bits.IOCIE = 1;    // Habilita interrupción IOC
    PIR0bits.IOCIF = 0;    // Limpia bandera global
}
// Función para establecer la velocidad y la dirección del motor
void setMotor(float vel, bool dir) {
    
    PWM1_16BIT_SetSlice1Output1DutyCycleRegister((uint16_t)vel); // Aplicar duty
    PWM1_16BIT_LoadBufferRegisters(); // Cargar en el siguiente ciclo
    // Control de dirección
    // Control de dirección
    if (dir == Forward) {
        M1pin_LAT = 0;
        M2pin_LAT = 1;
    } else if (dir == Backward) {
        M1pin_LAT = 1;
        M2pin_LAT = 0;
    } else {
        M1pin_LAT = 0;
        M2pin_LAT = 0;
    }
}

volatile uint16_t millisContador = 0;  // Para contar hasta 1000 ms
volatile int16_t encoder;

void temporizador_ISR(void) {
    millisContador++;  // Incrementar cada 1 ms
    if (millisContador >= 10) {  
        millisContador = 0;  // Reiniciar conteo         
        encoder = theta;
//        adcValue = ADC_ChannelSelectAndConvert(0);
        // Calcular el PID
        PID_Calculate(&pid, (float)adcValue, (float)encoder);

        // Dirección
        dir = (pid.output > 0) ? Forward : Backward;
        pwmDuty = fabs(pid.output);
        // Aplicar zona muerta
//        if (pwmDuty < PWM_MIN_DEADZONE && pwmDuty != 0.0f) {
//            pwmDuty = PWM_MIN_DEADZONE;
//        }

        // Limitar el PWM
        if (pwmDuty > 800) pwmDuty = 800;

        // Aplicar al motor
        setMotor(pwmDuty, dir);
//        printf("theta: %u  setpoint:%u encoder: %u error: %0.2f error1: %0.2f leyControl: %.2f pwmDuty: %.2f dir: %u \n\r", 
//                  theta, adcValue, encoder,error,error1, leyControl,pwmDuty,dir );
    }
       
    
}

void Uart_Read_String(char* Buf, unsigned int s_buf)
{
    unsigned int cont_buf = 0;
    char c;
    unsigned long timeout = 50000; // número arbitrario para evitar cuelgue

    while (timeout--) {
        if (UART1_IsRxReady()) {
            c = UART1_Read();
            if (c == '\n' || c == '\r') break;

            if (cont_buf < (s_buf - 1)) {
                Buf[cont_buf++] = c;
            }
        }
    }

    Buf[cont_buf] = '\0'; // terminar la cadena
}


void clear_data(char *data, size_t size) {
    memset(data, 0, size);
}

int main(void)  
{
    SYSTEM_Initialize();
    IOC_Initialize();
    
        // Habilita interrupciones globales y periféricas
    INTERRUPT_GlobalInterruptEnable();
     //Registra la función que se ejecutará cada vez que TMR0 interrumpa
    TMR0_OverflowCallbackRegister(temporizador_ISR);
        // ? ¡Aquí inicia el TMR0!
    TMR0_Start();

    // Habilitar el PWM (por si acaso no lo activó SYSTEM_Initialize)
    PWM1_16BIT_Enable();
    
    // Inicializar el PID
    PID_Init(&pid, 35, 0.1, 1.8, 800, 0.01); // kp, ki, kd, maxOutput, dt

    char command[11];  // 10 caracteres + 1 para el '\0'

    while (1) {         
          // Leer canal 0
        // ------- set point ----------
        LED0_SetLow();
        clear_data(command, 11);  // O sizeof(command)
        if (UART1_IsRxReady() == 1){

            // Leer la cadena desde UART
            Uart_Read_String(command, 11);
            printf("Recibido: %s\n", command);  // Verificar qué llegó

            // Solo leer el primer número
            if (sscanf(command, "%hu", &adcValue) == 1) { 
                 printf("adcValue: %u\n", adcValue);
            } else {
                printf("Error al leer el setpoint.\n");
            }
        }
        
        printf("theta: %d  setpoint:%d encoder: %d error: %0.2f "
                "leyControl: %.2f pwmDuty: %.2f dir: %u \n\r", 
                  theta, adcValue, encoder,pid.error,pid.output,pwmDuty,dir);
        
        __delay_ms(100); // Delay pequeño para suavizar cambios
    }    
}
