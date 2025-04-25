#include "mcc_generated_files/system/system.h"
#include <stdbool.h>  // 
#include <math.h>
#include <string.h>  // Para usar memset

#define Forward 1
#define Backward 0

volatile int16_t theta = 0; 
uint8_t dir;
int16_t adcValue = 0;

volatile uint16_t millisContador = 0;  // Para contar hasta 1000 ms
volatile int16_t encoder;

float previousError = 0;
float pwmDuty;
float error;


// Variables para el sistema difuso
float ruleActivation1 = 0, ruleActivation2 = 0, ruleActivation3 = 0;
float ruleActivation4 = 0, ruleActivation5 = 0, ruleActivation6 = 0;
float ruleActivation7 = 0, ruleActivation8 = 0, ruleActivation9 = 0;
float maxNegativeOutput = 0, maxZeroOutput = 0, maxPositiveOutput = 0, fuzzyControlOutput = 0;

// Variables globales para almacenar los resultados de las funciones de membresía
float errorNegative, errorZero, errorPositive;
float derivativeNegative, derivativeZero, derivativePositive;
float controlValueNegative, controlValueZero, controlValuePositive;


// Conjuntos difusos
float errorNegativeSet[4] = {-300, -300, -10, 0};
float errorZeroSet[3] = {-10, 0, 10};
float errorPositiveSet[4] = {0, 10, 300, 300};

float derivativeNegativeSet[4] = {-400, -400, -200, 0};
float derivativeZeroSet[3] = {-200, 0, 200};
float derivativePositiveSet[4] = {0, 200, 400, 400};

float controlNegativeSet[4] = {-500, -500, -150, 0};
float controlZeroSet[3] = {-150, 0, 150};
float controlPositiveSet[4] = {0, 150, 500, 500};

float errorInput = 0, derivativeInput = 0;

// Funciones de membresía
float triangularMembership(float x, float a, float b, float c) {
    if (x <= a || x >= c) {
        return 0.0;
    } else if (x >= a && x <= b) {
        return (x - a) / (b - a);
    } else if (x >= b && x <= c) {
        return (c - x) / (c - b);
    }
    return 0.0;
}

float trapezoidalMembership(float x, float a, float b, float c, float d) {
    if (x <= a || x >= d) {
        return 0.0;
    } else if (x >= a && x <= b) {
        return (x - a) / (b - a);
    } else if (x >= b && x <= c) {
        return 1.0;
    } else if (x >= c && x <= d) {
        return (d - x) / (c - d);
    }
    return 0.0;
}

// Funciones para calcular los valores reales de las leyes de control
float calculateNegativeControlValue() {
    return (((1 - maxNegativeOutput) * (controlNegativeSet[3] - controlNegativeSet[2]) + controlNegativeSet[2] + controlNegativeSet[1]) / 2); 
}

float calculateZeroControlValue() {
    float A, B;
    A = maxZeroOutput * (controlZeroSet[1] - controlZeroSet[0]) + controlZeroSet[0];
    B = (1 - maxZeroOutput) * (controlZeroSet[2] - controlZeroSet[1]) + controlZeroSet[1];
    return ((A + B) / 2);
}

float calculatePositiveControlValue() {
    return ((maxPositiveOutput * (controlPositiveSet[1] - controlPositiveSet[0]) + controlPositiveSet[0] + controlPositiveSet[2]) / 2);
}

// Función para inicializar las variables de membresía
void initializeMembershipFunctions() {
    // Inicializar las funciones de membresía del error
    errorNegative = trapezoidalMembership(errorInput, errorNegativeSet[0], errorNegativeSet[1], errorNegativeSet[2], errorNegativeSet[3]);
    errorZero = triangularMembership(errorInput, errorZeroSet[0], errorZeroSet[1], errorZeroSet[2]);
    errorPositive = trapezoidalMembership(errorInput, errorPositiveSet[0], errorPositiveSet[1], errorPositiveSet[2], errorPositiveSet[3]);

    // Inicializar las funciones de membresía de la derivada del error
    derivativeNegative = trapezoidalMembership(derivativeInput, derivativeNegativeSet[0], derivativeNegativeSet[1], derivativeNegativeSet[2], derivativeNegativeSet[3]);
    derivativeZero = triangularMembership(derivativeInput, derivativeZeroSet[0], derivativeZeroSet[1], derivativeZeroSet[2]);
    derivativePositive = trapezoidalMembership(derivativeInput, derivativePositiveSet[0], derivativePositiveSet[1], derivativePositiveSet[2], derivativePositiveSet[3]);
    
    // Inicializar las leyes de control corregir hallar z
    controlValueNegative = calculateNegativeControlValue();
    controlValueZero = calculateZeroControlValue();
    controlValuePositive = calculatePositiveControlValue();
}
// Control difuso
void fuzzyControl() {
    
    ruleActivation1 = fmin(derivativeNegative, errorNegative); // Error grande y disminuyendo rápidamente
    ruleActivation2 = fmin(derivativeZero, errorNegative);     // Error grande y estable
    ruleActivation3 = fmin(derivativePositive, errorNegative); // Error grande y aumentando rápidamente

    ruleActivation4 = fmin(derivativeNegative, errorZero);     // Error pequeño y disminuyendo rápidamente
    ruleActivation5 = fmin(derivativeZero, errorZero);         // Error pequeño y estable
    ruleActivation6 = fmin(derivativePositive, errorZero);     // Error pequeño y aumentando rápidamente

    ruleActivation7 = fmin(derivativeNegative, errorPositive); // Error grande en el otro sentido y disminuyendo rápidamente
    ruleActivation8 = fmin(derivativeZero, errorPositive);     // Error grande en el otro sentido y estable
    ruleActivation9 = fmin(derivativePositive, errorPositive); // Error grande en el otro sentido y aumentando rápidamente

    // Calcular las salidas máximas para cada conjunto
    maxNegativeOutput = fmax(ruleActivation1, ruleActivation2);
    maxZeroOutput = fmax(ruleActivation3, ruleActivation4);
    maxZeroOutput = fmax(maxZeroOutput,ruleActivation5);
    maxZeroOutput = fmax(maxZeroOutput, ruleActivation6);
    maxZeroOutput = fmax(maxZeroOutput, ruleActivation7);
    maxPositiveOutput = fmax(ruleActivation8, ruleActivation9);

  float denominator = maxNegativeOutput + maxZeroOutput + maxPositiveOutput;
  if (denominator == 0) {
    fuzzyControlOutput = 0;
  } else {
    fuzzyControlOutput = ((controlValueNegative * maxNegativeOutput) + 
                          (controlValueZero * maxZeroOutput) + 
                          (controlValuePositive * maxPositiveOutput)) / denominator;
  }
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



void temporizador_ISR(void) {
    millisContador++;  // Incrementar cada 1 ms
    if (millisContador >= 10) {  
        millisContador = 0;  // Reiniciar conteo    
        
        encoder = theta;
        error = (float)adcValue - (float)encoder;
        errorInput = error;
        
        float deltaTime = 0.01; // Intervalo de tiempo en segundos (10 ms)
        float derivative = (error - previousError) / deltaTime;
        previousError = error;
        derivativeInput = derivative;
        
        initializeMembershipFunctions();
        // Ejecutar lógica difusa
        fuzzyControl();

        // Dirección
        dir = (fuzzyControlOutput > 0) ? Forward : Backward;
        pwmDuty = fabs(fuzzyControlOutput);

        // Limitar el PWM
        if (pwmDuty > 800) pwmDuty = 800;
        setMotor(pwmDuty, dir);
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
        
        
        
        printf("theta: %d setPoint:%d encoder: %d error: %0.2f leyControl: %.2f pwm: %.2f\n\r", 
                  theta, adcValue, encoder,error,fuzzyControlOutput,pwmDuty);
        
        __delay_ms(100); // Delay pequeño para suavizar cambios
    }    
}
