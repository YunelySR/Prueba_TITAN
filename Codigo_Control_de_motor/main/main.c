//Librerías a utilizar en el programa
#include <stdio.h>
#include <driver/gpio.h>
#include <esp_adc/adc_oneshot.h>
#include <driver/mcpwm_timer.h>
#include <driver/mcpwm_oper.h>
#include <driver/mcpwm_cmpr.h>
#include <driver/mcpwm_gen.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/timers.h>
#include <freertos/queue.h>
#include <math.h>


//Macros a utilizar en la máquina de estado
#define ESTADO_INICIAL 0
#define ESTADO_ABIERTO 1
#define ESTADO_ABRIENDO 2
#define ESTADO_CERRADO 3
#define ESTADO_CERRANDO 4
#define ESTADO_DETENIDO 5
#define ESTADO_ERROR 6


//Macros para controlar los datos IO
#define FALSE 0
#define TRUE 1
#define DERECHA 1
#define IZQUIERDA 0
#define TIEMPO_MAX 1200   //Tiempo máximo que puede durar el porton abriéndose o cerrándose (1 minuto)


//Macros para definir las resoluciones del ADC y del PWM
#define RESOLUCION_ADC 4095
#define RESOLUCION_PWM 5000


//Macro utilizada para definir el tamaño de memoria asignada a cada tarea
#define STACK_SIZE 1024


/*
GPIOs de configuración      *       GPIOs del programa
GPIO 0                      *       GPIO 1 - ADC
GPIO 3                      *       GPIO 2 - BOTÓN
GPIO 45                     *       GPIO 38 - LSA
GPIO 46                     *       GPIO 37 - LSC
                            *       GPIO 5 - PWM 1
                            *       GPIO 4 - PWM 2
*/

//Macros para los pines del microcontrolador
#define GPIO_BOTON 2
#define GPIO_LSA 38
#define GPIO_LSC 37
#define GPIO_PWM_1 5
#define GPIO_PWM_2 4


//Manejadores de eventos y variables que se utilizarán en el ADC
//Declaración y configuración de los parámetros del manejador de eventos del ADC
adc_oneshot_unit_handle_t handle_adc1;
adc_oneshot_unit_init_cfg_t init_config_handle =
{
    .unit_id = ADC_UNIT_1,              //ADC que usaremos, en este caso el 1
    .clk_src = ADC_RTC_CLK_SRC_DEFAULT, //Fuente de reloj que usará el ADC, en este caso la fuente de reloj RC Fast que estápor defecto
    .ulp_mode = ADC_ULP_MODE_DISABLE,   //Desactivamos el modo ULP (Ultra Low Power) cuando usemos el ADC
};

//Declaración y configuración de los parámetros de los canales ADC que usaremos
adc_oneshot_chan_cfg_t config_param_adc =
{
    .atten = ADC_ATTEN_DB_0,        //Ajustamos la atenuación para los canales del ADC en 0dB
    .bitwidth = ADC_BITWIDTH_12,    //Ajustamos la resolución del ADC en 12 bits
};

// Estructura de datos para calcular el valor RMS del ADC
struct valor_rms
{
    unsigned int numero_de_lecturas; // Número de lecturas que tomaremos del ADC para calcular el valor RMS
    int lectura;                     // Valor de la lectura momentánea del ADC
    unsigned long int acumulado;     // Variable para almacenar la suma de los cuadrados de las lecturas tomadas del ADC
} calculo_adc;


//Manejadores de eventos y variables que se utilizarán en el PWM
//Declaración y configuración de los parámetros del manejador de eventos del timer que utilizará el PWM
mcpwm_timer_handle_t handle_timer_1;
mcpwm_timer_handle_t handle_timer_2;
mcpwm_timer_config_t config_timer =
{
    .group_id = 0,                           // Grupo al que pertenece el timer
    .intr_priority = 0,                      // El controlador asigna una prioridad predeterminada
    .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,  // Elegimos la fuente de reloj para el timer
    .resolution_hz = 100000000,              // Elegimos la frecuencia de entrada para el módulo MCPWM, en este caso 100MHz, por lo que 1 ciclo dura 10ns
    .count_mode = MCPWM_TIMER_COUNT_MODE_UP, // Elegimos el modo de conteo hacia arriba
    .period_ticks = RESOLUCION_ADC,          // Elegimos la cantidad de ciclos base que tendrá el PWM, en este caso 5000, por lo que su frecuencia será de 20kHz
    .flags.update_period_on_empty = false,
    .flags.update_period_on_sync = false,
    .flags.allow_pd = false,
};

//Declaración y configuración de los parámetros del manejador de eventos del operador que utilizará el PWM
mcpwm_oper_handle_t handle_operator_1;
mcpwm_oper_handle_t handle_operator_2;
mcpwm_operator_config_t config_operator =
{
    .group_id = 0,                              // Grupo al que pertenece el operador
    .intr_priority = 0,                         // El controlador asigna una prioridad predeterminada
    .flags.update_gen_action_on_tez = false,
    .flags.update_gen_action_on_tep = true,     // Elegimos que se actualice la acción del generador cuando el temporizador llege al máximo
    .flags.update_gen_action_on_sync = false,
    .flags.update_dead_time_on_tez = false,
    .flags.update_dead_time_on_tep = false,
    .flags.update_dead_time_on_sync = false,
};

//Declaración y configuración de los parámetros del manejador de eventos del comparador que utilizará el PWM
mcpwm_cmpr_handle_t handle_comparator_1;
mcpwm_cmpr_handle_t handle_comparator_2;
mcpwm_comparator_config_t config_comparator =
{
    .intr_priority = 0,                 // Grupo al que pertenece el comparador
    .flags.update_cmp_on_tez = false,
    .flags.update_cmp_on_tep = true,    // Elegimos que se actualice el valor del umbral cuando el temporizador llegue al máximo
    .flags.update_cmp_on_sync = false,
};

//Declaración y configuración de los parámetros del manejador de eventos del generador que utilizará el PWM
mcpwm_gen_handle_t handle_generator_1;
mcpwm_gen_handle_t handle_generator_2;
mcpwm_generator_config_t config_generator_1 =
{
    .gen_gpio_num = GPIO_PWM_1,     // Elegimos el GPIO 5 como salida del PWM 1
    .flags.invert_pwm = false,      // Elegimos no invertir la señal PWM 1
    .flags.pull_up = false,
    .flags.pull_down = true,        // Habilitamos una resistencia pull-down en el GPIO del PWM 1
    .flags.io_od_mode = false,
    .flags.io_loop_back = false,
};
mcpwm_generator_config_t config_generator_2 =
{
    .gen_gpio_num = GPIO_PWM_2,     // Elegimos el GPIO 4 como salida del PWM 2
    .flags.invert_pwm = false,      // Elegimos no invertir la señal PWM 2
    .flags.pull_up = false,
    .flags.pull_down = true,        // Habilitamos una resistencia pull-down en el GPIO del PWM 2
    .flags.io_od_mode = false,
    .flags.io_loop_back = false,
};

//Variables para utilizar los timers
TimerHandle_t xTimer1;
TimerHandle_t xTimer2;
int periodo_del_timer_1 = 50;
int periodo_del_timer_2 = 2;
int timer1_ID = 1;
int timer2_ID = 2;


//Variables para utilizar las colas
QueueHandle_t xQueue1;
QueueHandle_t xQueue2;


//Variables para controlar el flujo dentro de la máquina de estado
int ESTADO_ANTERIOR = ESTADO_INICIAL;
int ESTADO_ACTUAL = ESTADO_INICIAL;
int ESTADO_SIGUIENTE = ESTADO_INICIAL;


//Estructura de datos utilizada para almacenar los datos de IO
struct datos
{
    unsigned int LSC : 1;          // Limit switch que indica que porton está cerrado
    unsigned int LSA : 1;          // Limit switch que indica que el porton está abierto
    unsigned int boton : 1;        // Botón para abrir/cerrar/detener el porton
    unsigned int debounce_1 : 1;   // Variable para el debounce del botón
    unsigned int debounce_2 : 1;   // Variable para el debounce del botón
    unsigned int potenciometro;    // Valor del potenciometro registrado por el ADC
    bool direccion_motor;          // Indicará la dirección del motor del porton
    bool motor;                    // Indicará el encendido/apagado del motor del porton
    unsigned int duty_cycle;       // Valor del duty cycle del PWM que se le entregará al motor
    unsigned int tiempo_de_espera; // Variable que registrará cuanto tiempo dura el porton abriéndose o cerrándose
}datos_IO;


//Prototipos de las funciones de la máquina de estado
int Funcion_Inicial(void);
int Funcion_Abierto(void);
int Funcion_Abriendo(void);
int Funcion_Cerrado(void);
int Funcion_Cerrando(void);
int Funcion_Detenido(void);
int Funcion_Error(void);


//Prototipos de las funciones de las tareas que se crearán
void Tarea_lectura_de_entradas(void * pvParameters);
void Tarea_control_del_motor(void * pvParameters);


//Función para configurar los GPIOs digitales
esp_err_t Configuracion_GPIO(void)
{
    //Reseteamos los pines desacoplándolos de otros periféricos
    gpio_reset_pin(GPIO_BOTON);
    gpio_reset_pin(GPIO_LSA);
    gpio_reset_pin(GPIO_LSC);

    //Definimos si los pines son de entrada o de salida
    gpio_set_direction(GPIO_BOTON, GPIO_MODE_INPUT);
    gpio_set_direction(GPIO_LSA, GPIO_MODE_INPUT);
    gpio_set_direction(GPIO_LSC, GPIO_MODE_INPUT);

    //Inicializamos las variables del debounce en verdadero
    datos_IO.debounce_1 = TRUE;
    datos_IO.debounce_2 = TRUE;

    return ESP_OK;
}


//Función para configurar el ADC
esp_err_t Configuracion_ADC(void)
{
    //Creación del manejador de eventos del ADC
    adc_oneshot_new_unit(&init_config_handle, &handle_adc1);

    //Configuración del canal ADC que usaremos, en este caso es el canal 0 del ADC1
    adc_oneshot_config_channel(handle_adc1, ADC_CHANNEL_0, &config_param_adc);

    //Inicializamos en 0 las variables que utilizaremos para el cálculo RMS del ADC
    calculo_adc.numero_de_lecturas = 0;
    calculo_adc.lectura = 0;
    calculo_adc.acumulado = 0;

    return ESP_OK;
}


//Función para configurar el PWM
esp_err_t Configuracion_PWM(void)
{
    //Creación de los timers del PWM
    mcpwm_new_timer(&config_timer, &handle_timer_1);
    mcpwm_new_timer(&config_timer, &handle_timer_2);

    //Creación de los operadores del PWM
    mcpwm_new_operator(&config_operator, &handle_operator_1);
    mcpwm_new_operator(&config_operator, &handle_operator_2);

    //Creación de los comparadores del PWM
    mcpwm_new_comparator(handle_operator_1, &config_comparator, &handle_comparator_1);
    mcpwm_new_comparator(handle_operator_2, &config_comparator, &handle_comparator_2);

    //Creación de los generadores del PWM
    mcpwm_new_generator(handle_operator_1, &config_generator_1, &handle_generator_1);
    mcpwm_new_generator(handle_operator_2, &config_generator_2, &handle_generator_2);

    //Habilitamos los timers del PWM
    mcpwm_timer_enable(handle_timer_1);
    mcpwm_timer_enable(handle_timer_2);

    //Inicializamos los timers del PWM
    mcpwm_timer_start_stop(handle_timer_1, MCPWM_TIMER_START_NO_STOP);
    mcpwm_timer_start_stop(handle_timer_2, MCPWM_TIMER_START_NO_STOP);

    //Conectamos los timers del PWM con los operadores del PWM
    mcpwm_operator_connect_timer(handle_operator_1, handle_timer_1);
    mcpwm_operator_connect_timer(handle_operator_2, handle_timer_2);

    //Ajustamos los duty cycles de los PWM a un 0%, es decir, ajustamos el valor del comparadores a 0
    mcpwm_comparator_set_compare_value(handle_comparator_1, 0);
    mcpwm_comparator_set_compare_value(handle_comparator_2, 0);

    //Estructuras de los eventos que se generan cuando los timers se igualan a 0
    mcpwm_gen_timer_event_action_t tim_event_action_1 =
    {
        .direction = MCPWM_TIMER_DIRECTION_UP,
        .event = MCPWM_TIMER_EVENT_EMPTY,
        .action = MCPWM_GEN_ACTION_HIGH,
    };
    mcpwm_gen_timer_event_action_t tim_event_action_2 =
    {
        .direction = MCPWM_TIMER_DIRECTION_UP,
        .event = MCPWM_TIMER_EVENT_EMPTY,
        .action = MCPWM_GEN_ACTION_HIGH,
    };

    //Estructuras de los eventos que se generan cuando los timers se igualan al valor de sus comparadores 
    mcpwm_gen_compare_event_action_t comp_event_action_1 =
    {
        .direction = MCPWM_TIMER_DIRECTION_UP,
        .comparator = handle_comparator_1,
        .action = MCPWM_GEN_ACTION_LOW,
    };
    mcpwm_gen_compare_event_action_t comp_event_action_2 =
    {
        .direction = MCPWM_TIMER_DIRECTION_UP,
        .comparator = handle_comparator_2,
        .action = MCPWM_GEN_ACTION_LOW,
    };

    //Ajuste de los generadores del PWM para gestionar los eventos de los timers cuando se igualan a 0
    mcpwm_generator_set_action_on_timer_event(handle_generator_1, tim_event_action_1);
    mcpwm_generator_set_action_on_timer_event(handle_generator_2, tim_event_action_2);

    //Ajuste de los generadores del PWM para gestionar los eventos de los comparadores cuando sus timers se igualan a ellos
    mcpwm_generator_set_action_on_compare_event(handle_generator_1, comp_event_action_1);
    mcpwm_generator_set_action_on_compare_event(handle_generator_2, comp_event_action_2);

    return ESP_OK;
}


//Función para leer los sensores y el botón cada cierto tiempo
void Lectura_GPIO(TimerHandle_t pxTimer)
{
    //Lectura de los GPIOs de los sensores, el botón y el potenciómetro
    datos_IO.LSA = gpio_get_level(GPIO_LSA);
    datos_IO.LSC = gpio_get_level(GPIO_LSC);
    datos_IO.debounce_1 = gpio_get_level(GPIO_BOTON);

    //Aquí hacemos un debounce para el botón
    if (datos_IO.debounce_1 != datos_IO.debounce_2)
    {
        //Si el estado anterior (debounce_2) es verdadero y el estado actual (debounce_1) es falso entonces registramos el flanco de bajada
        if ((datos_IO.debounce_2 == TRUE) && (datos_IO.debounce_1 == FALSE))
        {
            //Igualamos la variable del botón a falso indicando que el botón se ha presionado
            datos_IO.boton = FALSE;
        }
        
        //Cada vez que hay un flanco de bajada o de subida igualamos las variables
        datos_IO.debounce_2 = datos_IO.debounce_1;
    }

    //Incremento del tiempo de que dura el porton abriendo o cerrando
    if ((ESTADO_ACTUAL == ESTADO_ABRIENDO) || (ESTADO_ACTUAL == ESTADO_CERRANDO))
    {
        datos_IO.tiempo_de_espera += periodo_del_timer_1;
    }
}


//Función para leer y calcular los valores del ADC
void Lectura_ADC(TimerHandle_t pxtimer)
{
    //Leemos el valor del ADC cada vez que timer active la interrupción
    adc_oneshot_read(handle_adc1, ADC_CHANNEL_0, &calculo_adc.lectura);

    //Calculamos su potencia elevandolo al cuadrado y luego lo sumamos al acumulado
    calculo_adc.acumulado += (calculo_adc.lectura * calculo_adc.lectura);

    //Incrementamos el número de lecturas en 1 cada que el timer active la interrupción y lea el valor del ADC
    ++calculo_adc.numero_de_lecturas;
    
    //Si el número de lecturas es mayor a 20 entonces procedemos a hacer el cálculo del RMS
    if (calculo_adc.numero_de_lecturas >= 20)
    {
        //Aquí calculamos y asignamos el valor RMS del ADC a la variable del potenciómetro
        datos_IO.potenciometro = sqrt(calculo_adc.acumulado / calculo_adc.numero_de_lecturas);

        //Luego de actualizar el valor del potenciómetro igualamos el número de lecturas a 0 para que inicie una nueva medición
        calculo_adc.numero_de_lecturas = 0;
    }
}


//Función para crear el timer
esp_err_t Creacion_de_los_timers(void)
{
    // Creación del timer 1 para leer los GPIOs
    xTimer1 = xTimerCreate("Timer para leer los sensores y el boton", // Nombre o breve descripción del timer
                           (pdMS_TO_TICKS(periodo_del_timer_1)),      // Periodo con el que el timer ejecutará la interrupción
                           pdTRUE,                                    // El timer se recargará automáticamente cuando termine
                           (void *)timer1_ID,                         // ID única del timer
                           Lectura_GPIO                               // Función a la que recurrirá el timer cuando ejecute la interrupción
    );

    if (xTimer1 == NULL)
    {
        // El timer no fue creado
    }
    else
    {
        // Inicia el timer
        if (xTimerStart(xTimer1, 0) != pdPASS)
        {
            // No se pudo configurar el timer en el estado Activo.
        }
    }

    // Creación del timer 2 para leer el ADC
    xTimer2 = xTimerCreate("Timer para leer y calcular el valor del ADC", // Nombre o breve descripción del timer
                           (pdMS_TO_TICKS(periodo_del_timer_2)),          // Periodo con el que el timer ejecutará la interrupción
                           pdTRUE,                                        // El timer se recargará automáticamente cuando termine
                           (void *)timer2_ID,                             // ID única del timer
                           Lectura_ADC                                    // Función a la que recurrirá el timer cuando ejecute la interrupción
    );

    if (xTimer2 == NULL)
    {
        // El timer no fue creado
    }
    else
    {
        // Inicia el timer
        if (xTimerStart(xTimer2, 0) != pdPASS)
        {
            // No se pudo configurar el timer en el estado Activo.
        }
    }

    return ESP_OK;
}


esp_err_t Creacion_de_las_colas(void)
{
    xQueue1 = xQueueCreate( 4, sizeof( bool ) );
    if( xQueue1 == 0 )
    {
        // La cola no se creó y no debe ser usada.
    };

    xQueue2 = xQueueCreate( 4, sizeof( bool ) );
    if( xQueue2 == 0 )
    {
        // La cola no se creó y no debe ser usada.
    };

    return ESP_OK;
}


//Función para crear las tareas
esp_err_t Creacion_de_las_tareas(void)
{
    static uint8_t ucParameterToPass;
    TaskHandle_t xHandle = NULL;

    xTaskCreate(Tarea_lectura_de_entradas, "Tarea para leer las entradas",
                STACK_SIZE,
                &ucParameterToPass,
                1,
                &xHandle);
    configASSERT( xHandle );

    xTaskCreate(Tarea_control_del_motor, "Tarea para controlar el motor",
                STACK_SIZE,
                &ucParameterToPass,
                2,
                &xHandle);
    configASSERT(xHandle);

    return ESP_OK;
}


//Función principal del microcontrolador donde se llaman a las demás funciones para configurar procesos, variables y periféricos
void app_main(void)
{
    Configuracion_GPIO();
    Configuracion_ADC();
    Configuracion_PWM();
    Creacion_de_los_timers();
    Creacion_de_las_colas();
    Creacion_de_las_tareas();
}


void Tarea_lectura_de_entradas(void * pvParameters)
{
    //Máquina de estados
    for(;;)
    {
        //Estado inicial
        if (ESTADO_SIGUIENTE == ESTADO_INICIAL)  
        {
            ESTADO_SIGUIENTE = Funcion_Inicial();
        }
        
        //Estado abierto
        if (ESTADO_SIGUIENTE == ESTADO_ABIERTO)  
        {
            ESTADO_SIGUIENTE = Funcion_Abierto();
        }

        //Estado abriendo
        if (ESTADO_SIGUIENTE == ESTADO_ABRIENDO)  
        {
            ESTADO_SIGUIENTE = Funcion_Abriendo();
        }

        //Estado cerrado
        if (ESTADO_SIGUIENTE == ESTADO_CERRADO)  
        {
            ESTADO_SIGUIENTE = Funcion_Cerrado();
        }

        //Estado cerrando
        if (ESTADO_SIGUIENTE == ESTADO_CERRANDO)  
        {
            ESTADO_SIGUIENTE = Funcion_Cerrando();
        }

        //Estado detenido
        if (ESTADO_SIGUIENTE == ESTADO_DETENIDO)  
        {
            ESTADO_SIGUIENTE = Funcion_Detenido();
        }

        //Estado error
        if (ESTADO_SIGUIENTE == ESTADO_ERROR)  
        {
            ESTADO_SIGUIENTE = Funcion_Error();
        }
    }
}


int Funcion_Inicial(void)
{
    //Actualización de estados
    ESTADO_ANTERIOR = ESTADO_ACTUAL;
    ESTADO_ACTUAL = ESTADO_INICIAL;

    //Actualización de datos
    datos_IO.motor = FALSE;

    //Envío de datos a la tarea del control del motor usando colas
    xQueueSend(xQueue1, &datos_IO.motor, pdMS_TO_TICKS(50));

    for(;;)
    {
        //Delay para que no se dispare el WDT
        vTaskDelay(pdMS_TO_TICKS(10));

        //Estado inicial ---> Estado cerrando
        if ((datos_IO.LSA == FALSE) && (datos_IO.LSC == FALSE))
        {
            return ESTADO_CERRANDO;
        }

        //Estado inicial ---> Estado cerrado
        if ((datos_IO.LSA == FALSE) && (datos_IO.LSC == TRUE))
        {
            return ESTADO_CERRADO;
        }
        
        //Estado inicial ---> Estado cerrando
        if ((datos_IO.LSA == TRUE) && (datos_IO.LSC == FALSE))
        {
            return ESTADO_CERRANDO;
        }

        //Estado inicial ---> Estado error
        if ((datos_IO.LSA == TRUE) && (datos_IO.LSC == TRUE))
        {
            return ESTADO_ERROR;
        }
    }
}


int Funcion_Abierto(void)
{
    //Actualización de estados
    ESTADO_ANTERIOR = ESTADO_ACTUAL;
    ESTADO_ACTUAL = ESTADO_ABIERTO;

    //Actualización de datos
    datos_IO.motor = FALSE;
    datos_IO.boton = TRUE;

    //Envío de datos a la tarea del control del motor usando colas
    xQueueSend(xQueue1, &datos_IO.motor, pdMS_TO_TICKS(50));

    for(;;)
    {
        //Delay para que no se dispare el WDT
        vTaskDelay(pdMS_TO_TICKS(10));

        //Estado abierto ---> Estado cerrando
        if (datos_IO.boton == FALSE)
        {
            return ESTADO_CERRANDO;
        }
    }
}


int Funcion_Abriendo(void)
{
    //Actualización de estados
    ESTADO_ANTERIOR = ESTADO_ACTUAL;
    ESTADO_ACTUAL = ESTADO_ABRIENDO;

    //Actualización de datos
    datos_IO.direccion_motor = IZQUIERDA;
    datos_IO.motor = TRUE;
    datos_IO.boton = TRUE;
    datos_IO.tiempo_de_espera = 0;

    //Envío de datos a la tarea del control del motor usando colas
    xQueueSend(xQueue1, &datos_IO.motor, pdMS_TO_TICKS(50));
    xQueueSend(xQueue2, &datos_IO.direccion_motor, pdMS_TO_TICKS(50));

    for (;;)
    {
        //Delay para que no se dispare el WDT
        vTaskDelay(pdMS_TO_TICKS(10));

        //Estado abriendo ---> Estado abierto
        if (datos_IO.LSA == TRUE)
        {
            return ESTADO_ABIERTO;
        }

        //Estado abriendo ---> Estado detenido
        if (datos_IO.boton == FALSE)
        {
            return ESTADO_DETENIDO;
        }

        //Estado abriendo ---> Estado error
        if (datos_IO.tiempo_de_espera >= TIEMPO_MAX)
        {
            return ESTADO_ERROR;
        }   
    }
}


int Funcion_Cerrado(void)
{
    //Actualización de estados
    ESTADO_ANTERIOR = ESTADO_ACTUAL;
    ESTADO_ACTUAL = ESTADO_CERRADO;

    //Actualización de datos
    datos_IO.motor = FALSE;
    datos_IO.boton = TRUE;

    //Envío de datos a la tarea del control del motor usando colas
    xQueueSend(xQueue1, &datos_IO.motor, pdMS_TO_TICKS(50));

    for(;;)
    {
        //Delay para que no se dispare el WDT
        vTaskDelay(pdMS_TO_TICKS(10));

        //Estado cerrado ---> Estado abriendo
        if (datos_IO.boton == FALSE)
        {
            return ESTADO_ABRIENDO;
        }
    }
}


int Funcion_Cerrando(void)
{
    //Actualización de estados
    ESTADO_ANTERIOR = ESTADO_ACTUAL;
    ESTADO_ACTUAL = ESTADO_CERRANDO;

    //Actualización de datos
    datos_IO.direccion_motor = DERECHA;
    datos_IO.motor = TRUE;
    datos_IO.boton = TRUE;
    datos_IO.tiempo_de_espera = 0;

    //Envío de datos a la tarea del control del motor usando colas
    xQueueSend(xQueue1, &datos_IO.motor, pdMS_TO_TICKS(50));
    xQueueSend(xQueue2, &datos_IO.direccion_motor, pdMS_TO_TICKS(50));

    for(;;)
    {
        //Delay para que no se dispare el WDT
        vTaskDelay(pdMS_TO_TICKS(10));

        //Estado cerrando ---> Estado cerrado
        if(datos_IO.LSC == TRUE)
        {
            return ESTADO_CERRADO;
        }

        //Estado cerrando ---> Estado detenido
        if(datos_IO.boton == FALSE)
        {
            return ESTADO_DETENIDO;
        }

        //Estado cerrando ---> Estado error
        if(datos_IO.tiempo_de_espera >= TIEMPO_MAX)
        {
            return ESTADO_ERROR;
        }
    }
}


int Funcion_Detenido(void)
{
    //Actualización de estados
    ESTADO_ANTERIOR = ESTADO_ACTUAL;
    ESTADO_ACTUAL = ESTADO_DETENIDO;

    //Actualización de datos
    datos_IO.motor = FALSE;
    datos_IO.boton = TRUE;

    //Envío de datos a la tarea del control del motor usando colas
    xQueueSend(xQueue1, &datos_IO.motor, pdMS_TO_TICKS(50));

    for(;;)
    {
        //Delay para que no se dispare el WDT
        vTaskDelay(pdMS_TO_TICKS(10));

        //Estado detenido ---> Estado abriendo
        if ((datos_IO.boton == FALSE) && (ESTADO_ANTERIOR == ESTADO_ABRIENDO))
        {
            return ESTADO_ABRIENDO;
        }

        //Estado detenido ---> Estado cerrando
        if ((datos_IO.boton == FALSE) && (ESTADO_ANTERIOR == ESTADO_CERRANDO))
        {
            return ESTADO_CERRANDO;
        }
    }
}


int Funcion_Error(void)
{
    //Actualización de estados
    ESTADO_ANTERIOR = ESTADO_ACTUAL;
    ESTADO_ACTUAL = ESTADO_ERROR;

    //Actualización de datos
    datos_IO.motor = FALSE;
    datos_IO.boton = TRUE;
    datos_IO.tiempo_de_espera = 0;

    //Envío de datos a la tarea del control del motor usando colas
    xQueueSend(xQueue1, &datos_IO.motor, pdMS_TO_TICKS(50));

    for(;;)
    {
        //Delay para que no se dispare el WDT
        vTaskDelay(pdMS_TO_TICKS(10));

        //Estado error ---> Estado inicial
        if (datos_IO.boton == FALSE)
        {
            return ESTADO_INICIAL;
        }
    }
}


void Tarea_control_del_motor(void * pvParameters)
{
    //Variables para ajustar los forces de los generadores PWM una sola vez
    bool direccion_actual = 0;
    bool direccion_anterior = 0;
    bool motor_actual = FALSE;
    bool motor_anterior = FALSE;
    bool ajuste = FALSE;

    for(;;)
    {
        //Recepción de datos de la tarea de lectura de entradas usando colas
        xQueueReceive(xQueue1, &motor_actual, pdMS_TO_TICKS(10));
        xQueueReceive(xQueue2, &direccion_actual, pdMS_TO_TICKS(10));

        //Si hay alguna modificación en los valores del motor y su dirección entonces se reseteará la variable de ajuste
        if ((motor_actual != motor_anterior) || (direccion_actual != direccion_anterior))
        {
            ajuste = FALSE;
            motor_anterior = motor_actual;
            direccion_anterior = direccion_actual;
        }
        
        //Si el motor está pagado entonces ponemos los generadores PWM en un nivel LOW
        if (((motor_actual == FALSE) && (direccion_actual == IZQUIERDA)) || ((motor_actual == FALSE) && (direccion_actual == DERECHA)))
        {
            //Delay para que no se dispare el WDT
            vTaskDelay(pdMS_TO_TICKS(10));

            //Forzamos a los generadores PWM a estar en un nivel LOW
            if (ajuste == FALSE)
            {
                mcpwm_generator_set_force_level(handle_generator_1, FALSE, TRUE);
                mcpwm_generator_set_force_level(handle_generator_2, FALSE, TRUE);
                ajuste = TRUE;
            }
        }

        /*
        Si el motor está encendido y con dirección a la derecha entonces asignamos un duty cycle en 
        el generador PWM 1 y el generador PWM 2 lo ponemos en un nivel LOW
        */
        if ((motor_actual == TRUE) && (direccion_actual == DERECHA))
        {
            //Delay para que no se dispare el WDT
            vTaskDelay(pdMS_TO_TICKS(10));

            //Forzamos al generador del PWM 2 a estar en un nivel LOW y removemos el force al generador del PWM 1
            if (ajuste == FALSE)
            {
                mcpwm_generator_set_force_level(handle_generator_2, FALSE, TRUE);
                mcpwm_generator_set_force_level(handle_generator_1, -1, FALSE);
                ajuste = TRUE;
            }
            
            //Calculamos y asiganmos el valor del duty cycle del generador del PWM 1 de manera continua
            datos_IO.duty_cycle = (datos_IO.potenciometro * RESOLUCION_PWM) / RESOLUCION_ADC;
            mcpwm_comparator_set_compare_value(handle_comparator_1, datos_IO.duty_cycle);
        }

        /*
        Si el motor está encendido y con dirección a la izquierda entonces asignamos un duty cycle en 
        el generador PWM 2 y el generador PWM 1 lo ponemos en un nivel LOW
        */
        if ((motor_actual == TRUE) && (direccion_actual == IZQUIERDA))
        {
            //Delay para que no se dispare el WDT
            vTaskDelay(pdMS_TO_TICKS(10));

            //Forzamos al generador del PWM 1 a estar en un nivel LOW y removemos el force al generador del PWM 2
            if (ajuste == FALSE)
            {
                mcpwm_generator_set_force_level(handle_generator_1, FALSE, TRUE);
                mcpwm_generator_set_force_level(handle_generator_2, -1, FALSE);
                ajuste = TRUE;
            }
            
            //Calculamos y asiganmos el valor del duty cycle del generador del PWM 2 de manera continua
            datos_IO.duty_cycle = (datos_IO.potenciometro * RESOLUCION_PWM) / RESOLUCION_ADC;
            mcpwm_comparator_set_compare_value(handle_comparator_2, datos_IO.duty_cycle);
        }
    }
}