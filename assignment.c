/*
 *  Author: 29980534 SOH OWEN
 */ 

#include <asf.h>
#include "stdio_serial.h"
#include "conf_board.h"
#include "conf_clock.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include <stdio.h>
#include <assert.h>
#include <string.h>

#define MY_PHOTOELECTRIC PIO_PC24_IDX //ARDUINO DUE DIGITAL PIN 6
#define ENCODER_A PIO_PC28_IDX //ARDUINO DUE DIGITAL PIN 3
#define ENCODER_B PIO_PB25_IDX //ARDUINO DUE DIGITAL PIN 2
#define MOTOR_DIR PIO_PC25_IDX //ARDUINO DUE DIGITAL PIN 5

// VALUES FOR TURNING POINT ESTIMATION
#define P1 -0.0005831 
#define P2 0.09918
#define P3 -5.6709
#define P4 129.6884

// VALUES FOR MOTOR SETTINGS
#define PWM_FREQUENCY      1000
#define PERIOD_VALUE       100
#define INIT_DUTY_VALUE    0

// GLOBAL VARIABLES
xQueueHandle xQueue;
xSemaphoreHandle xBinarySemaphore;

/** PWM channel instance for motor */
pwm_channel_t g_pwm_channel_led0;

int measured_step = 0; // The current position of the motor
int direction = 0; // The direction the pendulum is swinging
int time_taken = 0; // Time taken for long side of pendulum, inversely proportional to speed
int last_sense_time = 0; // Last time the PHOTOELECTRIC SENSOR was triggered, used to detect manual stop of pendulum
int half_period = 0; // Half period of pendulum, used to detect manual stop of pendulum

// Function prototypes
static void configure_console(void);
static void vHandlerTask( void *pvParameters );
void photoelectric_handler(const uint32_t id, const uint32_t index);
void quadrature_handler(const uint32_t id, const uint32_t index);

int main(void)
{
	/* Initialize the SAM system. */
	sysclk_init();
	board_init();
	configure_console();

	// SET PIN 6 FOR PHOTOELECTRIC SENSOR AS PULL-UP INPUT
	ioport_set_pin_dir(MY_PHOTOELECTRIC, IOPORT_DIR_INPUT);
	ioport_set_pin_mode(MY_PHOTOELECTRIC, IOPORT_MODE_PULLUP);
	
	// SET PIN 3 FOR ENCODER A AS PULL-UP INPUT
	ioport_set_pin_dir(ENCODER_A, IOPORT_DIR_INPUT);
	ioport_set_pin_mode(ENCODER_A, IOPORT_MODE_PULLUP);
	
	// SET PIN 2 FOR ENCODER B AS PULL-UP INPUT
	ioport_set_pin_dir(ENCODER_B, IOPORT_DIR_INPUT);
	ioport_set_pin_mode(ENCODER_B, IOPORT_MODE_PULLUP); 
	
	ioport_set_pin_dir(MOTOR_DIR, IOPORT_DIR_OUTPUT); // SET PIN 5 FOR MOTOR DIRECTION AS OUTPUT

	/* PWM CONFIG */
	pmc_enable_periph_clk(ID_PWM); /* Enable PWM peripheral clock */
	pwm_channel_disable(PWM, PWM_CHANNEL_6); /* Disable PWM channels for LEDs for safety */

	/* Set PWM clock A as PWM_FREQUENCY*PERIOD_VALUE (clock B is not used) */
	pwm_clock_t clock_setting = {
		.ul_clka = PWM_FREQUENCY * PERIOD_VALUE,
		.ul_clkb = 0,
		.ul_mck = sysclk_get_cpu_hz()
	};
	pwm_init(PWM, &clock_setting);
	
	/* Initialize PWM channel for PWM_CHANNEL_6 */
	g_pwm_channel_led0.alignment = PWM_ALIGN_LEFT; /* Period is left-aligned */
	g_pwm_channel_led0.polarity = PWM_LOW; /* Output waveform starts at a low level */
	g_pwm_channel_led0.ul_prescaler = PWM_CMR_CPRE_CLKA; /* Use PWM clock A as source clock */
	g_pwm_channel_led0.ul_period = PERIOD_VALUE; /* Period value of output waveform */
	g_pwm_channel_led0.ul_duty = INIT_DUTY_VALUE; /* Duty cycle value of output waveform */
	g_pwm_channel_led0.channel = PWM_CHANNEL_6; /* Set Channel 6 */
	pwm_channel_init(PWM, &g_pwm_channel_led0);
	
	pwm_channel_enable(PWM, PWM_CHANNEL_6); /* Enable PWM channels for LEDs */
	
	xQueue = xQueueCreate( 1, sizeof( long ) ); // Create queue of 1 long value (addresses), since queue is immediately read anyways
	vSemaphoreCreateBinary( xBinarySemaphore ); // Create Binary Semaphore
	
	BaseType_t res;
	res = xTaskCreate( vHandlerTask, "UPDATE PWM", 512, NULL, 1, NULL );
	
	if(res == pdPASS )
	{
		printf("Task created\n");
		
		/* ENCODER A INTERRUPT */
		pmc_enable_periph_clk(ID_PIOC); // Enable PIOC Controller
		pio_set_input(PIOC, PIO_PC28, PIO_PULLUP); // Configure PIN 3 as PULLUP
		pio_handler_set(PIOC, ID_PIOC, PIO_PC28, PIO_IT_EDGE, quadrature_handler); // Configure INTERRUPT HANDLER
		pio_enable_interrupt(PIOC, PIO_PC28); // Enable Interrupt for PIN 3
		NVIC_SetPriority(PIOC_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 2);
		NVIC_EnableIRQ(PIOC_IRQn); // Enable Interrupts C
		
		/* ENCODER A INTERRUPT */
		pmc_enable_periph_clk(ID_PIOB); // Enable PIOC Controller
		pio_set_input(PIOB, PIO_PB25, PIO_PULLUP); // Configure PIN 2 as PULLUP
		pio_handler_set(PIOB, ID_PIOB, PIO_PB25, PIO_IT_EDGE, quadrature_handler); // Configure INTERRUPT HANDLER
		pio_enable_interrupt(PIOB, PIO_PB25); // Enable Interrupt for PIN 2
		NVIC_SetPriority(PIOB_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 2);
		NVIC_EnableIRQ(PIOB_IRQn); // Enable Interrupts B
		
		/* PHOTO ELECTRIC INTERRUPT */
		pmc_enable_periph_clk(ID_PIOC);
		pio_set_input(PIOC, PIO_PC24, PIO_PULLUP);
		pio_handler_set(PIOC, ID_PIOC, PIO_PC24, PIO_IT_EDGE, photoelectric_handler);
		pio_enable_interrupt(PIOC, PIO_PC24);
		NVIC_SetPriority(PIOC_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 1);
		NVIC_EnableIRQ(PIOC_IRQn);
		
		vTaskStartScheduler();
	}

	for( ;; );
	return 0;
}

static void vHandlerTask( void *pvParameters )
{
	static int max_angle = 0; // Max angle of turning point (PENDULUM)
	static int max_step = 0; // Max pulses of turning point (MOTOR)
	static int pwm = 0; // Variable PWM value
	static int receive = 0;
	portBASE_TYPE xStatus;
	
	for(;;)
	{
		// SEMAPHORE AND QUEUE ONLY USED TO DISALLOW MOTOR TO MOVE INITIALLY BEFORE PHOTOELECTRIC SENSOR TRIGGERED
		xSemaphoreTake(xBinarySemaphore, portMAX_DELAY);
		xStatus = xQueueReceive(xQueue, &receive, 0);
		
		if(xStatus == pdPASS)
		{
			if (direction == -1) // CCW DETECTED
			{
				ioport_set_pin_level(MOTOR_DIR, HIGH); // SET CCW ROTATION
			}
			else if (direction == 1) // CW DETECTED
			{
				ioport_set_pin_level(MOTOR_DIR, LOW); // SET CW ROTATION
			}
			
			for (;;)
			{	
				// Check if pendulum has been manually stopped
				// The expected time pendulum returns to photoelectric sensor is given by half_period
				// If it exceeds this expected time, then motor should go back to 0 position 
				if (xTaskGetTickCount() - last_sense_time > (half_period + 50))
				{
					if (direction == -1) // CCW DETECTED
					{
						ioport_set_pin_level(MOTOR_DIR, LOW); // SET CW ROTATION TO RETURHN TO 0 POSITION
						
						if (measured_step >= 0) // Check if reached 0
						{
							pwm_channel_update_duty(PWM, &g_pwm_channel_led0, 0); // If reached 0, stop motor
						}
					}
					else
					{
						ioport_set_pin_level(MOTOR_DIR, HIGH); // SET CCW ROTATION TO RETURHN TO 0 POSITION
						
						if (measured_step <= 0) // Check if reached 0
						{
							pwm_channel_update_duty(PWM, &g_pwm_channel_led0, 0); // If reached 0, stop motor
						}
					}
				}
				else // If pendulum doesn't stop
				{
					// Variable PWM depends on time taken for long side of pendulum which depends on pendulum speed
					// Higher the pendulum speed, then higher then PWM
					if (time_taken < 16)
					{
						pwm = 14;
					}
					else if (time_taken >= 16 && time_taken < 20)
					{
						pwm = 12;
					}
					else if (time_taken >= 20 && time_taken < 26)
					{
						pwm = 10;
					}
					else if (time_taken >= 26 && time_taken < 40)
					{
						pwm = 9;
					}
					else if (time_taken >= 40)
					{
						pwm = 8;
					}
					
					// Update PWM value
					pwm_channel_update_duty(PWM, &g_pwm_channel_led0, pwm);
					
					// Compute maximum angle and maximum pulses for turning point
					max_angle = P1 * time_taken * time_taken * time_taken + P2 * time_taken * time_taken + P3 * time_taken + P4;
					max_step = max_angle * 98 / 360;
					
					printf("Time taken: %d. Measured Step: %d. Thres: %d. PWM: %d\n", time_taken, measured_step, max_step, pwm);
					
					// If reached turning point, then swing the other direction
					if (measured_step <= -max_step)
					{
						ioport_set_pin_level(MOTOR_DIR, LOW);
					}
					else if (measured_step >= max_step)
					{
						ioport_set_pin_level(MOTOR_DIR, HIGH);
					}
				}
			}
		}
	}
}

void photoelectric_handler(const uint32_t id, const uint32_t index)
{
	static int count = 0; // To catch false triggers
	
	static int previousState = 0; // Previous state of photoelectric sensor
	static int currentState = 0; // Current state of photoelectric sensor
	static int first_read = 1; // Flag to check if first pulse should be read
	
	static uint32_t prev_tA = 0; // To computer half-period
	static uint32_t tA_1 = 0;
	static uint32_t tA_2 = 0;
	static uint32_t tA = 0;
	
	static uint32_t tB_1 = 0;
	static uint32_t tB_2 = 0;
	static uint32_t tB = 0;
	
	portBASE_TYPE xStatus;
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
	
	// If interrupt other than photoelectric detected, exit handler
	if((id != ID_PIOC) || (index != PIO_PC24)){
		return;
	}
	
	// When power is turned on, interrupt triggers twice for some reason and will mess up the code, this is to catch those triggers
	if (count < 2)
	{
		count++;
		return;
	}

	currentState = ioport_get_pin_level(MY_PHOTOELECTRIC); // Read photoelectric sensor state
	
	// Detects rising edge
	if (currentState == 1 && previousState == 0)
	{
		// Checks if first pulse should be read
		if (first_read == 1)
		{
			tA_1 = xTaskGetTickCountFromISR();
		}
		else
		{
			tB_1 = xTaskGetTickCountFromISR();
		}
	}
	// Detects falling edge
	else if (currentState == 0 && previousState == 1)
	{
		// Checks if first pulse should be read
		if (first_read == 1)
		{
			tA_2 = xTaskGetTickCountFromISR();
			tA = tA_2 - tA_1;
			half_period = tA_2 - prev_tA;
			prev_tA = tA_2;
			first_read = 0; // Update flag for first read
		}
		else
		{
			tB_2 = xTaskGetTickCountFromISR();
			tB = tB_2 - tB_1;
			first_read = 1; // Update flag for first read
			
			if (tB < tA) // If longer end detected first
			{
				direction = -1; // Direction is CCW
				time_taken = tA;
			}
			else // If shorter end detected first
			{
				direction = 1; // Direction is CW
				time_taken = tB;
			}

			last_sense_time = tA_2; // Update last sense time of photoelectric sensor to make sure that pendulum was not stop manually
			
			// Push to queue and give semaphore
			xStatus = xQueueSendToBackFromISR(xQueue, &time_taken, 0);
			xSemaphoreGiveFromISR(xBinarySemaphore, &xHigherPriorityTaskWoken);
		}
	}
	
	previousState = currentState; // Update previous state
}

void quadrature_handler(const uint32_t id, const uint32_t index)
{
	static int prevENC_A = 0; // Previous encoder A value
	static int prevENC_B = 0; // Previous encoder B value
	static int ENC_A = 0; // Current encoder A value
	static int ENC_B = 0; // Current encoder B value
	
	// If Interrupt from Encoder A and B are detected
	if ((id == ID_PIOC) && (index == PIO_PC28) || (id == ID_PIOB) && (index == PIO_PB25))
	{
		ENC_A = ioport_get_pin_level(ENCODER_A); // Get ENCODER A value
		ENC_B = ioport_get_pin_level(ENCODER_B); // Get ENCODER B value
		
		// (1,0) -> (0,0) then motor moved CW
		if ((ENC_A == 0 && ENC_B == 0) && (prevENC_A == 1 && prevENC_B == 0))
		{
			measured_step += 1;
		}
		// (0,1) -> (0,0) then motor moved CCW
		else if ((ENC_A == 0 && ENC_B == 0) && (prevENC_A == 0 && prevENC_B == 1))
		{
			measured_step -= 1;
		}
		
		//printf("A: %d, B: %d\n", ENC_A, ENC_B);
		//printf("Measured Step: %d\n", measured_step);
		
		prevENC_A = ENC_A; // Update previous ENCODER A value
		prevENC_B = ENC_B; // Update previous ENCODER B value
	}
}

static void configure_console(void)
{
	const usart_serial_options_t uart_serial_options = {
		.baudrate = CONF_UART_BAUDRATE,
		.paritytype = CONF_UART_PARITY
	};

	/* Configure console UART. */
	sysclk_enable_peripheral_clock(CONSOLE_UART_ID);
	stdio_serial_init(CONSOLE_UART, &uart_serial_options);
}

void vApplicationIdleHook( void )
{
	
}
void vApplicationMallocFailedHook( void )
{

}
void vApplicationStackOverflowHook( xTaskHandle *pxTask, signed char *pcTaskName )
{

}
void vApplicationTickHook( void )
{
}