/**
 *  Computa??o Embarcada
 *  Insper
 *  Rafael Corsi - rafael.corsi@insper.edu.br
 *  C?digo exemplo uso I2C
 *  Maio - 2019
 *
 *  Bug conhecido : spi read deve ser executado duas vezes ?
 *      - funcao afetada : mcu6050_i2c_bus_read()
 *
 *  Conectar :
 *            MCU  |  SAME70-XPLD | 
 *           ----------------------
 *            SDA  |   EXT2-11    |  PA3
 *            SCL  |   EXT2-12    |  PA4
 *            GND  |   EXT2-19    |
 *            VCC  |   EXT2-20    | 
 * 
 * TODO: calibracao
 *       bug dois reads i2c
 */

 
#include "asf.h"
#include "bme280.h"
#include "conf_example.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

/**
 * LEDs
 */
#define LED_PIO_ID		  ID_PIOC
#define LED_PIO         PIOC
#define LED_PIN		      8
#define LED_PIN_MASK    (1<<LED_PIN)

/**
 * Bot?o
 */
#define BUT_PIO_ID              ID_PIOA
#define BUT_PIO                 PIOA
#define BUT_PIN		            11
#define BUT_PIN_MASK            (1 << BUT_PIN)
#define BUT_DEBOUNCING_VALUE    79

/** 
 * 
 */
#define TWIHS_MCU6050_ID    ID_TWIHS0 
#define TWIHS_MCU6050       TWIHS0  

#define YEAR        2019
#define MONTH       6
#define DAY         13
#define WEEK        24
#define HOUR        16
#define MINUTE      0
#define SECOND      0

uint32_t timestamp = 1559417273;

QueueHandle_t xQueueTerminal;
QueueHandle_t xQueueSdcard;

SemaphoreHandle_t xSemaphoreT;
SemaphoreHandle_t xSemaphoreSD;

/************************************************************************/
/* VAR globais                                                          */
/************************************************************************/

float rangePerDigit ; // 2G
//const float rangePerDigit = 9.80665f ; // 2G

volatile uint8_t flag_led0 = 1;

 int16_t  accX, accY, accZ;
volatile uint8_t  accXHigh, accYHigh, accZHigh;
volatile uint8_t  accXLow,  accYLow,  accZLow;

/************************************************************************/
/* PROTOTYPES                                                           */
/************************************************************************/

void BUT_init(void);
void RTC_init(void);
void LED_init(int estado);
void pin_toggle(Pio *pio, uint32_t mask);

/************************************************************************/
/* BME                                                                  */
/************************************************************************/ 
/*	
 *  \Brief: The function is used as I2C bus init
 */
void bme280_i2c_bus_init(void)
{
	twihs_options_t bno055_option;
	pmc_enable_periph_clk(TWIHS_MCU6050_ID);

	/* Configure the options of TWI driver */
	bno055_option.master_clk = sysclk_get_cpu_hz();
	bno055_option.speed      = 10000;
	twihs_master_init(TWIHS_MCU6050, &bno055_option);
}

uint8_t bme280_i2c_read_reg(uint CHIP_ADDRESS, uint reg_address, char *value){
  	uint i = 1;
    
  	  twihs_packet_t p_packet;
  	  p_packet.chip         = CHIP_ADDRESS;//BME280_ADDRESS;
  	  p_packet.addr_length  = 0;

  	  char data = reg_address; //BME280_CHIP_ID_REG;
  	  p_packet.buffer       = &data;
  	  p_packet.length       = 1;
  	
  	  if(twihs_master_write(TWIHS_MCU6050, &p_packet) != TWIHS_SUCCESS)
  	    return 1;

  	  p_packet.addr_length  = 0;
  	  p_packet.length       = 1;
      p_packet.buffer       = value;

  	  if(twihs_master_read(TWIHS_MCU6050, &p_packet) != TWIHS_SUCCESS)
  	    return 1;
        
    return 0;  
}

int8_t bme280_i2c_config_temp(void){
  	int32_t ierror = 0x00;
  	
  	twihs_packet_t p_packet;
  	p_packet.chip         = BME280_ADDRESS;//BME280_ADDRESS;
    p_packet.addr[0]      = BME280_CTRL_MEAS_REG;
  	p_packet.addr_length  = 1;

  	char data = 0b00100111; //BME280_CHIP_ID_REG;
  	p_packet.buffer       = &data;
  	p_packet.length       = 1;
  	
  	if(twihs_master_write(TWIHS_MCU6050, &p_packet) != TWIHS_SUCCESS)
  	return 1;
}

int8_t bme280_i2c_read_temp(uint *temp)
{
	int32_t ierror = 0x00;
  char tmp[3];
  
  bme280_i2c_read_reg(BME280_ADDRESS, BME280_TEMPERATURE_MSB_REG, &tmp[2]);
  bme280_i2c_read_reg(BME280_ADDRESS, BME280_TEMPERATURE_MSB_REG, &tmp[2]);
  
  bme280_i2c_read_reg(BME280_ADDRESS, BME280_TEMPERATURE_LSB_REG, &tmp[1]);
  bme280_i2c_read_reg(BME280_ADDRESS, BME280_TEMPERATURE_LSB_REG, &tmp[1]);

  *temp = tmp[2] << 8 | tmp[1];
	return 0;
}

int8_t bme280_i2c_read_humd(uint *humd)
{
	int32_t ierror = 0x00;
	char hum[3];
	
	bme280_i2c_read_reg(BME280_ADDRESS, BME280_HUMIDITY_MSB_REG, &hum[2]);
	bme280_i2c_read_reg(BME280_ADDRESS, BME280_HUMIDITY_MSB_REG, &hum[2]);
	
	bme280_i2c_read_reg(BME280_ADDRESS, BME280_HUMIDITY_LSB_REG, &hum[1]);
	bme280_i2c_read_reg(BME280_ADDRESS, BME280_HUMIDITY_LSB_REG, &hum[1]);

	*humd = hum[2] << 8 | hum[1];
	return 0;
}

int8_t bme280_i2c_read_prss(uint *prss)
{
	int32_t ierror = 0x00;
	char prs[3];
	
	bme280_i2c_read_reg(BME280_ADDRESS, BME280_PRESSURE_MSB_REG, &prs[2]);
	bme280_i2c_read_reg(BME280_ADDRESS, BME280_PRESSURE_MSB_REG, &prs[2]);
	
	bme280_i2c_read_reg(BME280_ADDRESS, BME280_PRESSURE_LSB_REG, &prs[1]);
	bme280_i2c_read_reg(BME280_ADDRESS, BME280_PRESSURE_LSB_REG, &prs[1]);

	*prss = prs[2] << 8 | prs[1];
	return 0;
}

uint8_t bme280_validate_id(void){
  char id;
  bme280_i2c_read_reg(BME280_ADDRESS, BME280_CHIP_ID_REG, &id );
  if (bme280_i2c_read_reg(BME280_ADDRESS, BME280_CHIP_ID_REG, &id )) 
    return 1;
  if (id != 0x60)
    return 1;
  return 0; 
}

/************************************************************************/
/* Handlers                                                             */
/************************************************************************/

/**
 *  Handle Interrupcao botao 1
 */
static void Button1_Handler(uint32_t id, uint32_t mask)
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  xSemaphoreGiveFromISR(xSemaphoreT, &xHigherPriorityTaskWoken);
  BaseType_t xHigherPriorityTaskWoken2 = pdFALSE;
  xSemaphoreGiveFromISR(xSemaphoreSD, &xHigherPriorityTaskWoken2);
  
  pin_toggle(PIOD, (1<<28));
  pin_toggle(LED_PIO, LED_PIN_MASK);
}

void RTC_Handler(void)
{
	uint32_t ul_status = rtc_get_status(RTC);

	/*
	*  Verifica por qual motivo entrou
	*  na interrupcao, se foi por segundo
	*  ou Alarm
	*/
	if ((ul_status & RTC_SR_SEC) == RTC_SR_SEC) {
		rtc_clear_status(RTC, RTC_SCCR_SECCLR);
		timestamp++;
	}
	
	rtc_clear_status(RTC, RTC_SCCR_ACKCLR);
	rtc_clear_status(RTC, RTC_SCCR_TIMCLR);
	rtc_clear_status(RTC, RTC_SCCR_CALCLR);
	rtc_clear_status(RTC, RTC_SCCR_TDERRCLR);
	
}

/************************************************************************/
/* RTOS                                                                 */
/************************************************************************/

#define TASK_SENSOR_STACK_SIZE             (2*1024/sizeof(portSTACK_TYPE))
#define TASK_SENSOR_STACK_PRIORITY         (tskIDLE_PRIORITY)

#define TASK_TERMINAL_STACK_SIZE            (2*1024/sizeof(portSTACK_TYPE))
#define TASK_TERMINAL_STACK_PRIORITY        (tskIDLE_PRIORITY)

#define TASK_SDCARD_STACK_SIZE              (2*1024/sizeof(portSTACK_TYPE))
#define TASK_SDCARD_STACK_PRIORITY          (tskIDLE_PRIORITY)


void task_sensor(void){	
	xQueueTerminal = xQueueCreate( 10, sizeof( uint32_t ) );
	xQueueSdcard = xQueueCreate( 10, sizeof( uint32_t ) );
	
	/* Inicializa i2c */
	printf("Inicializando bus i2c \n");
	bme280_i2c_bus_init();
	delay_ms(10);
	
	/* verificando presenca do chip */
	while(bme280_validate_id()){
		printf("Chip nao encontrado\n");
		delay_ms(200);
	}
	
	printf("Chip encontrado, inicializando temperatura \n");
	bme280_i2c_config_temp();
	
	uint temp;
	uint humd;
	uint prss;
	char frase[50];
	
	while (true) {
		if (bme280_i2c_read_temp(&temp)){
			sprintf(frase, "erro readinG temperature \n");
		}else{
			sprintf(frase, "Temperatura: %d \n", temp/100);
		}
		xQueueSendToBackFromISR(xQueueTerminal, &frase, 0);
		xQueueSendToBackFromISR(xQueueSdcard, &frase, 0);
		
		if (bme280_i2c_read_humd(&humd)){
			sprintf(frase, "erro reading humidade \n");
		}else{
			sprintf(frase, "Humidade: %d \n", humd/100);
		}
		xQueueSendToBackFromISR(xQueueTerminal, &frase, 0);
		xQueueSendToBackFromISR(xQueueSdcard, &frase, 0);
		
		if (bme280_i2c_read_prss(&prss)){
			sprintf(frase, "erro reading pressao \n");
		}else{
			sprintf(frase, "Pressao: %d \n", humd/100);
		}
		xQueueSendToBackFromISR(xQueueTerminal, &frase, 0);
		xQueueSendToBackFromISR(xQueueSdcard, &frase, 0);
		//vTaskDelay(4000);
	}
}

void task_terminal(void){
	printf("OLA T");
	xSemaphoreT = xSemaphoreCreateBinary();
	
	if (xSemaphoreT == NULL)
		printf("falha em criar o semaforo \n");
		
	int print_mode = 1;
	char frase[50];
	
	while (true) {
		if( xSemaphoreTake(xSemaphoreT, ( TickType_t ) 10) == pdTRUE ){
			print_mode = !print_mode;
		}
		if (xQueueReceive( xQueueTerminal, &(frase), ( TickType_t )  500 / portTICK_PERIOD_MS)) {
			printf("%s", frase);
		}
		//printf("Starting ADC\n");
		vTaskDelay(4000);
	}
}

void task_sdcard(void){
	printf("OLA SD");
	xSemaphoreSD = xSemaphoreCreateBinary();
	
	if (xSemaphoreSD == NULL)
		printf("falha em criar o semaforo \n");
	
	char test_file_name[] = "0:sd_mmc_test.txt";
	Ctrl_status status;
	FRESULT res;
	FATFS fs;
	FIL file_object;
	
	/* Initialize SD MMC stack */
	sd_mmc_init();

	printf("\x0C\n\r-- SD/MMC/SDIO Card Example on FatFs --\n\r");
	printf("-- Compiled: %s %s --\n\r", __DATE__, __TIME__);
	
	uint written;
	FIL fdst;
	DWORD size;
	int save_mode = 0;
	
	while (true) {
		if( xSemaphoreTake(xSemaphoreSD, ( TickType_t ) 10) == pdTRUE ){
			save_mode = !save_mode;
		}
		
		if(save_mode){
			printf("Please plug an SD, MMC or SDIO card in slot.\n\r");

			/* Wait card present and ready */
			do {
				status = sd_mmc_test_unit_ready(0);
				if (CTRL_FAIL == status) {
					printf("Card install FAIL\n\r");
					printf("Please unplug and re-plug the card.\n\r");
					while (CTRL_NO_PRESENT != sd_mmc_check(0)) {
					}
				}
			} while (CTRL_GOOD != status);

			printf("Mount disk (f_mount)...\r\n");
			memset(&fs, 0, sizeof(FATFS));
			res = f_mount(LUN_ID_SD_MMC_0_MEM, &fs);
			if (FR_INVALID_DRIVE == res) {
				printf("[FAIL] res %d\r\n", res);
				goto main_end_of_test;
			}
			printf("[OK]\r\n");

			printf("Create a file (f_open)...\r\n");
			test_file_name[0] = LUN_ID_SD_MMC_0_MEM + '0';
			res = f_open(&file_object, (char const *)test_file_name, FA_OPEN_ALWAYS | FA_WRITE);
		
			if (res != FR_OK) {
				printf("[FAIL] res %d\r\n", res);
				goto main_end_of_test;
			}
			printf("[OK]\r\n");

			main_end_of_test:
			printf("Please unplug the card.\n\r");
			while (CTRL_NO_PRESENT != sd_mmc_check(0)) {
				printf("Write to test file (f_puts)...\r\n");
				size = (&file_object)->fsize;
				res = f_lseek(&file_object,size);
				if (0 == f_write(&file_object, "Test SD/MMC stack\n", 18, &written)) {
					f_close(&file_object);
					printf("[FAIL]\r\n");
					goto main_end_of_test;
				}
				//written += 18;
				printf("[OK]\r\n");
				f_close(&file_object);
				printf("Test is successful.\n\r");
				delay_s(2);
			}
		}
		
		//vTaskDelay(4000);
	}
}


/************************************************************************/
/* Funcoes                                                              */
/************************************************************************/

void RTC_init(){
	/* Configura o PMC */
	pmc_enable_periph_clk(ID_RTC);

	/* Default RTC configuration, 24-hour mode */
	rtc_set_hour_mode(RTC, 0);

	/* Configura data e hora manualmente */
	rtc_set_date(RTC, YEAR, MONTH, DAY, WEEK);
	rtc_set_time(RTC, HOUR, MINUTE, SECOND);

	NVIC_DisableIRQ(RTC_IRQn);
	NVIC_ClearPendingIRQ(RTC_IRQn);
	NVIC_SetPriority(RTC_IRQn, 6);
	NVIC_EnableIRQ(RTC_IRQn);

	/* Ativa interrupcao via alarme */
	rtc_enable_interrupt(RTC,  RTC_IER_SECEN);

}

/** 
 *  Toggle pin controlado pelo PIO (out)
 */
void pin_toggle(Pio *pio, uint32_t mask){
   if(pio_get_output_data_status(pio, mask))
    pio_clear(pio, mask);
   else
    pio_set(pio,mask);
}

/**
 * @Brief Inicializa o pino do BUT
 */
void BUT_init(void){
    /* config. pino botao em modo de entrada */
    pmc_enable_periph_clk(BUT_PIO_ID);
    pio_set_input(BUT_PIO, BUT_PIN_MASK, PIO_PULLUP | PIO_DEBOUNCE);
    
    /* config. interrupcao em borda de descida no botao do kit */
    /* indica funcao (but_Handler) a ser chamada quando houver uma interrup??o */
    pio_enable_interrupt(BUT_PIO, BUT_PIN_MASK);
    pio_handler_set(BUT_PIO, BUT_PIO_ID, BUT_PIN_MASK, PIO_IT_FALL_EDGE, Button1_Handler);
    
    /* habilita interrup?c?o do PIO que controla o botao */
    /* e configura sua prioridade                        */
    NVIC_EnableIRQ(BUT_PIO_ID);
    NVIC_SetPriority(BUT_PIO_ID, 1);
};

/**
 * @Brief Inicializa o pino do LED
 */
void LED_init(int estado){
    pmc_enable_periph_clk(LED_PIO_ID);
    pio_set_output(LED_PIO, LED_PIN_MASK, estado, 0, 0 );
};

/**
 * \brief Configure UART console.
 * BaudRate : 115200
 * 8 bits
 * 1 stop bit
 * sem paridade
 */
static void configure_console(void)
{

  /* Configura USART1 Pinos */
  sysclk_enable_peripheral_clock(ID_PIOB);
  sysclk_enable_peripheral_clock(ID_PIOA);
  pio_set_peripheral(PIOB, PIO_PERIPH_D, PIO_PB4);  // RX
  pio_set_peripheral(PIOA, PIO_PERIPH_A, PIO_PA21); // TX
 	MATRIX->CCFG_SYSIO |= CCFG_SYSIO_SYSIO4;
 
	const usart_serial_options_t uart_serial_options = {
		.baudrate   = CONF_UART_BAUDRATE,
		.charlength = CONF_UART_CHAR_LENGTH,
		.paritytype = CONF_UART_PARITY,
		.stopbits   = CONF_UART_STOP_BITS,
	};

	/* Configure console UART. */
	sysclk_enable_peripheral_clock(CONSOLE_UART_ID);
	stdio_serial_init(CONF_UART, &uart_serial_options);
}

/************************************************************************/
/* RTOS hooks                                                           */
/************************************************************************/

/**
 * \brief Called if stack overflow during execution
 */
extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,
		signed char *pcTaskName)
{
	printf("stack overflow %x %s\r\n", pxTask, (portCHAR *)pcTaskName);
	/* If the parameters have been corrupted then inspect pxCurrentTCB to
	 * identify which task has overflowed its stack.
	 */
	for (;;) {
	}
}

/**
 * \brief This function is called by FreeRTOS idle task
 */
extern void vApplicationIdleHook(void)
{
	pmc_sleep(SAM_PM_SMODE_SLEEP_WFI);
}

/**
 * \brief This function is called by FreeRTOS each tick
 */
extern void vApplicationTickHook(void)
{
}

extern void vApplicationMallocFailedHook(void)
{
	/* Called if a call to pvPortMalloc() fails because there is insufficient
	free memory available in the FreeRTOS heap.  pvPortMalloc() is called
	internally by FreeRTOS API functions that create tasks, queues, software
	timers, and semaphores.  The size of the FreeRTOS heap is set by the
	configTOTAL_HEAP_SIZE configuration constant in FreeRTOSConfig.h. */

	/* Force an assert. */
	configASSERT( ( volatile void * ) NULL );
}

/************************************************************************/
/* Main Code	                                                        */
/************************************************************************/
int main(void){
	
////////////////////////////////////////////////////////////////////////////
  RTC_init();
////////////////////////////////////////////////////////////////////////////
  
  /* buffer para recebimento de dados */
  uint8_t bufferRX[100];
  uint8_t bufferTX[100];
  
  uint8_t rtn;

  /* Initialize the SAM system */
  sysclk_init();
  board_init();
   
  /* Disable the watchdog */
  WDT->WDT_MR = WDT_MR_WDDIS;
  
  /* Inicializa com serial com PC*/
  configure_console();
  printf("Demo do sensor BME280, sem calibracao! \n"); 
  
  /* Inicializa funcao de delay */
  delay_init( sysclk_get_cpu_hz());
   
   const usart_serial_options_t usart_serial_options = {
	   .baudrate   = CONF_TEST_BAUDRATE,
	   .charlength = CONF_TEST_CHARLENGTH,
	   .paritytype = CONF_TEST_PARITY,
	   .stopbits   = CONF_TEST_STOPBITS,
   };
  
   irq_initialize_vectors();
   cpu_irq_enable();

   stdio_serial_init(CONF_TEST_USART, &usart_serial_options);
  
  if (xTaskCreate(task_sensor, "sensor", TASK_SENSOR_STACK_SIZE, NULL, TASK_SENSOR_STACK_PRIORITY, NULL) != pdPASS) {
	  printf("Failed to create test led task\r\n");
  }

  /* Create task to handler touch */
  if (xTaskCreate(task_terminal, "terminal", TASK_TERMINAL_STACK_SIZE, NULL, TASK_TERMINAL_STACK_PRIORITY, NULL) != pdPASS) {
	  printf("Failed to create test led task\r\n");
  }
  
  /* Create task to handler LCD */
  if (xTaskCreate(task_sdcard, "sdcard", TASK_SDCARD_STACK_SIZE, NULL, TASK_SDCARD_STACK_PRIORITY, NULL) != pdPASS) {
	  printf("Failed to create test led task\r\n");
  }
	/* Start the scheduler. */
	vTaskStartScheduler();
	
	/* Configura Leds */
	LED_init(1);
	
	/* Configura os bot?es */
	BUT_init();

	while (1) {
		pin_toggle(LED_PIO, LED_PIN_MASK);
		delay_ms(1000);
	}
	return 0;
}