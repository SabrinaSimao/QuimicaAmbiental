/**
 *	Avaliacao intermediaria 
 *	Computacao - Embarcada
 *        Abril - 2018
 * Objetivo : criar um Relogio + Timer 
 * Materiais :
 *    - SAME70-XPLD
 *    - OLED1
 *
 * Exemplo OLED1 por Eduardo Marossi
 * Modificacoes: 
 *    - Adicionado nova fonte com escala maior
 */
#include <asf.h>

#include "oled/gfx_mono_ug_2832hsweg04.h"
#include "oled/gfx_mono_text.h"
#include "oled/sysfont.h"

/************************************************************************/
/* DEFINES                                                              */
/************************************************************************/

/**
 *  Informacoes para o RTC
 *  poderia ser extraida do __DATE__ e __TIME__
 *  ou ser atualizado pelo PC.
 */
#define YEAR        2010
#define MOUNTH      1
#define DAY         1
#define WEEK        1
#define HOUR        0
#define MINUTE      0
#define SECOND      0

#define BUT_PIO_ID			  ID_PIOA
#define BUT_PIO				  PIOA
#define BUT_PIN				  11
#define BUT_PIN_MASK		  (1 << BUT_PIN)

#define BUT_1_PIO_ID ID_PIOD
#define BUT_1_PIO PIOD
#define BUT_1_PIN 28
#define BUT_1_PIN_MASK (1<<BUT_1_PIN)

#define BUT_2_PIO_ID ID_PIOC
#define BUT_2_PIO PIOC
#define BUT_2_PIN 31
#define BUT_2_PIN_MASK (1<<BUT_2_PIN)

#define BUT_3_PIO_ID ID_PIOA
#define BUT_3_PIO PIOA
#define BUT_3_PIN 19
#define BUT_3_PIN_MASK (1<<BUT_3_PIN)

#define LED_PIO_ID	   ID_PIOA
#define LED_PIO        PIOA
#define LED_PIN		   0
#define LED_PIN_MASK   (1<<LED_PIN)

#define SEN_PIO_ID	   ID_PIOA
#define SEN_PIO        PIOA
#define SEN_PIN		   4
#define SEN_PIN_MASK   (1<<SEN_PIN)

/************************************************************************/
/* VAR globais                                                          */
/************************************************************************/
volatile uint8_t flag_led0 = 1;
volatile uint8_t flag_but_1 = 0;
volatile uint8_t flag_led_2 = 0;
volatile uint8_t flag_led_3 = 0;
volatile uint8_t flag_lcd = 0;
volatile uint8_t flag_print = 0;
volatile uint8_t flag_print_1 = 0;
volatile uint8_t flag_print_2 = 0;
int led_counter = 0;
int but_counter = 0;
int minuto = MINUTE+1;
int hora = HOUR;
int seg = 0;
int min1 = 0;
int minuto_alarme = 0;
int count_alarme_s = 0;
int count_alarme_m = 0;
/************************************************************************/
/* PROTOTYPES                                                           */
/************************************************************************/

void BUT_init(void);
void LED_init(int estado);
void TC_init(Tc * TC, int ID_TC, int TC_CHANNEL, int freq);
void RTC_init(void);
void print(int h, int m);
void pin_toggle(Pio *pio, uint32_t mask);

/************************************************************************/
/* Handlers                                                             */
/************************************************************************/

/**
*  Handle Interrupcao botao 1
*/
static void Button_1_Handler(uint32_t id, uint32_t mask){
	if(!flag_but_1){
		seg = 0;
		min1 = 0;
		count_alarme_m = 0;
		count_alarme_s = 0;
		flag_but_1 =!flag_but_1;
		print(min1,seg);
	}
}
static void Button_2_Handler(uint32_t id, uint32_t mask){
	
	minuto_alarme = 60;
	flag_print_1 =! flag_print_1;
	
}
static void Button_3_Handler(uint32_t id, uint32_t mask){
	
	minuto_alarme = 120;
	flag_print_1 =! flag_print_1;
	
}

static void Button_0_Handler(uint32_t id, uint32_t mask){
		flag_led0 =! flag_led0;
		pmc_disable_periph_clk(ID_TC1);
		tc_stop(TC0,1);
}

/**
*  Interrupt handler for TC1 interrupt.
*/
void TC1_Handler(void){
	volatile uint32_t ul_dummy;

	/****************************************************************
	* Devemos indicar ao TC que a interrupo foi satisfeita.
	******************************************************************/
	ul_dummy = tc_get_status(TC0, 1);

	/* Avoid compiler warning */
	UNUSED(ul_dummy);

	if(flag_led0)
		pin_toggle(LED_PIO, LED_PIN_MASK);
		
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
* \brief Interrupt handler for the RTC. Refresh the display.
*/
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
				
		int y = pio_get(SEN_PIO, PIO_INPUT, SEN_PIN_MASK);
		
		//molhado
		if(!y){
			seg++;
			if(seg == 60){
				min1++;
				seg = 0;
			} 
			if (minuto_alarme > 0){
				minuto_alarme--;
			}
			
			flag_print =! flag_print;
			
			if (minuto_alarme == 0){
				pmc_enable_periph_clk(ID_TC1);
				flag_led0 =! flag_led0;
			}
		
		}
		
	}
	
	/* Time or date alarm */
	if ((ul_status & RTC_SR_ALARM) == RTC_SR_ALARM) {
			rtc_clear_status(RTC, RTC_SCCR_ALRCLR);

	}
	rtc_clear_status(RTC, RTC_SCCR_ACKCLR);
	rtc_clear_status(RTC, RTC_SCCR_TIMCLR);
	rtc_clear_status(RTC, RTC_SCCR_CALCLR);
	rtc_clear_status(RTC, RTC_SCCR_TDERRCLR);	
}

/**
* Configura o RTC para funcionar com interrupcao de alarme
*/
void RTC_init(){
	/* Configura o PMC */
	pmc_enable_periph_clk(ID_RTC);

	/* Default RTC configuration, 24-hour mode */
	rtc_set_hour_mode(RTC, 0);

	/* Configura data e hora manualmente */
	rtc_set_date(RTC, YEAR, MOUNTH, DAY, WEEK);
	rtc_set_time(RTC, HOUR, MINUTE, SECOND);

	/* Configure RTC interrupts */
	NVIC_DisableIRQ(RTC_IRQn);
	NVIC_ClearPendingIRQ(RTC_IRQn);
	NVIC_SetPriority(RTC_IRQn, 1);
	NVIC_EnableIRQ(RTC_IRQn);

	/* Ativa interrupcao via alarme */
	rtc_enable_interrupt(RTC,  RTC_IER_ALREN|RTC_IER_SECEN);

}

/**
* Configura TimerCounter (TC) para gerar uma interrupcao no canal (ID_TC e TC_CHANNEL)
* na taxa de especificada em freq.
*/
void TC_init(Tc * TC, int ID_TC, int TC_CHANNEL, int freq){
	uint32_t ul_div;
	uint32_t ul_tcclks;
	uint32_t ul_sysclk = sysclk_get_cpu_hz();

	uint32_t channel = 1;

	pmc_enable_periph_clk(ID_TC);

	/** Configura o TC para operar em  4Mhz e interrupco no RC compare */
	tc_find_mck_divisor(freq, ul_sysclk, &ul_div, &ul_tcclks, ul_sysclk);
	tc_init(TC, TC_CHANNEL, ul_tcclks | TC_CMR_CPCTRG);
	tc_write_rc(TC, TC_CHANNEL, (ul_sysclk / ul_div) / freq);

	/* Configura e ativa interrupco no TC canal 0 */
	/* Interrupo no C */
	
	NVIC_EnableIRQ((IRQn_Type) ID_TC);
	NVIC_SetPriority(ID_TC, 0);
	tc_enable_interrupt(TC, TC_CHANNEL, TC_IER_CPCS);
	/* Inicializa o canal 0 do TC */
	tc_start(TC0, 1);
	
}


/**
* @Brief Inicializa o pino do LED
*/
void LED_init(int estado){
	pmc_enable_periph_clk(LED_PIO_ID);
	pio_set_output(LED_PIO, LED_PIN_MASK, estado, 0, 0 );
};

void BUT_init(void){
	pmc_enable_periph_clk(BUT_PIO_ID);
	pio_set_input(BUT_PIO, BUT_PIN_MASK, PIO_PULLUP | PIO_DEBOUNCE);

	pio_enable_interrupt(BUT_PIO, BUT_PIN_MASK);
	pio_handler_set(BUT_PIO, BUT_PIO_ID, BUT_PIN_MASK, PIO_IT_FALL_EDGE, Button_0_Handler);

	NVIC_EnableIRQ(BUT_PIO_ID);
	NVIC_SetPriority(BUT_PIO_ID, 1);
};

void BUT_1_init(void){
	pmc_enable_periph_clk(BUT_1_PIO_ID);
	pio_set_input(BUT_1_PIO, BUT_1_PIN_MASK, PIO_PULLUP | PIO_DEBOUNCE);

	pio_enable_interrupt(BUT_1_PIO, BUT_1_PIN_MASK);
	pio_handler_set(BUT_1_PIO, BUT_1_PIO_ID, BUT_1_PIN_MASK, PIO_IT_FALL_EDGE, Button_1_Handler);

	NVIC_EnableIRQ(BUT_1_PIO_ID);
	NVIC_SetPriority(BUT_1_PIO_ID, 1);
};

void BUT_2_init(void){
	/* config. pino botao em modo de entrada */
	pmc_enable_periph_clk(BUT_2_PIO_ID);
	pio_set_input(BUT_2_PIO, BUT_2_PIN_MASK, PIO_PULLUP | PIO_DEBOUNCE);

	pio_enable_interrupt(BUT_2_PIO, BUT_2_PIN_MASK);
	pio_handler_set(BUT_2_PIO, BUT_2_PIO_ID, BUT_2_PIN_MASK, PIO_IT_FALL_EDGE, Button_2_Handler);

	NVIC_EnableIRQ(BUT_2_PIO_ID);
	NVIC_SetPriority(BUT_2_PIO_ID, 1);
};

void BUT_3_init(void){

	pmc_enable_periph_clk(BUT_3_PIO_ID);
	pio_set_input(BUT_3_PIO, BUT_3_PIN_MASK, PIO_PULLUP | PIO_DEBOUNCE);

	pio_enable_interrupt(BUT_3_PIO, BUT_3_PIN_MASK);
	pio_handler_set(BUT_3_PIO, BUT_3_PIO_ID, BUT_3_PIN_MASK, PIO_IT_FALL_EDGE, Button_3_Handler);

	NVIC_EnableIRQ(BUT_3_PIO_ID);
	NVIC_SetPriority(BUT_3_PIO_ID, 1);
};

void print(int m, int s){
	
	uint8_t stringLCD[256];
	sprintf(stringLCD, "T:%02d:%02d", m, s);
	gfx_mono_draw_string(stringLCD, 0, 0, &sysfont);
	
}

int main (void)
{
	board_init();
	sysclk_init();
	delay_init();
	gfx_mono_ssd1306_init();

	/* Configura os bot?es */
	BUT_init();
	BUT_1_init();
	BUT_2_init();
	BUT_3_init();
	
	/* Configura Leds */
	LED_init(1);
	
	/** Configura RTC */
	RTC_init();

	
	/** Configura timer TC0, canal 1 */
	TC_init(TC0, ID_TC1, 1, 4);

	flag_led0 = 0;
	flag_but_1 = 0;
	flag_led_2 = 0;
	flag_led_3 = 0;
	flag_print = 0;
	flag_print_1 = 0;
	flag_print_2 = 0;
/*	int led_counter = 0;
	int but_counter = 0;
	int minuto = MINUTE+1;
	int hora = HOUR;
	int seg = 0;
	int min1 = 0;
	int minuto_alarme =0;
	int count_alarme_s = 0;
	int count_alarme_m = 0;
	*/
	while(1) {
	
		if (flag_print){
			print(min1,seg);
			flag_print =! flag_print;
		}
		
		if (flag_print_1){
			uint8_t stringLCD[256];
			sprintf(stringLCD, "Al%02d:%02d", 1, 0);
			gfx_mono_draw_string(stringLCD, 0, 0, &sysfont);
			flag_print_1 =! flag_print_1;
		}
		if (flag_print_2){
			uint8_t stringLCD[256];
			sprintf(stringLCD, "Al%02d:%02d", 2, 0);
			gfx_mono_draw_string(stringLCD, 0, 0, &sysfont);
			flag_print_2 =! flag_print_2;
		}

	}
}
