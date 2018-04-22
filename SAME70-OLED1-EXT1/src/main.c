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

#define BUZZ_PIO_ID	    ID_PIOA
#define BUZZ_PIO        PIOA
#define BUZZ_PIN		24
#define BUZZ_PIN_MASK   (1 << BUZZ_PIN)


/************************************************************************/
/* VAR globais                                                          */
/************************************************************************/
volatile uint8_t flag_led = 0;

volatile uint8_t flag_Alarm1 = 0;
volatile uint8_t flag_Alarm2 = 0;

volatile uint8_t flag_runTime = 0;
volatile uint8_t set_alarm = 1;

int alarm_done = 0;
int led_counter = 0;
int but_counter = 0;

int seg = 0;
int min1 = 0;
int minuto_alarme = 0;
/************************************************************************/
/* PROTOTYPES                                                           */
/************************************************************************/
void TC_init();

void LED_init(int estado);
void print(int h, int m);
void pin_toggle(Pio *pio, uint32_t mask);
void play_buzz(Pio *pio, uint32_t mask);

/************************************************************************/
/* Handlers                                                             */
/************************************************************************/

/**
*  Handle Interrupcao botao 1
*/

static void Button_0_Handler(uint32_t id, uint32_t mask){
	flag_led = 0;
	pio_set(LED_PIO, LED_PIN_MASK);	
	tc_stop(TC0,0);
}


static void Button_1_Handler(uint32_t id, uint32_t mask){
		seg = 0;
		min1 = 0;
		
		minuto_alarme = 0;
		
		set_alarm = 1;
		
		flag_led = 0;
		flag_Alarm1 = 0;
		flag_Alarm2 = 0;
	
		flag_runTime = 0;
		alarm_done = 0;
		
}


static void Button_2_Handler(uint32_t id, uint32_t mask){

	if (!flag_runTime){
		minuto_alarme = 30;
		flag_Alarm1   = !flag_Alarm1;
		flag_runTime  = !flag_runTime;
		alarm_done = 1;
	}
}

static void Button_3_Handler(uint32_t id, uint32_t mask){
		
	if (!flag_runTime){		
		minuto_alarme = 120;
		flag_Alarm2   = !flag_Alarm2;
		flag_runTime  = !flag_runTime;
		alarm_done = 1;
	}
}

/**
*  Interrupt handler for TC1 interrupt.
*/
void TC0_Handler(void){
	volatile uint32_t ul_dummy;

	/****************************************************************
	* Devemos indicar ao TC que a interrupo foi satisfeita.
	******************************************************************/
	ul_dummy = tc_get_status(TC0, 0);

	/* Avoid compiler warning */
	UNUSED(ul_dummy);

	if(flag_led)
		pin_toggle(LED_PIO, LED_PIN_MASK);
		//play_buzz(BUZZ_PIO, BUZZ_PIN_MASK);
	else{
		pio_set(LED_PIO, LED_PIN_MASK);
	}
	
}

void TC1_Handler(void){
	volatile uint32_t ul_dummy;

	/****************************************************************
	* Devemos indicar ao TC que a interrupo foi satisfeita.
	******************************************************************/
	ul_dummy = tc_get_status(TC0, 1);

	/* Avoid compiler warning */
	UNUSED(ul_dummy);
	if (flag_runTime && alarm_done){
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
			
			print(min1, seg);
			if (minuto_alarme == 0){
				TC_init(TC0, ID_TC0, 0, 8);
				flag_led = 1;
				minuto_alarme = -1;
			}
			
		}
	}
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

void play_buzz(Pio *pio, uint32_t mask){
	if(pio_get_output_data_status(pio, mask))
	pio_clear(pio, mask);
	else
	pio_set(pio,mask);
}

/**
* @Brief Inicializa o pino do LED
*/
void LED_init(int estado){
	pmc_enable_periph_clk(LED_PIO_ID);
	pio_set_output(LED_PIO, LED_PIN_MASK, estado, 0, 0 );
	NVIC_EnableIRQ(LED_PIO_ID);
	NVIC_SetPriority(LED_PIO_ID, 0);
};

void BUT_init(void){
	
	//Config. botao modo de entrada */
	pmc_enable_periph_clk(BUT_PIO_ID);
	pio_set_input(BUT_PIO, BUT_PIN_MASK, PIO_PULLUP | PIO_DEBOUNCE);

	pio_enable_interrupt(BUT_PIO, BUT_PIN_MASK);
	pio_handler_set(BUT_PIO, BUT_PIO_ID, BUT_PIN_MASK, PIO_IT_FALL_EDGE, Button_0_Handler);


	//Config. botao 1 modo de entrada */
	pmc_enable_periph_clk(BUT_1_PIO_ID);
	pio_set_input(BUT_1_PIO, BUT_1_PIN_MASK, PIO_PULLUP | PIO_DEBOUNCE);

	pio_enable_interrupt(BUT_1_PIO, BUT_1_PIN_MASK);
	pio_handler_set(BUT_1_PIO, BUT_1_PIO_ID, BUT_1_PIN_MASK, PIO_IT_FALL_EDGE, Button_1_Handler);

	//Config. botao 2 modo de entrada */
	pmc_enable_periph_clk(BUT_2_PIO_ID);
	pio_set_input(BUT_2_PIO, BUT_2_PIN_MASK, PIO_PULLUP | PIO_DEBOUNCE);

	pio_enable_interrupt(BUT_2_PIO, BUT_2_PIN_MASK);
	pio_handler_set(BUT_2_PIO, BUT_2_PIO_ID, BUT_2_PIN_MASK, PIO_IT_FALL_EDGE, Button_2_Handler);

	//Config. botao 3 modo de entrada */
	pmc_enable_periph_clk(BUT_3_PIO_ID);
	pio_set_input(BUT_3_PIO, BUT_3_PIN_MASK, PIO_PULLUP | PIO_DEBOUNCE);

	pio_enable_interrupt(BUT_3_PIO, BUT_3_PIN_MASK);
	pio_handler_set(BUT_3_PIO, BUT_3_PIO_ID, BUT_3_PIN_MASK, PIO_IT_FALL_EDGE, Button_3_Handler);

	delay_ms(400);
	
	pio_get_interrupt_status(BUT_1_PIO);
	NVIC_EnableIRQ(BUT_1_PIO_ID);
	NVIC_SetPriority(BUT_1_PIO_ID, 1);
	
	pio_get_interrupt_status(BUT_PIO);
	NVIC_EnableIRQ(BUT_PIO_ID);
	NVIC_SetPriority(BUT_PIO_ID, 1);
	
	pio_get_interrupt_status(BUT_2_PIO);
	NVIC_EnableIRQ(BUT_2_PIO_ID);
	NVIC_SetPriority(BUT_2_PIO_ID, 1);

	NVIC_EnableIRQ(BUT_3_PIO_ID);
	pio_get_interrupt_status(BUT_3_PIO);
	NVIC_SetPriority(BUT_3_PIO_ID, 1);
};

void print(int m, int s){
	
	uint8_t stringLCD[256];
	sprintf(stringLCD, "T:%02d:%02d", m, s);
	gfx_mono_draw_string(stringLCD, 0, 0, &sysfont);
}

void TC_init(Tc * TC, int ID_TC, int TC_CHANNEL, int freq){
	uint32_t ul_div;
	uint32_t ul_tcclks;
	uint32_t ul_sysclk = sysclk_get_cpu_hz();

	uint32_t channel = 1;

	/* Configura o PMC */
	pmc_enable_periph_clk(ID_TC);

	/** Configura o TC para operar em  4Mhz e interrupco no RC compare */
	tc_find_mck_divisor(freq, ul_sysclk, &ul_div, &ul_tcclks, ul_sysclk);
	tc_init(TC, TC_CHANNEL, ul_tcclks | TC_CMR_CPCTRG);
	tc_write_rc(TC, TC_CHANNEL, (ul_sysclk / ul_div) / freq);

	/* Configura e ativa interrupco no TC canal 0 */
	
	/* Interrupo no C */
	NVIC_EnableIRQ((IRQn_Type) ID_TC);
	tc_enable_interrupt(TC, TC_CHANNEL, TC_IER_CPCS);
	NVIC_SetPriority(ID_TC, 0);
	
	/* Inicializa o canal 0 do TC */
	tc_start(TC, TC_CHANNEL);
}

int main (void)
{
	board_init();
	sysclk_init();
	delay_init();
	gfx_mono_ssd1306_init();

	/* Configura os bot?es */
	BUT_init();
	
	/* Configura Leds */
	LED_init(1);
	
	
	
	uint8_t stringLCD[256];
	
	TC_init(TC0, ID_TC1, 1, 1);
	pmc_disable_periph_clk(ID_TC1);
	
	
	while(1) {
	
	//	if ((flag_runTime || flag_runTime2) && alarm_done){
			
	//	}
		if(set_alarm){
			gfx_mono_draw_string("Set_Alm", 0, 0, &sysfont);
			set_alarm = 0;
		}
		if (flag_Alarm1){
			sprintf(stringLCD, "Al%02d:%02d", 1, 0);
			gfx_mono_draw_string(stringLCD, 0, 0, &sysfont);
			flag_Alarm1 = !flag_Alarm1;
			pmc_enable_periph_clk(ID_TC1);
		}
		
		if (flag_Alarm2){
			sprintf(stringLCD, "Al%02d:%02d", 2, 0);
			gfx_mono_draw_string(stringLCD, 0, 0, &sysfont);
			flag_Alarm2 = !flag_Alarm2;
			pmc_enable_periph_clk(ID_TC1);
		}

	}
}

