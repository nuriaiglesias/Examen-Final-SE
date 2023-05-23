#include "MKL46Z4.h"
#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "board.h"
#include "pin_mux.h"
#include "clock_config.h"

// LED (RG)
// LED_GREEN = PTD5
// LED_RED = PTE29

int verde = 0;
int rojo = 0;

// LED_GREEN = PTD5
void led_green_init()
{
  SIM->COPC = 0;
  SIM->SCGC5 |= SIM_SCGC5_PORTD_MASK;
  PORTD->PCR[5] = PORT_PCR_MUX(1);
  GPIOD->PDDR |= (1 << 5);
  GPIOD->PSOR = (1 << 5);
}

// LED_RED = PTE29
void led_red_init()
{
  SIM->COPC = 0;
  SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;
  PORTE->PCR[29] = PORT_PCR_MUX(1);
  GPIOE->PDDR |= (1 << 29);
  GPIOE->PSOR = (1 << 29);
}

void Green_LED_On(void)
{
  GPIOD->PCOR = (1u << 5);
}

void Red_LED_On(void)
{
  GPIOE->PCOR = (1u << 29);
}

void Green_LED_Off(void)
{
  GPIOD->PSOR = (1u << 5);
}

void Red_LED_Off(void)
{
  GPIOE->PSOR = (1u << 29);
}

void sw1_button_init(void)
{
  SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;
  PORTC->PCR[3] = PORT_PCR_MUX(1) | PORT_PCR_PE(1) | PORT_PCR_PS(1) | PORT_PCR_IRQC(10); 
  GPIOC->PDDR &= ~(1 << 3);
  PORTC->ISFR |= (1 << 3);
  NVIC_SetPriority(PORTC_PORTD_IRQn, 0); 
}

void sw2_button_init(void)
{
  SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;
  PORTC->PCR[12] = PORT_PCR_MUX(1) | PORT_PCR_PE(1) | PORT_PCR_PS(1) | PORT_PCR_IRQC(10); // set port c pin 12 as GPIO, Pull enable and select
  GPIOC->PDDR &= ~(1 << 12);
  PORTC->ISFR |= (1 << 12);     // Detect interruption
  NVIC_SetPriority(PORTC_PORTD_IRQn, 1);
}

int check_SW1(void)
{ // returns 1 if SW1 is pressed and 0 if switch is not pressed
  if ((0x00000008 & PTC->PDIR) == 0x00000008)
  {
    return 0;
  }
  else
  {
    return 1;
  }
}

int check_SW2(void)
{ // returns 1 if SW2 is pressed and 0 if switch is not pressed
  if ((0x00001000 & PTC->PDIR) == 0x00001000)
  {
    return 0;
  }
  else
  {
    return 1;
  }
}


void PORTD_Int_Handler(void){
      if (check_SW1()) // SW1 has been pressed
      { 
        Green_LED_On();
        Red_LED_Off();
        while (check_SW1()); // Wait until the button stops being pressed
        PORTC->PCR[3] &= ~PORT_PCR_ISF_MASK;
      }

      if (check_SW2()) // SW2 has been pressed
      { 
        Red_LED_On();
        Green_LED_Off();
        while (check_SW2());
        PORTC->PCR[12] &= ~PORT_PCR_ISF_MASK;
      }
}

int main(void)
{
  led_green_init();
  led_red_init();
  sw1_button_init();
  sw2_button_init();
  NVIC_EnableIRQ(PORTC_PORTD_IRQn);

  /* Init board hardware. */
  BOARD_InitPins();
  BOARD_BootClockRUN();
  BOARD_InitDebugConsole();

  while(1){
  char command[256];

  PRINTF("$ ");  // Muestra el prompt

  // Lee el comando introducido por el usuario
  SCANF("%s", command);

  if (strcmp(command, "led1") == 0)
  {
    PRINTF("%s", command);
    verde = 1;
    rojo = 0;
    LED_GREEN_ON();
    LED_RED_OFF();
  }
  else if (strcmp(command, "led2") == 0)
  {
    PRINTF("%s", command);
    verde = 0;
    rojo = 1;
    LED_RED_ON();
    LED_GREEN_OFF();
  }
  else if (strcmp(command, "off") == 0)
  {
    PRINTF("%s", command);
    verde = 0;
    rojo = 0;
    LED_GREEN_OFF();
    LED_RED_OFF();
  }
  else if (strcmp(command, "toggle") == 0)
  {
    PRINTF("%s", command);
    if(verde == 1){
        LED_GREEN_OFF();
        LED_RED_ON();
        verde = 0;
        rojo = 1;
    }else if(rojo == 1){
        LED_GREEN_ON();
        LED_RED_OFF();
        verde = 1;
        rojo = 0;
    }
  }
  }

    return 0;
}