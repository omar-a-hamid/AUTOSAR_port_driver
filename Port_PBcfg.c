/******************************************************************************
*
* Module: Port
*
* File Name: Port_PBCfg.C
*
* Description: Prebuild configure source file for TM4C123GH6PM Microcontroller - Port Driver.
*
* Author: Fady Ibrahim
******************************************************************************/

#include "Port.h"


/*
*Module version 1.0.0
*/
#define PORT_PBCFG_SW_MAJOR_VERSION (1U)
#define PORT_PBCFG_SW_MINOR_VERSION (0U)
#define PORT_PBCFG_SW_PACTH_VERSION (0U)

/*
*AUTOSAR version 4.0.3
*/
#define PORT_PBCFG_AR_RELEASE_MAJOR_VERSION (4U)
#define PORT_PBCFG_AR_RELEASE_MINOR_VERSION (0U)
#define PORT_PBCFG_AR_RELEASE_PATCH_VERSION (3U)

/* AUTOSAR Version checking between Port_PBcfg.c and Port.h files */
#if ((PORT_PBCFG_AR_RELEASE_MAJOR_VERSION != PORT_AR_RELEASE_MAJOR_VERSION)\
||  (PORT_PBCFG_AR_RELEASE_MINOR_VERSION != PORT_AR_RELEASE_MINOR_VERSION)\
  ||  (PORT_PBCFG_AR_RELEASE_PATCH_VERSION != PORT_AR_RELEASE_PATCH_VERSION))
#error "The AR version of PBcfg.c does not match the expected version"
#endif

/* Software Version checking between PORT_PBcfg.c and PORT.h files */
#if ((PORT_PBCFG_SW_MAJOR_VERSION != PORT_SW_MAJOR_VERSION)\
||  (PORT_PBCFG_SW_MINOR_VERSION != PORT_SW_MINOR_VERSION)\
  ||  (PORT_PBCFG_SW_PATCH_VERSION != PORT_SW_PATCH_VERSION))
#error "The SW version of PBcfg.c does not match the expected version"
#endif

const Port_ConfigType Port_Configuration={
  PortConfig_PORTA_NUM,PortConfig_PIN0_NUM,INPUT,OFF,STD_LOW,PortConfig_Mode_DEFUALT,STD_ON,STD_ON,
  PortConfig_PORTA_NUM,PortConfig_PIN1_NUM,INPUT,OFF,STD_LOW,PortConfig_Mode_DEFUALT,STD_ON,STD_ON,
  PortConfig_PORTA_NUM,PortConfig_PIN2_NUM,INPUT,OFF,STD_LOW,PortConfig_Mode_DEFUALT,STD_ON,STD_ON,
  PortConfig_PORTA_NUM,PortConfig_PIN3_NUM,INPUT,OFF,STD_LOW,PortConfig_Mode_DEFUALT,STD_ON,STD_ON,
  PortConfig_PORTA_NUM,PortConfig_PIN4_NUM,INPUT,OFF,STD_LOW,PortConfig_Mode_DEFUALT,STD_ON,STD_ON,
  PortConfig_PORTA_NUM,PortConfig_PIN5_NUM,INPUT,OFF,STD_LOW,PortConfig_Mode_DEFUALT,STD_ON,STD_ON,
  PortConfig_PORTA_NUM,PortConfig_PIN6_NUM,INPUT,OFF,STD_LOW,PortConfig_Mode_DEFUALT,STD_ON,STD_ON,
  PortConfig_PORTA_NUM,PortConfig_PIN7_NUM,INPUT,OFF,STD_LOW,PortConfig_Mode_DEFUALT,STD_ON,STD_ON,
  PortConfig_PORTB_NUM,PortConfig_PIN0_NUM,INPUT,OFF,STD_LOW,PortConfig_Mode_DEFUALT,STD_ON,STD_ON,
  PortConfig_PORTB_NUM,PortConfig_PIN1_NUM,INPUT,OFF,STD_LOW,PortConfig_Mode_DEFUALT,STD_ON,STD_ON,
  PortConfig_PORTB_NUM,PortConfig_PIN2_NUM,INPUT,OFF,STD_LOW,PortConfig_Mode_DEFUALT,STD_ON,STD_ON,
  PortConfig_PORTB_NUM,PortConfig_PIN3_NUM,INPUT,OFF,STD_LOW,PortConfig_Mode_DEFUALT,STD_ON,STD_ON,
  PortConfig_PORTB_NUM,PortConfig_PIN4_NUM,INPUT,OFF,STD_LOW,PortConfig_Mode_DEFUALT,STD_ON,STD_ON,
  PortConfig_PORTB_NUM,PortConfig_PIN5_NUM,INPUT,OFF,STD_LOW,PortConfig_Mode_DEFUALT,STD_ON,STD_ON,
  PortConfig_PORTB_NUM,PortConfig_PIN6_NUM,INPUT,OFF,STD_LOW,PortConfig_Mode_DEFUALT,STD_ON,STD_ON,
  /*
  PortConfig_PORTB_NUM,PortConfig_PIN7_NUM,INPUT,OFF,STD_LOW,PortConfig_Mode_DEFUALT,STD_ON,STD_ON,
  PortConfig_PORTC_NUM,PortConfig_PIN0_NUM,INPUT,OFF,STD_LOW,PortConfig_Mode_DEFUALT,STD_ON,STD_ON,
  PortConfig_PORTC_NUM,PortConfig_PIN1_NUM,INPUT,OFF,STD_LOW,PortConfig_Mode_DEFUALT,STD_ON,STD_ON,
  PortConfig_PORTC_NUM,PortConfig_PIN2_NUM,INPUT,OFF,STD_LOW,PortConfig_Mode_DEFUALT,STD_ON,STD_ON,
  PortConfig_PORTC_NUM,PortConfig_PIN3_NUM,INPUT,OFF,STD_LOW,PortConfig_Mode_DEFUALT,STD_ON,STD_ON,
  PortConfig_PORTC_NUM,PortConfig_PIN4_NUM,INPUT,OFF,STD_LOW,PortConfig_Mode_DEFUALT,STD_ON,STD_ON,
  PortConfig_PORTC_NUM,PortConfig_PIN5_NUM,INPUT,OFF,STD_LOW,PortConfig_Mode_DEFUALT,STD_ON,STD_ON,
  PortConfig_PORTC_NUM,PortConfig_PIN6_NUM,INPUT,OFF,STD_LOW,PortConfig_Mode_DEFUALT,STD_ON,STD_ON,
  PortConfig_PORTC_NUM,PortConfig_PIN7_NUM,INPUT,OFF,STD_LOW,PortConfig_Mode_DEFUALT,STD_ON,STD_ON,
  */
  PortConfig_PORTD_NUM,PortConfig_PIN0_NUM,INPUT,OFF,STD_LOW,PortConfig_Mode_DEFUALT,STD_ON,STD_ON,
  PortConfig_PORTD_NUM,PortConfig_PIN1_NUM,INPUT,OFF,STD_LOW,PortConfig_Mode_DEFUALT,STD_ON,STD_ON,
  PortConfig_PORTD_NUM,PortConfig_PIN2_NUM,INPUT,OFF,STD_LOW,PortConfig_Mode_DEFUALT,STD_ON,STD_ON,
  PortConfig_PORTD_NUM,PortConfig_PIN3_NUM,INPUT,OFF,STD_LOW,PortConfig_Mode_DEFUALT,STD_ON,STD_ON,
  PortConfig_PORTD_NUM,PortConfig_PIN4_NUM,INPUT,OFF,STD_LOW,PortConfig_Mode_DEFUALT,STD_ON,STD_ON,
  PortConfig_PORTD_NUM,PortConfig_PIN5_NUM,INPUT,OFF,STD_LOW,PortConfig_Mode_DEFUALT,STD_ON,STD_ON,
  PortConfig_PORTD_NUM,PortConfig_PIN6_NUM,INPUT,OFF,STD_LOW,PortConfig_Mode_DEFUALT,STD_ON,STD_ON,
  PortConfig_PORTD_NUM,PortConfig_PIN7_NUM,INPUT,OFF,STD_LOW,PortConfig_Mode_DEFUALT,STD_ON,STD_ON,
  PortConfig_PORTE_NUM,PortConfig_PIN0_NUM,INPUT,OFF,STD_LOW,PortConfig_Mode_DEFUALT,STD_ON,STD_ON,
  PortConfig_PORTE_NUM,PortConfig_PIN1_NUM,INPUT,OFF,STD_LOW,PortConfig_Mode_DEFUALT,STD_ON,STD_ON,
  PortConfig_PORTE_NUM,PortConfig_PIN2_NUM,INPUT,OFF,STD_LOW,PortConfig_Mode_DEFUALT,STD_ON,STD_ON,
  PortConfig_PORTE_NUM,PortConfig_PIN3_NUM,INPUT,OFF,STD_LOW,PortConfig_Mode_DEFUALT,STD_ON,STD_ON,
  PortConfig_PORTE_NUM,PortConfig_PIN4_NUM,INPUT,OFF,STD_LOW,PortConfig_Mode_DEFUALT,STD_ON,STD_ON,
  PortConfig_PORTE_NUM,PortConfig_PIN5_NUM,INPUT,OFF,STD_LOW,PortConfig_Mode_DEFUALT,STD_ON,STD_ON,
  PortConfig_PORTF_NUM,PortConfig_PIN0_NUM,INPUT,OFF,STD_LOW,PortConfig_Mode_DEFUALT,STD_ON,STD_ON,//sw2
  PortConfig_PORTF_NUM,PortConfig_PIN1_NUM,OUTPUT,OFF,STD_LOW,PortConfig_Mode_DIGITAL_OUTPUT,STD_ON,STD_ON,//led
  PortConfig_PORTF_NUM,PortConfig_PIN2_NUM,INPUT,OFF,STD_LOW,PortConfig_Mode_DEFUALT,STD_ON,STD_ON,//led
  PortConfig_PORTF_NUM,PortConfig_PIN3_NUM,INPUT,OFF,STD_LOW,PortConfig_Mode_DEFUALT,STD_ON,STD_ON,//led
  PortConfig_PORTF_NUM,PortConfig_PIN4_NUM,INPUT,PULL_UP,STD_LOW,PortConfig_Mode_DIGITAL_INPUT,STD_ON,STD_ON,//sw1
};

/* Description: Structure to configure each individual PIN:
*	1. the PORT Which the pin belongs to. 0, 1, 2, 3, 4 or 5
*	2. the number of the pin in the PORT.
*      3. the direction of pin --> INPUT or OUTPUT
*      4. the internal resistor --> Disable, Pull up or Pull down
*      5. initial value of the pin
*      6. the mode of the pin --> ADC,DIO,UART,SSI,I2C,PWM,PHB,IDX,T2CCP,WTCCP,CAN,USB,NMI,C,TRD
*      7. pin mode changeability --> MODE_CHANGE_OFF or  STD_ON
*      8. pin direction changeability --> DIRECTION_CHANGE_OFF or STD_ON
*/