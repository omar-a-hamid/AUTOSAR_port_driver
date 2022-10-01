 /******************************************************************************
 *
 * Module: Port
 *
 * File Name: Port_Cfg.h
 *
 * Description: Pre-Compile Configuration Header file for TM4C123GH6PM Microcontroller - Port Driver
 *
 * Author: Fady Ibrahim
 ******************************************************************************/

#ifndef PORT_CFG_H
#define PORT_CFG_H

/*
 * Module Version 1.0.0
 */
#define PORT_CFG_SW_MAJOR_VERSION              (1U)
#define PORT_CFG_SW_MINOR_VERSION              (0U)
#define PORT_CFG_SW_PATCH_VERSION              (0U)

/*
 * AUTOSAR Version 4.0.3
 */
#define PORT_CFG_AR_RELEASE_MAJOR_VERSION     (4U)
#define PORT_CFG_AR_RELEASE_MINOR_VERSION     (0U)
#define PORT_CFG_AR_RELEASE_PATCH_VERSION     (3U)

/* Pre-compile option for Development Error Detect */
#define PORT_DEV_ERROR_DETECT                (STD_ON)

/* Pre-compile option for Version Info API */
#define PORT_VERSION_INFO_API                (STD_ON)

/* Pre-compile option for presence of Dio_FlipChannel API */
#define PORT_FLIP_CHANNEL_API                (STD_ON)

/**/
#define PORT_DEFUALT_CHANNEL_LEVEL           (STD_HIGH)



/* Number of the configured Dio Channels */
#define PORT_CONFIGURED_CHANNLES              (2U)


/* Number of the configured Port Channels */
#define PORT_CONFIGURED_CHANNLES                (34U)

/* Number of the configured Port mods */
#define PORT_CONFIGURED_MODES                   (42U)


#define PortConfig_PORTA_NUM (uint8)0
#define PortConfig_PORTB_NUM (uint8)1
#define PortConfig_PORTC_NUM (uint8)2
#define PortConfig_PORTD_NUM (uint8)3
#define PortConfig_PORTE_NUM (uint8)4
#define PortConfig_PORTF_NUM (uint8)5

/*pins configuring with id */

#define PortConfig_PIN0_NUM (uint8)0
#define PortConfig_PIN1_NUM (uint8)1
#define PortConfig_PIN2_NUM (uint8)2
#define PortConfig_PIN3_NUM (uint8)3
#define PortConfig_PIN4_NUM (uint8)4
#define PortConfig_PIN5_NUM (uint8)5
#define PortConfig_PIN6_NUM (uint8)6
#define PortConfig_PIN7_NUM (uint8)7

/* Modes configuration with id based on GPIO modes distribution*/


#define PortConfig_Mode_DEFUALT         (uint8)0
#define PortConfig_Mode_ANALOG 		    (uint8)1

#define PortConfig_Mode_UART0TX 		(uint8)2
#define PortConfig_Mode_UART1TX 		(uint8)3
#define PortConfig_Mode_UART2TX 		(uint8)4
#define PortConfig_Mode_UART3TX 		(uint8)5
#define PortConfig_Mode_UART4TX 		(uint8)6
#define PortConfig_Mode_UART5TX 		(uint8)7
#define PortConfig_Mode_UART6TX 		(uint8)8
#define PortConfig_Mode_UART7TX 		(uint8)9

#define PortConfig_Mode_UART0RX 		(uint8)10
#define PortConfig_Mode_UART1RX 		(uint8)11
#define PortConfig_Mode_UART2RX 		(uint8)12
#define PortConfig_Mode_UART3RX 		(uint8)13
#define PortConfig_Mode_UART4RX 		(uint8)14
#define PortConfig_Mode_UART5RX 		(uint8)15
#define PortConfig_Mode_UART6RX 		(uint8)16
#define PortConfig_Mode_UART7RX 		(uint8)17

#define PortConfig_Mode_SSI0TX	    	(uint8)18
#define PortConfig_Mode_SSI1TX 	    	(uint8)19
#define PortConfig_Mode_SSI2TX 	    	(uint8)20
#define PortConfig_Mode_SSI3TX 	    	(uint8)21

#define PortConfig_Mode_SSI0RX	    	(uint8)22
#define PortConfig_Mode_SSI1RX 	    	(uint8)23
#define PortConfig_Mode_SSI2RX 	    	(uint8)24
#define PortConfig_Mode_SSI3RX 	    	(uint8)25

#define PortConfig_Mode_I2C0 	    	(uint8)26
#define PortConfig_Mode_I2C1 	    	(uint8)27
#define PortConfig_Mode_I2C2 		    (uint8)28
#define PortConfig_Mode_I2C3 	    	(uint8)29


#define PortConfig_Mode_CAN0TX 	    	(uint8)30
#define PortConfig_Mode_CAN1TX 	    	(uint8)31
#define PortConfig_Mode_CAN0RX 		    (uint8)32
#define PortConfig_Mode_CAN1RX 		    (uint8)33



#define PortConfig_Mode_M0FAULT 	    (uint8)34

#define PortConfig_Mode_M0PWM 		    (uint8)35
#define PortConfig_Mode_M1PWM 		    (uint8)36

#define PortConfig_Mode_TIMER 		    (uint8)37

#define PortConfig_Mode_DIGITAL_USB     (uint8)38

#define PortConfig_Mode_IDX 		    (uint8)39

#define PortConfig_Mode_DIGITAL_INPUT   (uint8)40
#define PortConfig_Mode_DIGITAL_OUTPUT  (uint8)41


/* Modes configuration ctl register*/


#define PortConfig_CTL_REG_DEFUALT         (uint8)0
#define PortConfig_CTL_REG_DIO             (uint8)0

#define PortConfig_CTL_REG_ADC 		    (uint8)0

#define PortConfig_CTL_REG_UART_DEFUALT 	(uint8)1 
#define PortConfig_CTL_REG_UART_EXCEPTION 	(uint8)2 //c45


#define PortConfig_CTL_REG_SSI_DEFUALT     (uint8)2
#define PortConfig_CTL_REG_SSI_EXCEPTION  (uint8)1//PORT d


#define PortConfig_CTL_REG_I2C	    	    (uint8)3

#define PortConfig_CTL_REG_M0 		    (uint8)4

#define PortConfig_CTL_REG_M1 		    (uint8)5

#define PortConfig_CTL_REG_QEI 		    (uint8)6


#define PortConfig_CTL_REG_TIMER 		    (uint8)7

#define PortConfig_CTL_REG_DIGITAL_USB     (uint8)8

#define PortConfig_CTL_REG_CAN_DEFUALT     (uint8)8
#define PortConfig_CTL_REG_CAN_EXCEPTION  	(uint8)3








#define PortConfig_Mode_NMI 		(uint8)8

/*configuring pins with ports*/

/*PORT A*/
#define PortConfig_PORTA_PIN0_ID_INDEX (uint8)0x00
#define PortConfig_PORTA_PIN1_ID_INDEX (uint8)0x01
#define PortConfig_PORTA_PIN2_ID_INDEX (uint8)0x02
#define PortConfig_PORTA_PIN3_ID_INDEX (uint8)0x03
#define PortConfig_PORTA_PIN4_ID_INDEX (uint8)0x04
#define PortConfig_PORTA_PIN5_ID_INDEX (uint8)0x05
#define PortConfig_PORTA_PIN6_ID_INDEX (uint8)0x06
#define PortConfig_PORTA_PIN7_ID_INDEX (uint8)0x07

/*port B*/

#define PortConfig_PORTB_PIN0_ID_INDEX (uint8)0x08
#define PortConfig_PORTB_PIN1_ID_INDEX (uint8)0x09
#define PortConfig_PORTB_PIN2_ID_INDEX (uint8)0x0A
#define PortConfig_PORTB_PIN3_ID_INDEX (uint8)0x0B
#define PortConfig_PORTB_PIN4_ID_INDEX (uint8)0x0C
#define PortConfig_PORTB_PIN5_ID_INDEX (uint8)0x0D
#define PortConfig_PORTB_PIN6_ID_INDEX (uint8)0x0E
#define PortConfig_PORTB_PIN7_ID_INDEX (uint8)0x0F


/*PORT C*/
#define PortConfig_PORTC_PIN0_ID_INDEX (uint8)0x10
#define PortConfig_PORTC_PIN1_ID_INDEX (uint8)0x11
#define PortConfig_PORTC_PIN2_ID_INDEX (uint8)0x12
#define PortConfig_PORTC_PIN3_ID_INDEX (uint8)0x13
#define PortConfig_PORTC_PIN4_ID_INDEX (uint8)0x14
#define PortConfig_PORTC_PIN5_ID_INDEX (uint8)0x15
#define PortConfig_PORTC_PIN6_ID_INDEX (uint8)0x16
#define PortConfig_PORTC_PIN7_ID_INDEX (uint8)0x17

/*port D*/

#define PortConfig_PORTD_PIN0_ID_INDEX (uint8)0x18
#define PortConfig_PORTD_PIN1_ID_INDEX (uint8)0x19
#define PortConfig_PORTD_PIN2_ID_INDEX (uint8)0x1A
#define PortConfig_PORTD_PIN3_ID_INDEX (uint8)0x1B
#define PortConfig_PORTD_PIN4_ID_INDEX (uint8)0x1C
#define PortConfig_PORTD_PIN5_ID_INDEX (uint8)0x1D
#define PortConfig_PORTD_PIN6_ID_INDEX (uint8)0x1E
#define PortConfig_PORTD_PIN7_ID_INDEX (uint8)0x1F
/*PORT E*/
#define PortConfig_PORTE_PIN0_ID_INDEX (uint8)0x20
#define PortConfig_PORTE_PIN1_ID_INDEX (uint8)0x21
#define PortConfig_PORTE_PIN2_ID_INDEX (uint8)0x22
#define PortConfig_PORTE_PIN3_ID_INDEX (uint8)0x23
#define PortConfig_PORTE_PIN4_ID_INDEX (uint8)0x24
#define PortConfig_PORTE_PIN5_ID_INDEX (uint8)0x25


/*port F*/

#define PortConfig_PORTF_PIN0_ID_INDEX (uint8)0x26
#define PortConfig_PORTF_PIN1_ID_INDEX (uint8)0x27
#define PortConfig_PORTF_PIN2_ID_INDEX (uint8)0x28
#define PortConfig_PORTF_PIN3_ID_INDEX (uint8)0x29
#define PortConfig_PORTF_PIN4_ID_INDEX (uint8)0x2A





/* Channel Index in the array of structures in Dio_PBcfg.c */
// #define DioConf_LED1_CHANNEL_ID_INDEX        (uint8)0x00
// #define DioConf_SW1_CHANNEL_ID_INDEX         (uint8)0x01

/* DIO Configured Port ID's  */
// #define DioConf_LED1_PORT_NUM                (Dio_PortType)5 /* PORTF */
// #define DioConf_SW1_PORT_NUM                 (Dio_PortType)5 /* PORTF */

/* DIO Configured Channel ID's */
// #define DioConf_LED1_CHANNEL_NUM             (Dio_ChannelType)1 /* Pin 1 in PORTF */
// #define DioConf_SW1_CHANNEL_NUM              (Dio_ChannelType)4 /* Pin 4 in PORTF */

#endif /* PORT_CFG_H */
