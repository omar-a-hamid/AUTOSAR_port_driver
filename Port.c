/******************************************************************************
 *
 * Module: Port
 *
 * File Name: Port.c
 *
 * Description: Source file for TM4C123GH6PM Microcontroller - Port Driver.
 *
 * Author: Mohamed Tarek
 ******************************************************************************/

#include "Port.h"
#include "Common_Macros.h"

#include "tm4c123gh6pm_registers.h"

#if (PORT_DEV_ERROR_DETECT == STD_ON)

#include "Det.h"
/* AUTOSAR Version checking between Det and Dio Modules */
#if ((DET_AR_MAJOR_VERSION != PORT_AR_RELEASE_MAJOR_VERSION) || (DET_AR_MINOR_VERSION != PORT_AR_RELEASE_MINOR_VERSION) || (DET_AR_PATCH_VERSION != PORT_AR_RELEASE_PATCH_VERSION))
#error "The AR version of Det.h does not match the expected version"
#endif

#endif
STATIC const Port_ConfigType  *  channels;
STATIC uint8 Port_Status = PORT_NOT_INITIALIZED;

/************************************************************************************
 * Service Name: Port_Init
 * Sync/Async: Synchronous
 * Service ID: 0x00
 * Reentrancy: reentrant
 * Parameters (in): ConfigPtr - Pointer to post-build configuration data
 * Parameters (inout): None
 * Parameters (out): None
 * Return value: None
 * Description: Function to Setup the pin configuration:
 *              - Setup the pin as Digital GPIO pin
 *              - Setup the direction of the GPIO pin
 *              - Provide initial value for o/p pin
 *              - Setup the internal resistor for i/p pin
 ************************************************************************************/
void Port_Init(const Port_ConfigType *ConfigPtr)
{

    volatile uint32 *PortGpio_Ptr = NULL_PTR; /* point to the required Port Registers base address */
    volatile uint32 delay = 0;
    channels = ConfigPtr; // to keep pointer value if needed in run time
#if (PORT_DEV_ERROR_DETECT == STD_ON)
    /* check if the input configuration pointer is not a NULL_PTR */
    if (NULL_PTR == ConfigPtr)
    {
        Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_INIT_SID,
                        PORT_E_PARAM_CONFIG);
    }
    else
#endif

    {
        Port_Status = PORT_INITIALIZED;
        /* Enable clock for PORT and allow time for clock to start*/

        SYSCTL_REGCGC2_REG |= (0x3F);
        delay = SYSCTL_REGCGC2_REG;

        // ConfigPtr->Channels[0].port_num;
        int pin = 0;
        for (pin = 0; pin < PORT_CONFIGURED_CHANNLES; pin++)
        {
            switch (ConfigPtr->Channels[pin].port_num)
            {
            case 0:
                PortGpio_Ptr = (volatile uint32 *)GPIO_PORTA_BASE_ADDRESS; /* PORTA Base Address */
                break;
            case 1:
                PortGpio_Ptr = (volatile uint32 *)GPIO_PORTB_BASE_ADDRESS; /* PORTB Base Address */
                break;
            case 2:
                PortGpio_Ptr = (volatile uint32 *)GPIO_PORTC_BASE_ADDRESS; /* PORTC Base Address */
                break;
            case 3:
                PortGpio_Ptr = (volatile uint32 *)GPIO_PORTD_BASE_ADDRESS; /* PORTD Base Address */
                break;
            case 4:
                PortGpio_Ptr = (volatile uint32 *)GPIO_PORTE_BASE_ADDRESS; /* PORTE Base Address */
                break;
            case 5:
                PortGpio_Ptr = (volatile uint32 *)GPIO_PORTF_BASE_ADDRESS; /* PORTF Base Address */
                break;
            }

            if (((ConfigPtr->Channels[pin].port_num == 3) && (ConfigPtr->Channels[pin].pin_num == 7)) || ((ConfigPtr->Channels[pin].port_num == 5) && (ConfigPtr->Channels[pin].pin_num == 0))) /* PD7 or PF0 */
            {
                *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_LOCK_REG_OFFSET) = 0x4C4F434B;                                 /* Unlock the GPIOCR register */
                SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_COMMIT_REG_OFFSET), ConfigPtr->Channels[pin].pin_num); /* Set the corresponding bit in GPIOCR register to allow changes on this pin */
            }
            else if ((ConfigPtr->Channels[pin].port_num == 2) && (ConfigPtr->Channels[pin].pin_num <= 3)) /* PC0 to PC3 */
            {
                /* Do Nothing ...  this is the JTAG pins */
            }
            else
            {
                /* Do Nothing ... No need to unlock the commit register for this pin */
            }

            if (ConfigPtr->Channels[pin].pin_mode == PortConfig_Mode_DEFUALT)
            {

                CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET), ConfigPtr->Channels[pin].pin_num);   /* Clear the corresponding bit in the GPIOAMSEL register to disable analog functionality on this pin */
                CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET), ConfigPtr->Channels[pin].pin_num);          /* Disable Alternative function for this pin by clear the corresponding bit in GPIOAFSEL register */
                *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(0x0000000F << (ConfigPtr->Channels[pin].pin_num * 4)); /* Clear the PMCx bits for this pin */
                SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIR_REG_OFFSET), ConfigPtr->Channels[pin].pin_num);                 /* Set the corresponding bit in the GPIODIR register to configure it as output pin */

                if (PORT_DEFUALT_CHANNEL_LEVEL == STD_HIGH)
                {
                    SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DATA_REG_OFFSET), ConfigPtr->Channels[pin].pin_num); /* Set the corresponding bit in the GPIODATA register to provide initial value 1 */
                }
                else
                {
                    CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DATA_REG_OFFSET), ConfigPtr->Channels[pin].pin_num); /* Clear the corresponding bit in the GPIODATA register to provide initial value 0 */
                }
            }
            else if (ConfigPtr->Channels[pin].pin_mode == PortConfig_Mode_ANALOG)
            {

                *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(0x0000000F << (ConfigPtr->Channels[pin].pin_num * 4)); /* Clear the PMCx bits for this pin */
                SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET), ConfigPtr->Channels[pin].pin_num);     /* Clear the corresponding bit in the GPIOAMSEL register to disable analog functionality on this pin */
                SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET), ConfigPtr->Channels[pin].pin_num);            /* Disable Alternative function for this pin by clear the corresponding bit in GPIOAFSEL register */
                CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET), ConfigPtr->Channels[pin].pin_num);    /* Clear the corresponding bit in the GPIODATA register to provide initial value 0 */
            }
            else
            {
                /*all modes */
                CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET), ConfigPtr->Channels[pin].pin_num); /* Clear the corresponding bit in the GPIOAMSEL register to disable analog functionality on this pin */
                SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET), ConfigPtr->Channels[pin].pin_num);          /* Disable Alternative function for this pin by clear the corresponding bit in GPIOAFSEL register */
                switch (ConfigPtr->Channels[pin].pin_mode)
                {
                case PortConfig_Mode_UART0TX:
                case PortConfig_Mode_UART0RX:

                case PortConfig_Mode_UART2TX:
                case PortConfig_Mode_UART2RX:
                case PortConfig_Mode_UART3TX:
                case PortConfig_Mode_UART3RX:
                case PortConfig_Mode_UART4TX:
                case PortConfig_Mode_UART4RX:
                case PortConfig_Mode_UART5TX:
                case PortConfig_Mode_UART5RX:
                case PortConfig_Mode_UART6TX:
                case PortConfig_Mode_UART6RX:
                case PortConfig_Mode_UART7TX:
                case PortConfig_Mode_UART7RX:
                    *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (PortConfig_CTL_REG_UART_DEFUALT << (ConfigPtr->Channels[pin].pin_num * 4)); /* Clear the PMCx bits for this pin */

                    break;
                case PortConfig_Mode_UART1TX:
                case PortConfig_Mode_UART1RX:
                    if (ConfigPtr->Channels[pin].port_num == 2)
                    {
                        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (PortConfig_CTL_REG_UART_EXCEPTION << (ConfigPtr->Channels[pin].pin_num * 4)); /* Clear the PMCx bits for this pin */
                    }
                    else
                    {
                        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (PortConfig_CTL_REG_UART_DEFUALT << (ConfigPtr->Channels[pin].pin_num * 4)); /* Clear the PMCx bits for this pin */
                    }
                    break;

                case PortConfig_Mode_SSI0TX:
                case PortConfig_Mode_SSI0RX:

                case PortConfig_Mode_SSI1TX:
                case PortConfig_Mode_SSI1RX:

                case PortConfig_Mode_SSI2TX:
                case PortConfig_Mode_SSI2RX:
                    *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (PortConfig_CTL_REG_SSI_DEFUALT << (ConfigPtr->Channels[pin].pin_num * 4)); /* Clear the PMCx bits for this pin */
                    break;
                case PortConfig_Mode_SSI3TX:
                case PortConfig_Mode_SSI3RX:
                    *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (PortConfig_CTL_REG_SSI_EXCEPTION << (ConfigPtr->Channels[pin].pin_num * 4)); /* Clear the PMCx bits for this pin */
                    break;
                case PortConfig_Mode_I2C0:
                case PortConfig_Mode_I2C1:
                case PortConfig_Mode_I2C2:
                case PortConfig_Mode_I2C3:
                    *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (PortConfig_CTL_REG_I2C << (ConfigPtr->Channels[pin].pin_num * 4)); /* Clear the PMCx bits for this pin */
                    break;

                case PortConfig_Mode_CAN0TX:
                case PortConfig_Mode_CAN0RX:

                    if (ConfigPtr->Channels[pin].port_num == PortConfig_PORTF_NUM)
                    {

                        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (PortConfig_CTL_REG_CAN_EXCEPTION << (ConfigPtr->Channels[pin].pin_num * 4)); /* Clear the PMCx bits for this pin */
                    }
                    else
                    {
                        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (PortConfig_CTL_REG_CAN_DEFUALT << (ConfigPtr->Channels[pin].pin_num * 4)); /* Clear the PMCx bits for this pin */
                    }
                    break;

                case PortConfig_Mode_CAN1TX:
                case PortConfig_Mode_CAN1RX:
                    *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (PortConfig_CTL_REG_CAN_DEFUALT << (ConfigPtr->Channels[pin].pin_num * 4)); /* Clear the PMCx bits for this pin */
                    break;

                case PortConfig_Mode_M0PWM:
                case PortConfig_Mode_M0FAULT:

                    *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (PortConfig_CTL_REG_M0 << (ConfigPtr->Channels[pin].pin_num * 4)); /* Clear the PMCx bits for this pin */
                    break;

                case PortConfig_Mode_M1PWM:
                    *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (PortConfig_CTL_REG_M1 << (ConfigPtr->Channels[pin].pin_num * 4)); /* Clear the PMCx bits for this pin */
                    break;

                case PortConfig_Mode_TIMER:

                    *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (PortConfig_CTL_REG_TIMER << (ConfigPtr->Channels[pin].pin_num * 4)); /* Clear the PMCx bits for this pin */
                    break;

                case PortConfig_Mode_DIGITAL_USB:
                    *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (PortConfig_CTL_REG_DIGITAL_USB << (ConfigPtr->Channels[pin].pin_num * 4)); /* Clear the PMCx bits for this pin */
                    break;

                case PortConfig_Mode_IDX:
                    *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (PortConfig_CTL_REG_QEI << (ConfigPtr->Channels[pin].pin_num * 4)); /* Clear the PMCx bits for this pin */
                    break;

                case PortConfig_Mode_DIGITAL_INPUT:
                case PortConfig_Mode_DIGITAL_OUTPUT:
                    CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET), ConfigPtr->Channels[pin].pin_num);   /* Clear the corresponding bit in the GPIOAMSEL register to disable analog functionality on this pin */
                    CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET), ConfigPtr->Channels[pin].pin_num);          /* Disable Alternative function for this pin by clear the corresponding bit in GPIOAFSEL register */
                    *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(0x0000000F << (ConfigPtr->Channels[pin].pin_num * 4)); /* Clear the PMCx bits for this pin */
                    break;
                }
                if (ConfigPtr->Channels[pin].direction == OUTPUT)
                {
                    SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIR_REG_OFFSET), ConfigPtr->Channels[pin].pin_num); /* Set the corresponding bit in the GPIODIR register to configure it as output pin */

                    if (ConfigPtr->Channels[pin].initial_value == STD_HIGH)
                    {
                        SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DATA_REG_OFFSET), ConfigPtr->Channels[pin].pin_num); /* Set the corresponding bit in the GPIODATA register to provide initial value 1 */
                    }
                    else
                    {
                        CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DATA_REG_OFFSET), ConfigPtr->Channels[pin].pin_num); /* Clear the corresponding bit in the GPIODATA register to provide initial value 0 */
                    }
                }
                else if (ConfigPtr->Channels[pin].direction == INPUT)
                {
                    CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIR_REG_OFFSET), ConfigPtr->Channels[pin].pin_num); /* Clear the corresponding bit in the GPIODIR register to configure it as input pin */

                    if (ConfigPtr->Channels[pin].resistor == PULL_UP)
                    {
                        SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_PULL_UP_REG_OFFSET), ConfigPtr->Channels[pin].pin_num); /* Set the corresponding bit in the GPIOPUR register to enable the internal pull up pin */
                    }
                    else if (ConfigPtr->Channels[pin].resistor == PULL_DOWN)
                    {
                        SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_PULL_DOWN_REG_OFFSET), ConfigPtr->Channels[pin].pin_num); /* Set the corresponding bit in the GPIOPDR register to enable the internal pull down pin */
                    }
                    else
                    {
                        CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_PULL_UP_REG_OFFSET), ConfigPtr->Channels[pin].pin_num);   /* Clear the corresponding bit in the GPIOPUR register to disable the internal pull up pin */
                        CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_PULL_DOWN_REG_OFFSET), ConfigPtr->Channels[pin].pin_num); /* Clear the corresponding bit in the GPIOPDR register to disable the internal pull down pin */
                    }
                }
                else
                {
                    /* Do Nothing */
                }

                SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET), ConfigPtr->Channels[pin].pin_num); /* Set the corresponding bit in the GPIODEN register to enable digital functionality on this pin */
            }                                                                                                                                     // else
        }                                                                                                                                         // for
    }
}
/************************************************************************************
 * Service Name: Port_SetPinDirection
 * Sync/Async: Synchronous
 * Service ID: 0x01
 * Reentrancy: reentrant
 * Parameters (in): Pin- Port Pin ID number       ,Direction -Port Pin direction
 * Parameters (inout): None
 * Parameters (out): None
 * Return value: None
 * Description: Sets the port pin direction
 ************************************************************************************/
void Port_SetPinDirection(Port_PinType Pin, Port_PinDirection Direction)
{

    boolean error = FALSE;
    volatile uint32 *PortGpio_Ptr = NULL_PTR; /* point to the required Port Registers base address */

#if (PORT_DEV_ERROR_DETECT == STD_ON)
    /* Check if the Driver is initialized before using this function */
    if (PORT_NOT_INITIALIZED == Port_Status)
    {
        Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID,
                        Port_SetPinDirection_SID, PORT_E_UNINIT);
        error = TRUE;
    }
    else
    {
        /* No Action Required */
    }
    /* Check if the used channel is within the valid range */
    if (PORT_CONFIGURED_CHANNLES <= PORT_CONFIGURED_CHANNLES)
    {

        Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID,
                        Port_SetPinDirection_SID, PORT_E_PARAM_PIN);
        error = TRUE;
    }
    else
    {
        /* No Action Required */
    }
    /* Check if the used channel is within the valid range */
    if (channels->Channels[Pin].direction_changeable == STD_OFF)
    {

        Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID,
                        Port_SetPinDirection_SID, PORT_E_DIRECTION_UNCHANGEABLE);
        error = TRUE;
    }
    else
    {
        /* No Action Required */
    }

#endif

    /* In-case there are no errors */
    if (FALSE == error)
    {
        /*channels->Channels[Pin].direction = Direction;*/

        switch (channels->Channels[Pin].port_num)
        {
        case 0:
            PortGpio_Ptr = (volatile uint32 *)GPIO_PORTA_BASE_ADDRESS; /* PORTA Base Address */
            break;
        case 1:
            PortGpio_Ptr = (volatile uint32 *)GPIO_PORTB_BASE_ADDRESS; /* PORTB Base Address */
            break;
        case 2:
            PortGpio_Ptr = (volatile uint32 *)GPIO_PORTC_BASE_ADDRESS; /* PORTC Base Address */
            break;
        case 3:
            PortGpio_Ptr = (volatile uint32 *)GPIO_PORTD_BASE_ADDRESS; /* PORTD Base Address */
            break;
        case 4:
            PortGpio_Ptr = (volatile uint32 *)GPIO_PORTE_BASE_ADDRESS; /* PORTE Base Address */
            break;
        case 5:
            PortGpio_Ptr = (volatile uint32 *)GPIO_PORTF_BASE_ADDRESS; /* PORTF Base Address */
            break;
        }
        if (Direction == OUTPUT)
        {
            SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIR_REG_OFFSET), channels->Channels[Pin].pin_num); /* Set the corresponding bit in the GPIODIR register to configure it as output pin */

            if (channels->Channels[Pin].initial_value == STD_HIGH)
            {
                SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DATA_REG_OFFSET), channels->Channels[Pin].pin_num); /* Set the corresponding bit in the GPIODATA register to provide initial value 1 */
            }
            else
            {
                CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DATA_REG_OFFSET), channels->Channels[Pin].pin_num); /* Clear the corresponding bit in the GPIODATA register to provide initial value 0 */
            }
        }
        else if (channels->Channels[Pin].direction == INPUT)
        {
            CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIR_REG_OFFSET), channels->Channels[Pin].pin_num); /* Clear the corresponding bit in the GPIODIR register to configure it as input pin */

            if (channels->Channels[Pin].resistor == PULL_UP)
            {
                SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_PULL_UP_REG_OFFSET), channels->Channels[Pin].pin_num); /* Set the corresponding bit in the GPIOPUR register to enable the internal pull up pin */
            }
            else if (channels->Channels[Pin].resistor == PULL_DOWN)
            {
                SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_PULL_DOWN_REG_OFFSET), channels->Channels[Pin].pin_num); /* Set the corresponding bit in the GPIOPDR register to enable the internal pull down pin */
            }
            else
            {
                CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_PULL_UP_REG_OFFSET), channels->Channels[Pin].pin_num);   /* Clear the corresponding bit in the GPIOPUR register to disable the internal pull up pin */
                CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_PULL_DOWN_REG_OFFSET), channels->Channels[Pin].pin_num); /* Clear the corresponding bit in the GPIOPDR register to disable the internal pull down pin */
            }
        }
        else
        {
            /* Do Nothing */
        }
    }
}
/************************************************************************************
 * Service Name: Port_RefreshPortDirection
 * Sync/Async: Synchronous
 * Reentrancy: Non reentrant
 * Parameters (in): None
 * Parameters (inout): None
 * Parameters (out): None
 * Return value: None
 * Description: Refreshes port direction
 ************************************************************************************/
void Port_RefreshPortDirection(void)
{
  boolean error=FALSE;
#if(PORT_DEV_ERROR_DETECT == STD_ON)
  {
    if (Port_Status == PORT_NOT_INITIALIZED)
    {
      Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, Port_SetPinDirection_SID,
                      PORT_E_UNINIT);
      error = TRUE ;
    }
    else
    {
      /* Do Nothing */
    }
    
  }
#endif
  if(error==FALSE)
  {
    uint8 delay=0;
    volatile uint32* PortGPIO_Ptr=NULL_PTR;
    volatile uint8 i=0;
    /*looping on the configured channels*/
    for(i=0;i<PORT_CONFIGURED_CHANNLES ;i++)
    {
      /*saving the port address in the pointer depending on the configured channel*/
      switch(channels->Channels[i].port_num)
      {
      case PortConfig_PORTA_NUM:
        PortGPIO_Ptr=(volatile uint32*)GPIO_PORTA_BASE_ADDRESS;
        break;
      case PortConfig_PORTB_NUM:
        PortGPIO_Ptr=(volatile uint32*)GPIO_PORTB_BASE_ADDRESS;
        break;
      case PortConfig_PORTC_NUM:
        PortGPIO_Ptr=(volatile uint32*)GPIO_PORTC_BASE_ADDRESS;
        break;
      case PortConfig_PORTD_NUM:
        PortGPIO_Ptr=(volatile uint32*)GPIO_PORTD_BASE_ADDRESS;
        break;
      case PortConfig_PORTE_NUM:
        PortGPIO_Ptr=(volatile uint32*)GPIO_PORTE_BASE_ADDRESS;
        break;
      case PortConfig_PORTF_NUM:
        PortGPIO_Ptr=(volatile uint32*)GPIO_PORTF_BASE_ADDRESS;
        break;
      }
     
     
      if(channels->Channels[i].direction==OUTPUT)
      {
        SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGPIO_Ptr + PORT_DIR_REG_OFFSET),channels->Channels[i].pin_num);
      }
      if(channels->Channels[i].direction==INPUT)
      {
        CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGPIO_Ptr + PORT_DIR_REG_OFFSET),channels->Channels[i].pin_num);
        
      }
      
      
    }
  }
}
/************************************************************************************
 * Service Name: Port_GetVersionInfo
 * Sync/Async: Synchronous
 * Reentrancy: Non reentrant
 * Parameters (in): None
 * Parameters (inout): None
 * Parameters (out): versioninfo     -Pointer to where to store the version information of this module.
 * Return value: None
 * Description: Returns the version information of this module.
 ************************************************************************************/
#if (PORT_VERSION_INFO_API == STD_ON)
void Port_GetVersionInfo(Std_VersionInfoType *versioninfo)
{
#if (PORT_DEV_ERROR_DETECT == STD_ON)
  /* Check if input pointer is not Null pointer */
  if(NULL_PTR == versioninfo)
  {
    /* Report to DET  */
    Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID,
                    Port_SetPinDirection_SID, PORT_E_PARAM_POINTER);
  }
  else
#endif /* (PORT_DEV_ERROR_DETECT == STD_ON) */
  {
    /* Copy the vendor Id */
    versioninfo->vendorID = (uint16)PORT_VENDOR_ID;
    /* Copy the module Id */
    versioninfo->moduleID = (uint16)PORT_MODULE_ID;
    /* Copy Software Major Version */
    versioninfo->sw_major_version = (uint8)PORT_SW_MAJOR_VERSION;
    /* Copy Software Minor Version */
    versioninfo->sw_minor_version = (uint8)PORT_SW_MINOR_VERSION;
    /* Copy Software Patch Version */
    versioninfo->sw_patch_version = (uint8)PORT_SW_PATCH_VERSION;
  }
}

#endif
/************************************************************************************
* Service Name: Port_SetPinMode
* Sync/Async: Synchronous
* Reentrancy: reentrant
* Parameters (in): Pin          - Port Pin ID number
                   Mode         - New Port Pin mode to be set on port pin.
* Parameters (inout): None
* Parameters (out): None
* Return value: None
* Description: Sets the port pin mode.
************************************************************************************/
void Port_SetPinMode(Port_PinType Pin, Port_PinModeType Mode)
{

    boolean error = FALSE;
    volatile uint32 *PortGpio_Ptr = NULL_PTR; /* point to the required Port Registers base address */

#if (PORT_DEV_ERROR_DETECT == STD_ON)
    /* Check if the Driver is initialized before using this function */
    if (PORT_NOT_INITIALIZED == Port_Status)
    {
        Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID,
                        Port_SetPinMode_SID, PORT_E_UNINIT);
        error = TRUE;
    }
    else
    {
        /* No Action Required */
    }
    /* Check if the used channel is within the valid range */
    if (PORT_CONFIGURED_CHANNLES <= PORT_CONFIGURED_CHANNLES)
    {

        Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID,
                        Port_SetPinDirection_SID, PORT_E_PARAM_PIN);
        error = TRUE;
    }
    else
    {
        /* No Action Required */
    }
    /* Check if the used channel is within the valid range */
    if (channels->Channels[Pin].mode_changeable == STD_OFF)
    {

        Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID,
                        Port_SetPinDirection_SID, PORT_E_MODE_UNCHANGEABLE);
        error = TRUE;
    }
    else
    {
        /* No Action Required */
    }
    /* Check if the used channel is within the valid range */
    if (Mode > PORT_CONFIGURED_MODES)
    {

        Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID,
                        Port_SetPinDirection_SID, PORT_E_PARAM_INVALID_MODE);
        error = TRUE;
    }
    else
    {
        /* No Action Required */
    }

#endif

    /* In-case there are no errors */
    if (FALSE == error)
    {
        /*channels->Channels[Pin].pin_mode = Mode;*/

        if (Mode == PortConfig_Mode_ANALOG)
        {

            *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(0x0000000F << (channels->Channels[Pin].pin_num * 4)); /* Clear the PMCx bits for this pin */
            SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET), channels->Channels[Pin].pin_num);     /* Clear the corresponding bit in the GPIOAMSEL register to disable analog functionality on this pin */
            SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET), channels->Channels[Pin].pin_num);            /* Disable Alternative function for this pin by clear the corresponding bit in GPIOAFSEL register */
            CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET), channels->Channels[Pin].pin_num);    /* Clear the corresponding bit in the GPIODATA register to provide initial value 0 */
        }
        else
        {
            /*all modes */
            CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET), channels->Channels[Pin].pin_num); /* Clear the corresponding bit in the GPIOAMSEL register to disable analog functionality on this pin */
            SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET), channels->Channels[Pin].pin_num);          /* Disable Alternative function for this pin by clear the corresponding bit in GPIOAFSEL register */
            switch (Mode)
            {
            case PortConfig_Mode_UART0TX:
            case PortConfig_Mode_UART0RX:

            case PortConfig_Mode_UART2TX:
            case PortConfig_Mode_UART2RX:
            case PortConfig_Mode_UART3TX:
            case PortConfig_Mode_UART3RX:
            case PortConfig_Mode_UART4TX:
            case PortConfig_Mode_UART4RX:
            case PortConfig_Mode_UART5TX:
            case PortConfig_Mode_UART5RX:
            case PortConfig_Mode_UART6TX:
            case PortConfig_Mode_UART6RX:
            case PortConfig_Mode_UART7TX:
            case PortConfig_Mode_UART7RX:
                *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (PortConfig_CTL_REG_UART_DEFUALT << (channels->Channels[Pin].pin_num * 4)); /* Clear the PMCx bits for this pin */

                break;
            case PortConfig_Mode_UART1TX:
            case PortConfig_Mode_UART1RX:
                if (channels->Channels[Pin].port_num == 2)
                {
                    *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (PortConfig_CTL_REG_UART_EXCEPTION << (channels->Channels[Pin].pin_num * 4)); /* Clear the PMCx bits for this pin */
                }
                else
                {
                    *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (PortConfig_CTL_REG_UART_DEFUALT << (channels->Channels[Pin].pin_num * 4)); /* Clear the PMCx bits for this pin */
                }
                break;

            case PortConfig_Mode_SSI0TX:
            case PortConfig_Mode_SSI0RX:

            case PortConfig_Mode_SSI1TX:
            case PortConfig_Mode_SSI1RX:

            case PortConfig_Mode_SSI2TX:
            case PortConfig_Mode_SSI2RX:
                *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (PortConfig_CTL_REG_SSI_DEFUALT << (channels->Channels[Pin].pin_num * 4)); /* Clear the PMCx bits for this pin */
                break;
            case PortConfig_Mode_SSI3TX:
            case PortConfig_Mode_SSI3RX:
                *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (PortConfig_CTL_REG_SSI_EXCEPTION << (channels->Channels[Pin].pin_num * 4)); /* Clear the PMCx bits for this pin */
                break;
            case PortConfig_Mode_I2C0:
            case PortConfig_Mode_I2C1:
            case PortConfig_Mode_I2C2:
            case PortConfig_Mode_I2C3:
                *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (PortConfig_CTL_REG_I2C << (channels->Channels[Pin].pin_num * 4)); /* Clear the PMCx bits for this pin */
                break;

            case PortConfig_Mode_CAN0TX:
            case PortConfig_Mode_CAN0RX:

                if (channels->Channels[Pin].port_num == PortConfig_PORTF_NUM)
                {

                    *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (PortConfig_CTL_REG_CAN_EXCEPTION << (channels->Channels[Pin].pin_num * 4)); /* Clear the PMCx bits for this pin */
                }
                else
                {
                    *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (PortConfig_CTL_REG_CAN_DEFUALT << (channels->Channels[Pin].pin_num * 4)); /* Clear the PMCx bits for this pin */
                }
                break;

            case PortConfig_Mode_CAN1TX:
            case PortConfig_Mode_CAN1RX:
                *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (PortConfig_CTL_REG_CAN_DEFUALT << (channels->Channels[Pin].pin_num * 4)); /* Clear the PMCx bits for this pin */
                break;

            case PortConfig_Mode_M0PWM:
            case PortConfig_Mode_M0FAULT:

                *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (PortConfig_CTL_REG_M0 << (channels->Channels[Pin].pin_num * 4)); /* Clear the PMCx bits for this pin */
                break;

            case PortConfig_Mode_M1PWM:
                *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (PortConfig_CTL_REG_M1 << (channels->Channels[Pin].pin_num * 4)); /* Clear the PMCx bits for this pin */
                break;

            case PortConfig_Mode_TIMER:

                *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (PortConfig_CTL_REG_TIMER << (channels->Channels[Pin].pin_num * 4)); /* Clear the PMCx bits for this pin */
                break;

            case PortConfig_Mode_DIGITAL_USB:
                *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (PortConfig_CTL_REG_DIGITAL_USB << (channels->Channels[Pin].pin_num * 4)); /* Clear the PMCx bits for this pin */
                break;

            case PortConfig_Mode_IDX:
                *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (PortConfig_CTL_REG_QEI << (channels->Channels[Pin].pin_num * 4)); /* Clear the PMCx bits for this pin */
                break;

            case PortConfig_Mode_DIGITAL_INPUT:
            case PortConfig_Mode_DIGITAL_OUTPUT:
                CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET), channels->Channels[Pin].pin_num);   /* Clear the corresponding bit in the GPIOAMSEL register to disable analog functionality on this pin */
                CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET), channels->Channels[Pin].pin_num);          /* Disable Alternative function for this pin by clear the corresponding bit in GPIOAFSEL register */
                *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(0x0000000F << (channels->Channels[Pin].pin_num * 4)); /* Clear the PMCx bits for this pin */
                break;
            }
            if ((Mode >= PortConfig_Mode_UART0RX && Mode <= PortConfig_Mode_UART7RX) || (Mode >= PortConfig_Mode_SSI0RX && Mode <= PortConfig_Mode_SSI3RX) || (Mode >= PortConfig_Mode_CAN0RX && Mode <= PortConfig_Mode_CAN1RX) || Mode == PortConfig_Mode_IDX || Mode <= PortConfig_Mode_M0FAULT)
            {
                CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIR_REG_OFFSET), channels->Channels[Pin].pin_num); /* Clear the corresponding bit in the GPIODIR register to configure it as input pin */
            }
            else
            {
                SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIR_REG_OFFSET), channels->Channels[Pin].pin_num); /* Set the corresponding bit in the GPIODIR register to configure it as output pin */
            }
        }
    }
}