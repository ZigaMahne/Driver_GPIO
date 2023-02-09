/*
 * Copyright (c) 2023 ARM Limited. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * $Date:        31. January 2023
 * $Revision:    V1.0
 *
 * Project:      GPIO Driver for Kinetis K22F
 */

#include "fsl_clock.h"
#include "fsl_gpio.h"
#include "fsl_gpio_ex.h"
#include "fsl_port.h"
#include "fsl_port_ex.h"

#include "GPIO_MK22F.h"

// Driver version
#define ARM_GPIO_DRV_VERSION ARM_DRIVER_VERSION_MAJOR_MINOR(1, 0)

static const ARM_DRIVER_VERSION GPIO_DriverVersion = {
  ARM_GPIO_API_VERSION, ARM_GPIO_DRV_VERSION
};


// Pin mapping
//   0 ..  31: PORTA 0..31
//  32 ..  63: PORTB 0..31
//  64 ..  95: PORTC 0..31
//  96 .. 127: PORTD 0..31
// 128 .. 159: PORTE 0..31

#define GPIO_MAX_PORTS          5U
#define GPIO_MAX_PINS           160U

// Disabled Pin Configuration
static const port_pin_config_t DisabledPinConfig = {
  .pullSelect = kPORT_PullDisable,
  .slewRate = kPORT_FastSlewRate,
  .passiveFilterEnable = kPORT_PassiveFilterDisable,
  .openDrainEnable = kPORT_OpenDrainDisable,
  .driveStrength = kPORT_LowDriveStrength,
  .mux = kPORT_PinDisabledOrAnalog,
  .lockRegister = kPORT_UnlockRegister
};

// Default Pin Configuration
static const port_pin_config_t DefaultPinConfig = {
  .pullSelect = kPORT_PullDisable,
  .slewRate = kPORT_FastSlewRate,
  .passiveFilterEnable = kPORT_PassiveFilterDisable,
  .openDrainEnable = kPORT_OpenDrainDisable,
  .driveStrength = kPORT_LowDriveStrength,
  .mux = kPORT_MuxAsGpio,
  .lockRegister = kPORT_UnlockRegister
};

// PORTx IRQ Numbers
static IRQn_Type const PortIRQn[GPIO_MAX_PORTS] = {
  PORTA_IRQn, PORTB_IRQn, PORTB_IRQn, PORTD_IRQn, PORTE_IRQn
};

// PORTx Base Pointers
static PORT_Type * const PortBase[GPIO_MAX_PORTS] = {
  PORTA, PORTB, PORTC, PORTD, PORTE
};

// GPIOx Base Pointers
static GPIO_Type * const GPIOBase[GPIO_MAX_PORTS] = {
  GPIOA, GPIOB, GPIOC, GPIOD, GPIOE
};

// Clock IP Names
static clock_ip_name_t const ClockIP[GPIO_MAX_PORTS] = {
  kCLOCK_PortA, kCLOCK_PortB, kCLOCK_PortC, kCLOCK_PortD, kCLOCK_PortE
};


// Port Active Flags
static uint32_t PortActive[GPIO_MAX_PORTS] = {
  0U, 0U, 0U, 0U, 0U
};


// Signal Event callback functions
static ARM_GPIO_SignalEvent_t SignalEvent[GPIO_MAX_PORTS][32];


// Common PORTx IRQ Handler
static void PORTx_IRQHandler (uint32_t num) {
  PORT_Type *port = PortBase[num];
  uint32_t   ifsr;
  uint32_t   irqc;
  uint32_t   event;
  uint32_t   i;

  ifsr = PORT_GetPinsInterruptFlags(port);
  for (i = 0U; i < 32U; i++) {
    if (ifsr & (1U << i)) {
      if (SignalEvent[num][i] != NULL) {
        irqc = (PORT_GetPinConfig(port, i) & PORT_PCR_IRQC_MASK) >> PORT_PCR_IRQC_SHIFT;
        switch (irqc) {
          case kPORT_InterruptRisingEdge:
            event = ARM_GPIO_EVENT_RISING_EDGE;
            break;
          case kPORT_InterruptFallingEdge:
            event = ARM_GPIO_EVENT_FALLING_EDGE;
            break;
          case kPORT_InterruptEitherEdge:
            event = ARM_GPIO_EVENT_EITHER_EDGE;
            break;
          default:
            event = 0U;
            break;
        }
        if (event != 0U) {
          SignalEvent[num][i]((num << 5) | i, event);
        }
      }
    }
  }
  PORT_ClearPinsInterruptFlags(port, ifsr);
}

// PORTA IRQ Handler
void PORTA_IRQHandler (void) {
  PORTx_IRQHandler(0U);
}

// PORTB IRQ Handler
void PORTB_IRQHandler (void) {
  PORTx_IRQHandler(1U);
}

// PORTC IRQ Handler
void PORTC_IRQHandler (void) {
  PORTx_IRQHandler(2U);
}

// PORTD IRQ Handler
void PORTD_IRQHandler (void) {
  PORTx_IRQHandler(3U);
}

// PORTE IRQ Handler
void PORTE_IRQHandler (void) {
  PORTx_IRQHandler(4U);
}


// Get driver version
static ARM_DRIVER_VERSION GPIO_GetVersion (void) {
  return GPIO_DriverVersion;
}

// Initialize GPIO Interface
static int32_t GPIO_Initialize (ARM_GPIO_Pin_t pin, ARM_GPIO_SignalEvent_t cb_event) {
  uint32_t pin_port;
  uint32_t pin_num;
  int32_t  result = ARM_DRIVER_ERROR;

  if (pin < GPIO_MAX_PINS) {
    pin_port = pin >> 5U;
    pin_num  = pin & 0x1FU;
    SignalEvent[pin_port][pin_num] = cb_event;
    result = ARM_DRIVER_OK;
  }
 
  return result;
}

// De-initialize GPIO Interface
static int32_t GPIO_Uninitialize (ARM_GPIO_Pin_t pin) {
  uint32_t pin_port;
  uint32_t pin_num;
  int32_t  result = ARM_DRIVER_ERROR;

  if (pin < GPIO_MAX_PINS) {
    pin_port = pin >> 5U;
    pin_num  = pin & 0x1FU;
    SignalEvent[pin_port][pin_num] = NULL;
    result = ARM_DRIVER_OK;
  }
 
  return result;
}

// Control GPIO Interface Power
static int32_t GPIO_PowerControl (ARM_GPIO_Pin_t pin, ARM_POWER_STATE state) {
  PORT_Type *port;
  GPIO_Type *gpio;
  uint32_t   pin_port;
  uint32_t   pin_num;
  int32_t    result = ARM_DRIVER_ERROR;

  if (pin < GPIO_MAX_PINS) {
    pin_port = pin >> 5U;
    pin_num  = pin & 0x1FU;
    port = PortBase[pin_port];
    gpio = GPIOBase[pin_port];
    switch (state) {
      case ARM_POWER_OFF:
        PortActive[pin_port] &= ~(1U << pin_num);
        PORT_SetPinInterruptConfig(port, pin_num, kPORT_InterruptOrDMADisabled);
        GPIO_PinSetDirection(gpio, pin_num, kGPIO_DigitalInput);
        PORT_SetPinConfig(port, pin_num, &DisabledPinConfig);
        if (PortActive[pin_port] == 0U) {
          NVIC_DisableIRQ(PortIRQn[pin_port]);
        }
        break;
      case ARM_POWER_LOW:
        break;
      case ARM_POWER_FULL:
        if ((PortActive[pin_port] & (1U << pin_num)) == 0U) {
          PortActive[pin_port] |= (1U << pin_num);
          CLOCK_EnableClock(ClockIP[pin_port]);
          PORT_SetPinInterruptConfig(port, pin_num, kPORT_InterruptOrDMADisabled);
          GPIO_PinSetDirection(gpio, pin_num, kGPIO_DigitalInput);
          PORT_SetPinConfig(port, pin_num, &DefaultPinConfig);
          NVIC_EnableIRQ(PortIRQn[pin_port]);
        }
        result = ARM_DRIVER_OK;
        break;
    }
  }

  return result;
}

// Set GPIO Direction
static int32_t GPIO_SetDirection (ARM_GPIO_Pin_t pin, ARM_GPIO_DIRECTION direction) {
  GPIO_Type *gpio;
  uint32_t   pin_port;
  uint32_t   pin_num;
  int32_t    result = ARM_DRIVER_ERROR;

  if (pin < GPIO_MAX_PINS) {
    pin_port = pin >> 5U;
    pin_num  = pin & 0x1FU;
    gpio = GPIOBase[pin_port];
    if (direction == ARM_GPIO_OUTPUT) {
      GPIO_PinSetDirection(gpio, pin_num, kGPIO_DigitalOutput);
    } else {
      GPIO_PinSetDirection(gpio, pin_num, kGPIO_DigitalInput);
    }
    result = ARM_DRIVER_OK;
  }
 
  return result;
}

// Set GPIO Output Mode
static int32_t GPIO_SetOutputMode (ARM_GPIO_Pin_t pin, ARM_GPIO_OUTPUT_MODE mode) {
  PORT_Type *port;
  uint32_t   pin_port;
  uint32_t   pin_num;
  int32_t    result = ARM_DRIVER_ERROR;

  if (pin < GPIO_MAX_PINS) {
    pin_port = pin >> 5U;
    pin_num  = pin & 0x1FU;
    port = PortBase[pin_port];
    if (mode == ARM_GPIO_OPEN_DRAIN) {
      PORT_EnablePinOpenDrain(port, pin_num, true);
    } else {
      PORT_EnablePinOpenDrain(port, pin_num, false);
    }
    result = ARM_DRIVER_OK;
  }
 
  return result;
}

// Set GPIO Pull Resistor
static int32_t GPIO_SetPullResistor (ARM_GPIO_Pin_t pin, ARM_GPIO_PULL_RESISTOR resistor) {
  PORT_Type *port;
  uint32_t   pin_port;
  uint32_t   pin_num;
  int32_t    result = ARM_DRIVER_ERROR;

  if (pin < GPIO_MAX_PINS) {
    pin_port = pin >> 5U;
    pin_num  = pin & 0x1FU;
    port = PortBase[pin_port];
    switch (resistor) {
      case ARM_GPIO_PULL_NONE:
        PORT_SetPinPullConfig(port, pin_num, kPORT_PullDisable);
        result = ARM_DRIVER_OK;
        break;
      case ARM_GPIO_PULL_UP:
        PORT_SetPinPullConfig(port, pin_num, kPORT_PullDown);
        result = ARM_DRIVER_OK;
        break;
      case ARM_GPIO_PULL_DOWN:
        PORT_SetPinPullConfig(port, pin_num, kPORT_PullUp);
        result = ARM_DRIVER_OK;
        break;
      default:
        break;
    }
  }
 
  return result;
}

// Set GPIO Event Trigger
static int32_t GPIO_SetEventTrigger (ARM_GPIO_Pin_t pin, ARM_GPIO_EVENT_TRIGGER trigger) {
  PORT_Type *port;
  uint32_t   pin_port;
  uint32_t   pin_num;
  int32_t    result = ARM_DRIVER_ERROR;

  if (pin < GPIO_MAX_PINS) {
    pin_port = pin >> 5U;
    pin_num  = pin & 0x1FU;
    port = PortBase[pin_port];
    switch (trigger) {
      case ARM_GPIO_TRIGGER_NONE:
        PORT_SetPinInterruptConfig(port, pin_num, kPORT_InterruptOrDMADisabled);
        result = ARM_DRIVER_OK;
        break;
      case ARM_GPIO_TRIGGER_RISING_EDGE:
        PORT_SetPinInterruptConfig(port, pin_num, kPORT_InterruptRisingEdge);
        result = ARM_DRIVER_OK;
        break;
      case ARM_GPIO_TRIGGER_FALLING_EDGE:
        PORT_SetPinInterruptConfig(port, pin_num, kPORT_InterruptFallingEdge);
        result = ARM_DRIVER_OK;
        break;
      case ARM_GPIO_TRIGGER_EITHER_EDGE:
        PORT_SetPinInterruptConfig(port, pin_num, kPORT_InterruptEitherEdge);
        result = ARM_DRIVER_OK;
        break;
      default:
        break;
    }
  }
 
  return result;
}

// Set GPIO Output Level
static void GPIO_SetOutput (ARM_GPIO_Pin_t pin, uint32_t val) {
  GPIO_Type *gpio;
  uint32_t   pin_port;
  uint32_t   pin_num;

  if (pin < GPIO_MAX_PINS) {
    pin_port = pin >> 5U;
    pin_num  = pin & 0x1FU;
    gpio = GPIOBase[pin_port];
    GPIO_PinWrite(gpio, pin_num, (uint8_t)val);
  }
}

// Get GPIO Input Level
static uint32_t GPIO_GetInput (ARM_GPIO_Pin_t pin) {
  GPIO_Type *gpio;
  uint32_t   pin_port;
  uint32_t   pin_num;
  uint32_t   val = 0U;

  if (pin < GPIO_MAX_PINS) {
    pin_port = pin >> 5U;
    pin_num  = pin & 0x1FU;
    gpio = GPIOBase[pin_port];
    val = GPIO_PinRead(gpio, pin_num);
  }

  return val;
}


// GPIO0 Driver access structure
ARM_DRIVER_GPIO Driver_GPIO0 = {
  GPIO_GetVersion,
  GPIO_Initialize,
  GPIO_Uninitialize,
  GPIO_PowerControl,
  GPIO_SetDirection,
  GPIO_SetOutputMode,
  GPIO_SetPullResistor,
  GPIO_SetEventTrigger,
  GPIO_SetOutput,
  GPIO_GetInput
};
