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
 * $Date:        9. February 2023
 * $Revision:    V1.0
 *
 * Project:      GPIO Driver for LPC5411x
 */

#include "fsl_clock.h"
#include "fsl_inputmux.h"
#include "fsl_pint.h"
#include "fsl_gpio_ex.h"
#include "fsl_iocon_ex.h"

#include "GPIO_LPC5411x.h"


// Pin mapping
//    0 ..  22: PORT0  0..22
//   29 ..  31: PORT0 29..31
//   32 ..  50: PORT1  0..17

// Pins 23,24,25,26:
// - have specific support for I2C drive
// - are already open drain and require an external pull-up to provide output functionality

#define GPIO_MAX_PORTS          2U
#define GPIO_MAX_PINS           48U


// Default Pin Configuration
static const uint32_t DefaultPinConfig = (
IOCON_PIO_FUNC(0U)      | // Pin function  : PIO
IOCON_PIO_MODE(2U)      | // Function mode : Pull-up enabled
IOCON_PIO_INVERT(0U)    | // Input polarity: Disabled
IOCON_PIO_DIGIMODE(1U)  | // Select mode   : Digital
IOCON_PIO_FILTEROFF(1U) | // Glitch filter : Enabled
IOCON_PIO_SLEW(0U)      | // Slew rate     : Standard mode
IOCON_PIO_OD(0U)          // Open drain    : Disabled
);

// PINx IRQ Numbers
static IRQn_Type const PinIRQn[FSL_FEATURE_PINT_NUMBER_OF_CONNECTED_OUTPUTS] = {
  PIN_INT0_IRQn, PIN_INT1_IRQn, PIN_INT2_IRQn, PIN_INT3_IRQn,
  PIN_INT4_IRQn, PIN_INT5_IRQn, PIN_INT6_IRQn, PIN_INT7_IRQn
};

// Clock IP Names
static clock_ip_name_t const ClockIP[GPIO_MAX_PORTS] = {
  kCLOCK_Gpio0, kCLOCK_Gpio1
};


// Signal Event callback functions
static ARM_GPIO_SignalEvent_t SignalEvent[FSL_FEATURE_PINT_NUMBER_OF_CONNECTED_OUTPUTS];


// Common PIN_INTx IRQ Handler
static void PIN_INTx_IRQHandler (uint32_t num) {
  uint32_t   event;
  pint_pin_enable_t out;

  PINT_PinInterruptGetConfig(PINT, (pint_pin_int_t)num, &out, 0U);

  switch (out) {
    case kPINT_PinIntEnableRiseEdge:
      event = ARM_GPIO_EVENT_RISING_EDGE;
      break;
    case kPINT_PinIntEnableFallEdge:
      event = ARM_GPIO_EVENT_FALLING_EDGE;
      break;
    case kPINT_PinIntEnableBothEdges:
      event = ARM_GPIO_EVENT_EITHER_EDGE;
      break;
    default:
      event = 0U;
      break;
  }

  if (event != 0U) {
    SignalEvent[num](0U, event);
  }

  // Edge sensitive: clear Pin interrupt after callback
  if ((PINT->ISEL & (1U << num)) == 0x0U) {
    PINT_PinInterruptClrStatus(PINT, (pint_pin_int_t)num);
  }
}

// Pin interrupt 0 IRQ Handler
void PIN_INT0_IRQHandler (void) {
  PIN_INTx_IRQHandler(0U);
}

// Pin interrupt 1 IRQ Handler
void PIN_INT1_IRQHandler (void) {
  PIN_INTx_IRQHandler(1U);
}

// Pin interrupt 2 IRQ Handler
void PIN_INT2_IRQHandler (void) {
  PIN_INTx_IRQHandler(2U);
}

// Pin interrupt 3 IRQ Handler
void PIN_INT3_IRQHandler (void) {
  PIN_INTx_IRQHandler(3U);
}

// Pin interrupt 4 IRQ Handler
void PIN_INT4_IRQHandler (void) {
  PIN_INTx_IRQHandler(4U);
}

// Pin interrupt 5IRQ Handler
void PIN_INT5_IRQHandler (void) {
  PIN_INTx_IRQHandler(5U);
}

// Pin interrupt 6 IRQ Handler
void PIN_INT6_IRQHandler (void) {
  PIN_INTx_IRQHandler(6U);
}

// Pin interrupt 7 IRQ Handler
void PIN_INT7_IRQHandler (void) {
  PIN_INTx_IRQHandler(7U);
}

// Setup GPIO Interface
static int32_t GPIO_Setup (ARM_GPIO_Pin_t pin, ARM_GPIO_SignalEvent_t cb_event) {
  uint32_t pin_port;
  uint32_t pin_num;
  uint32_t pint;
  int32_t  result = ARM_DRIVER_ERROR;

  if (pin < GPIO_MAX_PINS) {
    pin_port = pin >> 1U;
    pin_num  = pin & 0x1FU;
    pint     = pin / 6U;
    SignalEvent[pint] = cb_event;
    CLOCK_EnableClock(ClockIP[pin_port]);
    CLOCK_EnableClock(kCLOCK_Iocon);
    CLOCK_EnableClock(kCLOCK_InputMux);
    // Connect trigger sources to PINT
    INPUTMUX_AttachSignal(INPUTMUX, (pint_pin_int_t)pint, pin + (PINTSEL_PMUX_ID << PMUX_SHIFT));
    CLOCK_DisableClock(kCLOCK_InputMux);
    PINT_Init(PINT);
    PINT_PinInterruptConfig(PINT, (pint_pin_int_t)pint, kPINT_PinIntEnableNone, NULL);
    GPIO_PinSetDirection(GPIO, pin_port, pin_num, kGPIO_DigitalInput);
    IOCON_PinMuxSet(IOCON, (uint8_t)pin_port, (uint8_t)pin_num, DefaultPinConfig);
    NVIC_EnableIRQ(PinIRQn[pin >> 3U]);

   result = ARM_DRIVER_OK;

  }

  return result;
}

// Set GPIO Direction
static int32_t GPIO_SetDirection (ARM_GPIO_Pin_t pin, ARM_GPIO_DIRECTION direction) {
  uint32_t pin_port;
  uint32_t pin_num;
  int32_t  result = ARM_DRIVER_ERROR;

  if (pin < GPIO_MAX_PINS) {
    if( pin > 26U && pin < 29U) {
      result = ARM_DRIVER_ERROR_UNSUPPORTED;
    } else {
      pin_port = pin >> 1U;
      pin_num  = pin & 0x1FU;
      if (direction == ARM_GPIO_OUTPUT) {
        GPIO_PinSetDirection(GPIO, pin_port, pin_num, kGPIO_DigitalOutput);
      } else {
        GPIO_PinSetDirection(GPIO, pin_port, pin_num, kGPIO_DigitalInput);
      }
      result = ARM_DRIVER_OK;
      }
    }
  return result;
}

// Set GPIO Output Mode
static int32_t GPIO_SetOutputMode (ARM_GPIO_Pin_t pin, ARM_GPIO_OUTPUT_MODE mode) {
  uint32_t pin_port;
  uint32_t pin_num;
  int32_t  result = ARM_DRIVER_ERROR;

  if (pin < GPIO_MAX_PINS) {
    if( pin > 22U && pin < 29U) {
      result = ARM_DRIVER_ERROR_UNSUPPORTED;
    } else {
      pin_port = pin >> 1U;
      pin_num  = pin & 0x1FU;
      if (mode == ARM_GPIO_OPEN_DRAIN) {
        IOCON_EnablePinOpenDrain(IOCON, pin_port, pin_num, true);
      } else {
        IOCON_EnablePinOpenDrain(IOCON, pin_port, pin_num, false);
      }
      result = ARM_DRIVER_OK;
    }
  }

  return result;
}

// Set GPIO Pull Resistor
static int32_t GPIO_SetPullResistor (ARM_GPIO_Pin_t pin, ARM_GPIO_PULL_RESISTOR resistor) {
  uint32_t pin_port;
  uint32_t pin_num;
  int32_t  result = ARM_DRIVER_ERROR;

  if (pin < GPIO_MAX_PINS) {
        if( pin > 22U && pin < 29U) {
          result = ARM_DRIVER_ERROR_UNSUPPORTED;
        } else {
          pin_port = pin >> 1U;
          pin_num  = pin & 0x1FU;
          switch (resistor) {
            case ARM_GPIO_PULL_NONE:
              IOCON_SetPinPullConfig(IOCON, pin_port, pin_num, IOCON_MODE_INACT);
              result = ARM_DRIVER_OK;
              break;
            case ARM_GPIO_PULL_UP:
              IOCON_SetPinPullConfig(IOCON, pin_port, pin_num, IOCON_MODE_PULLDOWN);
              result = ARM_DRIVER_OK;
              break;
            case ARM_GPIO_PULL_DOWN:
              IOCON_SetPinPullConfig(IOCON, pin_port, pin_num, IOCON_MODE_PULLUP);
              result = ARM_DRIVER_OK;
              break;
            default:
              break;
          }
        }
  }

  return result;
}

// Set GPIO Event Trigger
static int32_t GPIO_SetEventTrigger (ARM_GPIO_Pin_t pin, ARM_GPIO_EVENT_TRIGGER trigger) {
  uint32_t pint;
  int32_t  result = ARM_DRIVER_ERROR;

  if (pin < GPIO_MAX_PINS) {
    pint     = pin / 6U;
    switch (trigger) {
      case ARM_GPIO_TRIGGER_NONE:
        PINT_PinInterruptConfig(PINT, (pint_pin_int_t)pint, kPINT_PinIntEnableNone, 0U);
        result = ARM_DRIVER_OK;
        break;
      case ARM_GPIO_TRIGGER_RISING_EDGE:
        PINT_PinInterruptConfig(PINT, (pint_pin_int_t)pint, kPINT_PinIntEnableRiseEdge, 0U);
        result = ARM_DRIVER_OK;
        break;
      case ARM_GPIO_TRIGGER_FALLING_EDGE:
        PINT_PinInterruptConfig(PINT, (pint_pin_int_t)pint, kPINT_PinIntEnableFallEdge, 0U);
        result = ARM_DRIVER_OK;
        break;
      case ARM_GPIO_TRIGGER_EITHER_EDGE:
        PINT_PinInterruptConfig(PINT, (pint_pin_int_t)pint, kPINT_PinIntEnableBothEdges, 0U);
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
  uint32_t pin_port;
  uint32_t pin_num;

  if (pin < GPIO_MAX_PINS) {
    pin_port = pin >> 1U;
    pin_num  = pin & 0x1FU;
    GPIO_PinWrite(GPIO, pin_port, pin_num, (uint8_t)val);
  }
}

// Get GPIO Input Level
static uint32_t GPIO_GetInput (ARM_GPIO_Pin_t pin) {
  uint32_t pin_port;
  uint32_t pin_num;
  uint32_t val = 0U;

  if (pin < GPIO_MAX_PINS) {
    pin_port = pin >> 1U;
    pin_num  = pin & 0x1FU;
    val = GPIO_PinRead(GPIO, pin_port, pin_num);
  }

  return val;
}


// GPIO Driver access structure
ARM_DRIVER_GPIO Driver_GPIO = {
  GPIO_Setup,
  GPIO_SetDirection,
  GPIO_SetOutputMode,
  GPIO_SetPullResistor,
  GPIO_SetEventTrigger,
  GPIO_SetOutput,
  GPIO_GetInput
};
