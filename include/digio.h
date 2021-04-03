/*
 * This file is part of the libopeninv project.
 *
 * Copyright (C) 2011 Johannes Huebner <dev@johanneshuebner.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#ifndef DIGIO_H_INCLUDED
#define DIGIO_H_INCLUDED

// #include <libopencm3/stm32/gpio.h>
#include "hal.h"
#include "digio_prj.h"

namespace PinMode {
   enum PinMode
   {
       INPUT_PD,
       INPUT_PU,
       INPUT_FLT,
       INPUT_AIN,
       OUTPUT,
       LAST
   };
}

class DigIo
{
public:
   #define DIG_IO_ENTRY(name, port, pin, mode) static DigIo name;
   DIG_IO_LIST
   #undef DIG_IO_ENTRY

   /** Map GPIO pin object to hardware pin.
    * @param[in] port port to use for this pin
    * @param[in] pin port-pin to use for this pin
    * @param[in] mode pinmode to use
    */
   void Configure(uint32_t port, uint16_t pin, PinMode::PinMode pinMode);

   /**
   * Get pin value
   *
   * @param[in] io pin index
   * @return pin value
   */
   // bool Get() { return gpio_get(_port, _pin) > 0; }
   bool Get() { return palReadPad(PAL_PORT(_line), PAL_PAD(_line)) > 0; }

   /**
   * Set pin high
   *
   * @param[in] io pin index
   */
   // void Set() { gpio_set(_port, _pin); }
   void Set() { palSetPad(PAL_PORT(_line), PAL_PAD(_line)); }

   /**
   * Set pin low
   *
   * @param[in] io pin index
   */
   // void Clear() { gpio_clear(_port, _pin); }
   void Clear() { palClearPad(PAL_PORT(_line), PAL_PAD(_line)); }

   /**
   * Toggle pin
   *
   * @param[in] io pin index
   */
   // void Toggle() { gpio_toggle(_port, _pin); }
   void Toggle() { palTogglePad(PAL_PORT(_line), PAL_PAD(_line)); }

private:
   uint32_t _port;
   uint32_t _pin;
   uint32_t _line = PAL_LINE(_port, _pin);
};
//Configure all digio objects from the given list
#define DIG_IO_ENTRY(name, port, pin, mode) DigIo::name.Configure(port, pin, mode);
#define DIG_IO_CONFIGURE(l) l

#endif // DIGIO_H_INCLUDED
