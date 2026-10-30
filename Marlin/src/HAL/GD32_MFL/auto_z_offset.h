/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2026 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
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
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */
#pragma once

#include "../../inc/MarlinConfig.h"

#if defined(ARDUINO_ARCH_MFL) && ENABLED(AUTO_Z_OFFSET)

/**
 * Measure the Z probe offset with the nozzle-contact reference switch.
 *
 * The Kobra 2 Neo has a spring-loaded switch beside the bed (X60, Y230) whose
 * top surface sits at bed height. Z homing puts Marlin's Z=0 at the probe
 * trigger point, so pushing the nozzle down onto that switch measures the
 * remaining offset error directly. The result is stored in probe.offset.z,
 * saved to EEPROM, and Z is re-homed with the new value.
 *
 * Returns true when the switch was reached and the offset was saved.
 */
bool auto_z_offset_calibrate(const bool prepare=true);

#endif
