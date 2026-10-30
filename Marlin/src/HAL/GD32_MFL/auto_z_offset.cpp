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

#include "../../inc/MarlinConfig.h"

#if defined(ARDUINO_ARCH_MFL) && ENABLED(AUTO_Z_OFFSET)

#include "auto_z_offset.h"

#include "../../core/serial.h"
#include "../../MarlinCore.h"
#include "../../module/motion.h"
#include "../../module/probe.h"
#include "../../module/planner.h"
#include "../../module/settings.h"
#include "../../module/temperature.h"
#include "../../feature/bedlevel/bedlevel.h"
#include "../../gcode/gcode.h"

/**
 * Reference switch and nozzle preparation, taken verbatim from the stock
 * Anycubic sources (autoGetZoffset.h):
 *
 *   NOZZLE_X 60     NOZZLE_Y 240             reference switch position
 *   NOZZLE_TEMP 220 NOZZLE_COOL_TEMP 140     purge / contact temperatures
 *   LOAD_LENGTH 120 LOAD_SPEED 8             purge
 *   UNLOAD_LENGTH 50 UNLOAD_SPEED 20         retract
 *   WIPE_Z -2.0                              nozzle sinks into the silicone pad
 *   wipe at switchX+25 .. switchX+30, 3 passes, 10 mm/s
 */
#ifndef AZO_SWITCH_X
  #define AZO_SWITCH_X                60.0f
#endif
#ifndef AZO_SWITCH_Y
  #define AZO_SWITCH_Y               240.0f
#endif

#ifndef AZO_PURGE_TEMP
  #define AZO_PURGE_TEMP                220  // (°C)
#endif
#ifndef AZO_LOAD_LENGTH
  #define AZO_LOAD_LENGTH             120.0f  // (mm)
#endif
#ifndef AZO_LOAD_SPEED
  #define AZO_LOAD_SPEED                8.0f  // (mm/s)
#endif
#ifndef AZO_UNLOAD_LENGTH
  #define AZO_UNLOAD_LENGTH            50.0f  // (mm)
#endif
#ifndef AZO_UNLOAD_SPEED
  #define AZO_UNLOAD_SPEED             20.0f  // (mm/s)
#endif
#ifndef AZO_WIPE_Z
  #define AZO_WIPE_Z                  -2.00f  // (mm)
#endif
#ifndef AZO_WIPE_X1
  #define AZO_WIPE_X1                  25.0f  // (mm from the switch)
#endif
#ifndef AZO_WIPE_X2
  #define AZO_WIPE_X2                  30.0f  // (mm from the switch)
#endif
#ifndef AZO_WIPE_PASSES
  #define AZO_WIPE_PASSES                 3
#endif
/** Stock firmware cools to NOZZLE_COOL_TEMP + 10 before touching the switch. */
#ifndef AZO_CONTACT_TEMP
  #define AZO_CONTACT_TEMP              150  // (°C)
#endif

/** Descent / approach parameters for the contact measurement. */
#ifndef AZO_FEEDRATE
  #define AZO_FEEDRATE                   3  // (mm/s)
#endif
#ifndef AZO_STEP
  #define AZO_STEP                    0.05f  // (mm)
#endif
#ifndef AZO_MAX_DESCENT
  #define AZO_MAX_DESCENT             8.00f  // (mm)
#endif
#ifndef AZO_APPROACH_Z
  #define AZO_APPROACH_Z              2.00f  // (mm)
#endif

/**
 * The switch reports a little after the nozzle has passed its resting surface.
 * Tune this if the first layer is consistently too high or too low.
 */
#ifndef AZO_SWITCH_TRAVEL
  #define AZO_SWITCH_TRAVEL           0.00f  // (mm) The switch triggers on contact
#endif

bool auto_z_offset_calibrate(const bool prepare/*=true*/) {
  if (!motion.all_axes_homed()) {
    SERIAL_ECHOLNPGM("Auto Z offset: home all axes first");
    return false;
  }

  // The reference switch sits outside the printable area
  motion.set_soft_endstop_loose(true);

  #if HAS_LEVELING
    const bool saved_leveling = planner.leveling_active;
    if (saved_leveling) set_bed_leveling_enabled(false);
  #endif

  // Z_MAX_PIN carries the nozzle-contact signal on this board
  SET_INPUT_PULLUP(Z_MAX_PIN);

  if (prepare) {
  // --- 1. Purge and retract at the front-left corner -----------------------
  motion.blocking_move_z(motion.position.z + Z_CLEARANCE_BETWEEN_PROBES, feedRate_t(5));
  motion.blocking_move_xy(X_MIN_POS, Y_MIN_POS, feedRate_t(40));
  planner.synchronize();

  thermalManager.setTargetHotend(AZO_PURGE_TEMP, 0);
  thermalManager.wait_for_hotend(0);

  motion.unscaled_e_move(AZO_LOAD_LENGTH, feedRate_t(AZO_LOAD_SPEED));
  motion.unscaled_e_move(-AZO_UNLOAD_LENGTH, feedRate_t(AZO_UNLOAD_SPEED));
  planner.synchronize();

  // --- 2. Wipe the nozzle on the silicone pad beside the switch -----------
  motion.blocking_move_xy(AZO_SWITCH_X + AZO_WIPE_X1, AZO_SWITCH_Y, feedRate_t(50));
  motion.blocking_move_z(AZO_WIPE_Z, feedRate_t(5));
  planner.synchronize();
  for (uint8_t i = 0; i < AZO_WIPE_PASSES; ++i) {
    motion.blocking_move_xy(AZO_SWITCH_X + AZO_WIPE_X1, AZO_SWITCH_Y, feedRate_t(10));
    motion.blocking_move_xy(AZO_SWITCH_X + AZO_WIPE_X2, AZO_SWITCH_Y, feedRate_t(10));
  }

  // --- 3. Cool down so the nozzle neither oozes nor marks the switch ------
  motion.blocking_move_xy(AZO_SWITCH_X + AZO_WIPE_X1 + 2.0f, AZO_SWITCH_Y, feedRate_t(50));
  if (thermalManager.degHotend(0) > AZO_CONTACT_TEMP) {
    thermalManager.setTargetHotend(0, 0);
    thermalManager.set_fan_speed(0, 255);
    while (thermalManager.degHotend(0) > AZO_CONTACT_TEMP) { marlin.idle(); safe_delay(500); }
    thermalManager.set_fan_speed(0, 0);
  }

  }
  else
    SERIAL_ECHOLNPGM("Auto Z offset: preparation skipped");

  // --- 4. Touch the nozzle-contact reference switch -----------------------
  motion.blocking_move_z(AZO_APPROACH_Z + Z_CLEARANCE_BETWEEN_PROBES, feedRate_t(5));
  motion.blocking_move_xy(AZO_SWITCH_X, AZO_SWITCH_Y, feedRate_t(50));
  planner.synchronize();

  // A switch that already reads closed cannot measure anything
  if (READ(Z_MAX_PIN)) {
    SERIAL_ECHOLNPGM("Auto Z offset: reference switch already closed");
    motion.blocking_move_z(AZO_APPROACH_Z + Z_CLEARANCE_BETWEEN_PROBES, feedRate_t(5));
    motion.set_soft_endstop_loose(false);
    #if HAS_LEVELING
      if (saved_leveling) set_bed_leveling_enabled(true);
    #endif
    return false;
  }

  motion.blocking_move_z(AZO_APPROACH_Z, feedRate_t(5));
  planner.synchronize();

  float z_contact = NAN;
  const float z_floor = motion.position.z - AZO_MAX_DESCENT;
  while (motion.position.z > z_floor) {
    if (READ(Z_MAX_PIN)) { z_contact = motion.position.z; break; }
    motion.blocking_move_z(motion.position.z - AZO_STEP, feedRate_t(AZO_FEEDRATE));
    if (READ(Z_MAX_PIN)) { z_contact = motion.position.z; break; }
    marlin.idle();
  }

  // Back to a safe height above the bed plane
  motion.blocking_move_z(AZO_APPROACH_Z + Z_CLEARANCE_BETWEEN_PROBES, feedRate_t(5));
  planner.synchronize();

  bool ok = !isnan(z_contact), saved = false;

  if (ok) {
    // Marlin's Z=0 is the probe trigger point, so the switch height IS the
    // remaining offset error
    const float delta = z_contact + AZO_SWITCH_TRAVEL;
    probe.offset.z = constrain(probe.offset.z + delta, -10.0f, 10.0f);
    saved = settings.save();
  }

  motion.set_soft_endstop_loose(false);
  #if HAS_LEVELING
    if (saved_leveling) set_bed_leveling_enabled(true);
  #endif

  if (ok)
    SERIAL_ECHOLNPGM("Auto Z offset: contact at Z", z_contact, ", offset Z", probe.offset.z, saved ? " (saved)" : " (SAVE FAILED)");
  else
    SERIAL_ECHOLNPGM("Auto Z offset: reference switch not reached");

  SERIAL_ECHOLNPGM("Auto Z offset: run G28 Z to apply the new offset");

  return ok;
}

/**
 * M44: Run the automatic Z offset calibration
 *
 *   S0  Skip heating, purging and wiping (fast contact test)
 *   P   Only report the nozzle-contact reference switch state
 */
void GcodeSuite::M44() {
  if (parser.seen('P')) {
    SET_INPUT_PULLUP(Z_MAX_PIN);
    SERIAL_ECHOPGM("PA0 nozzle-contact switch: ");
    SERIAL_ECHOLN(READ(Z_MAX_PIN) ? "TRIGGERED" : "open");
    return;
  }
  auto_z_offset_calibrate(!parser.seenval('S') || parser.value_bool());
}

#endif // ARDUINO_ARCH_MFL && AUTO_Z_OFFSET
