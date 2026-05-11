# Belay extruder-syncing + dual-feeder switching
#
# Copyright (C) 2023-2025 Ryan Ghosh <rghosh776@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
#
# === OVERVIEW ===
#
# Spoolchanger is a dual-feeder manager for Klipper 3D printers.
# It synchronizes one of two extruder_stepper motors with the main extruder
# and adjusts feed rate in real-time based on physical sensors.
#
# === HARDWARE ===
#
# 5 input pins are required:
#   left_feeder_pin  – filament presence sensor in the left feeder
#   right_feeder_pin – filament presence sensor in the right feeder
#   hub_pin          – mechanical switch, pressed when a feeder is actively
#                      pushing filament through the hub. Released when the
#                      current filament runs out (extruder drags it out).
#   buffer_full_pin  – buffer overflow sensor (too much slack)
#   buffer_empty_pin – buffer underflow sensor (too little slack)
#
# === OPERATION LOGIC ===
#
# [Feeder activation]
# When a filament is inserted into a feeder (pin pressed) and no feeder
# is currently active, that feeder becomes active – its stepper is synced
# with the main extruder and the Spoolchanger system is enabled.
#
# [Hub-based switchover]
# When the hub pin is released (active filament ran out and got pulled out
# of the hub), the system checks the opposite feeder:
#   - If the other feeder has filament → switch to it. rotation_distance
#     is set to base * multiplier_fast (fast mode) to quickly take up
#     any slack introduced during the switch.
#   - If the other feeder is empty → disable Spoolchanger.
#
# While the hub is pressed, feeder pin events are ignored – the mechanical
# hub state takes precedence for switchover decisions.
#
# [Manual feeder change without hub]
# If no hub is pressed, removing filament from the active feeder
# deactivates it (synced stepper becomes available for another feeder).
#
# [Buffer feedback control]
# Two buffer sensors provide closed-loop feed rate adjustment:
#   buffer_full  → rotation_distance *= multiplier_high (>1) → slows feed
#   buffer_empty → rotation_distance *= multiplier_low  (<1) → speeds feed
# This maintains optimal filament tension.
#
# === GCODE COMMANDS ===
#
# QUERY_BELAY BELAY=<name>           – report current state
# ENABLE_BELAY BELAY=<name>          – manually enable Belay
# DISABLE_BELAY BELAY=<name>         – manually disable Belay
# BELAY_SET_MULTIPLIER BELAY=<name>  – set multiplier_high/multiplier_low
# BELAY_CLEAR_OVERRIDE BELAY=<name>  – reserved (no-op in this version)

import logging


class Belay:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.name = config.get_name()

        # extruder stepper names – left & right feeders
        self.extruder_stepper_left = config.get("extruder_stepper_left")
        self.extruder_stepper_right = config.get("extruder_stepper_right")

        # configurable multipliers
        self.multiplier_fast = config.getfloat(
            "multiplier_fast", 0.7, above=0.0, maxval=1.0
        )
        self.multiplier_low = config.getfloat(
            "multiplier_low", 0.85, minval=0.5, maxval=1.0
        )
        self.multiplier_high = config.getfloat(
            "multiplier_high", 1.05, minval=1.0
        )
        self.debug_level = config.getint(
            "debug_level", default=0, minval=0, maxval=2
        )

        # full extruder_stepper objects (for sync_to_extruder)
        self.left_extruder_stepper = None
        self.right_extruder_stepper = None

        # raw stepper references (for set_rotation_distance)
        self.left_stepper = None
        self.right_stepper = None
        self.active_stepper = None

        # base rotation_distance values (read from stepper config)
        self.left_base_rd = None
        self.right_base_rd = None

        # pin states
        self.left_pin_pressed = False
        self.right_pin_pressed = False
        self.hub_pin_pressed = False

        # mode flags
        self.enabled = False
        self.fast_mode = False
        self.buffer_full_state = False
        self.buffer_empty_state = False

        # register input pins as buttons
        buffer_full_pin = config.get("buffer_full_pin")
        buffer_empty_pin = config.get("buffer_empty_pin")
        left_feeder_pin = config.get("left_feeder_pin")
        right_feeder_pin = config.get("right_feeder_pin")
        hub_pin = config.get("hub_pin")

        buttons = self.printer.load_object(config, "buttons")
        buttons.register_buttons([buffer_full_pin], self._buffer_full_callback)
        buttons.register_buttons([buffer_empty_pin], self._buffer_empty_callback)
        buttons.register_buttons([left_feeder_pin], self._left_feeder_callback)
        buttons.register_buttons([right_feeder_pin], self._right_feeder_callback)
        buttons.register_buttons([hub_pin], self._hub_callback)

        # reference holders
        self.gcode = self.printer.lookup_object("gcode")
        self.mutex = self.gcode.get_mutex()
        self.toolhead = None

        # register gcode commands
        self.gcode.register_mux_command(
            "QUERY_BELAY",
            "BELAY",
            self.name,
            self._cmd_QUERY_BELAY,
            desc=self._cmd_QUERY_BELAY_help,
        )
        self.gcode.register_mux_command(
            "ENABLE_BELAY",
            "BELAY",
            self.name,
            self._cmd_ENABLE_BELAY,
            desc=self._cmd_ENABLE_BELAY_help,
        )
        self.gcode.register_mux_command(
            "DISABLE_BELAY",
            "BELAY",
            self.name,
            self._cmd_DISABLE_BELAY,
            desc=self._cmd_DISABLE_BELAY_help,
        )
        self.gcode.register_mux_command(
            "BELAY_SET_MULTIPLIER",
            "BELAY",
            self.name,
            self._cmd_BELAY_SET_MULTIPLIER,
            desc=self._cmd_BELAY_SET_MULTIPLIER_help,
        )
        self.gcode.register_mux_command(
            "BELAY_CLEAR_OVERRIDE",
            "BELAY",
            self.name,
            self._cmd_BELAY_CLEAR_OVERRIDE,
            desc=self._cmd_BELAY_CLEAR_OVERRIDE_help,
        )

        # register event handlers
        self.printer.register_event_handler("klippy:connect", self._handle_connect)

    # ======== connection / ready ========

    def _handle_connect(self):
        self.toolhead = self.printer.lookup_object("toolhead")

        # lookup left feeder stepper
        left_obj = self.printer.lookup_object(
            "extruder_stepper {}".format(self.extruder_stepper_left)
        )
        self.left_extruder_stepper = left_obj.extruder_stepper
        self.left_stepper = self.left_extruder_stepper.stepper
        self.left_base_rd = self.left_stepper.get_rotation_distance()[0]

        # lookup right feeder stepper
        right_obj = self.printer.lookup_object(
            "extruder_stepper {}".format(self.extruder_stepper_right)
        )
        self.right_extruder_stepper = right_obj.extruder_stepper
        self.right_stepper = self.right_extruder_stepper.stepper
        self.right_base_rd = self.right_stepper.get_rotation_distance()[0]


    # ======== stepper helpers ========

    def _sync_stepper(self, stepper, enable):
        """Sync or unsync an extruder_stepper with the main extruder.
        Must call flush_step_generation() before calling this."""
        try:
            stepper_obj = (
                self.left_extruder_stepper
                if stepper is self.left_stepper
                else self.right_extruder_stepper
            )
            name = (
                self.extruder_stepper_left
                if stepper is self.left_stepper
                else self.extruder_stepper_right
            )
            if enable:
                stepper_obj.sync_to_extruder("extruder")
                if self.debug_level >= 1:
                    self.gcode.respond_info(
                        "Belay: '%s' synced with extruder" % name
                    )
            else:
                stepper_obj.sync_to_extruder("")
                if self.debug_level >= 1:
                    self.gcode.respond_info(
                        "Belay: '%s' unsynced from extruder" % name
                    )
        except Exception as e:
            logging.exception("Belay sync failed: %s", e)

    def _set_rotation_distance(self, stepper, distance):
        """Set rotation_distance on a raw stepper directly.
        Must call flush_step_generation() before calling this."""
        try:
            stepper.set_rotation_distance(distance)
        except Exception as e:
            logging.exception("Belay set_rotation_distance failed: %s", e)

    # ======== feeder callbacks ========

    def _left_feeder_callback(self, eventtime, state):
        """Called when the left feeder filament sensor changes state."""
        with self.mutex:
            if self.debug_level >= 2:
                self.gcode.respond_info(
                    "Belay: left_feeder state=%s"
                    % ("PRESSED" if state else "RELEASED")
                )

            if state:
                self.left_pin_pressed = True
                if self.active_stepper is None and not self.hub_pin_pressed:
                    self._activate_feeder(self.left_stepper)
            else:
                self.left_pin_pressed = False
                if self.active_stepper is self.left_stepper and not self.hub_pin_pressed:
                    self._deactivate_feeder("left")

    def _right_feeder_callback(self, eventtime, state):
        """Called when the right feeder filament sensor changes state."""
        with self.mutex:
            if self.debug_level >= 2:
                self.gcode.respond_info(
                    "Belay: right_feeder state=%s"
                    % ("PRESSED" if state else "RELEASED")
                )

            if state:
                self.right_pin_pressed = True
                if self.active_stepper is None and not self.hub_pin_pressed:
                    self._activate_feeder(self.right_stepper)
            else:
                self.right_pin_pressed = False
                if self.active_stepper is self.right_stepper and not self.hub_pin_pressed:
                    self._deactivate_feeder("right")

    def _base_rd_for(self, stepper):
        """Return the base rotation_distance for the given stepper."""
        if stepper is self.left_stepper:
            return self.left_base_rd
        elif stepper is self.right_stepper:
            return self.right_base_rd
        return self.left_base_rd  # fallback

    def _activate_feeder(self, stepper):
        """Activate a feeder: sync its stepper and enable Belay."""
        self.active_stepper = stepper
        self.toolhead.flush_step_generation()
        self._sync_stepper(self.active_stepper, True)
        self._enable()

    def _deactivate_feeder(self, side_name):
        """Deactivate the current feeder: unsync, clear state."""
        self._disable()
        self.toolhead.flush_step_generation()
        self._sync_stepper(self.active_stepper, False)
        self.active_stepper = None
        self.fast_mode = False
        if self.debug_level >= 1:
            self.gcode.respond_info(
                "Belay: %s feeder deactivated" % side_name
            )

    # ======== buffer sensor and hub callbacks ========

    def _hub_callback(self, eventtime, state):
        """Called when the hub sensor changes state.
        Switchover happens on RELEASE (state == False)."""
        with self.mutex:
            if self.debug_level >= 2:
                self.gcode.respond_info(
                    "Belay: hub state=%s" % ("PRESSED" if state else "RELEASED")
                )
            self.hub_pin_pressed = state

            # we only act on release
            if state:
                return

            if self.active_stepper is None:
                return

            # determine the opposite feeder
            if self.active_stepper is self.left_stepper:
                other_stepper = self.right_stepper
                other_pressed = self.right_pin_pressed
            else:
                other_stepper = self.left_stepper
                other_pressed = self.left_pin_pressed

            if not other_pressed:
                # no filament in the spare feeder – shut down
                self._disable()
                self.toolhead.flush_step_generation()
                self._sync_stepper(self.active_stepper, False)
                self.active_stepper = None
                self.fast_mode = False
                if self.debug_level >= 1:
                    self.gcode.respond_info(
                        "Belay: spare feeder empty – disabled"
                    )
                return

            # switch to the other feeder with fast mode
            self._disable()
            self.toolhead.flush_step_generation()
            self._sync_stepper(self.active_stepper, False)

            self.active_stepper = other_stepper
            self.fast_mode = True
            rd_fast = self._base_rd_for(self.active_stepper) * self.multiplier_fast

            self.toolhead.flush_step_generation()
            self._set_rotation_distance(self.active_stepper, rd_fast)
            self._sync_stepper(self.active_stepper, True)
            self._enable()

            if self.debug_level >= 1:
                self.gcode.respond_info(
                    "Belay: switched to %s (fast mode, rd = %.3f)"
                    % ("left" if other_stepper is self.left_stepper else "right",
                       rd_fast)
                )

    def _buffer_full_callback(self, eventtime, state):
        """Buffer full sensor – triggers on PRESS (state == True).
        Increases rotation_distance → slows down feed."""
        with self.mutex:
            self.buffer_full_state = state
            if self.debug_level >= 2:
                self.gcode.respond_info(
                    "Belay: buffer_full state=%s"
                    % ("PRESSED" if state else "RELEASED")
                )

            if not state:
                return
            if not self.enabled or self.active_stepper is None:
                return

            # exit fast mode when buffer fills up
            if self.fast_mode:
                self.fast_mode = False
                rd = self._base_rd_for(self.active_stepper) * self.multiplier_high
                self._set_rotation_distance(self.active_stepper, rd)
                if self.debug_level >= 1:
                    self.gcode.respond_info(
                        "Belay: fast mode ended – buffer full (rd = %.3f)" % rd
                    )
                return

            rd = self._base_rd_for(self.active_stepper) * self.multiplier_high
            self._set_rotation_distance(self.active_stepper, rd)
            if self.debug_level >= 1:
                self.gcode.respond_info(
                    "Belay: rd = %.3f (buffer full)" % rd
                )

    def _buffer_empty_callback(self, eventtime, state):
        """Buffer empty sensor – triggers on PRESS (state == True).
        Decreases rotation_distance → speeds up feed."""
        with self.mutex:
            self.buffer_empty_state = state
            if self.debug_level >= 2:
                self.gcode.respond_info(
                    "Belay: buffer_empty state=%s"
                    % ("PRESSED" if state else "RELEASED")
                )

            if not state:
                return
            if not self.enabled or self.active_stepper is None:
                return

            rd = self._base_rd_for(self.active_stepper) * self.multiplier_low
            self._set_rotation_distance(self.active_stepper, rd)
            if self.debug_level >= 1:
                self.gcode.respond_info(
                    "Belay: rd = %.3f (buffer empty)" % rd
                )

    # ======== enable / disable ========

    def _enable(self):
        """Enable Belay and set initial rotation_distance based on buffer state."""
        if self.active_stepper is None:
            return
        self.enabled = True
        if not self.fast_mode:
            if self.buffer_full_state:
                rd = self._base_rd_for(self.active_stepper) * self.multiplier_high
            else:
                rd = self._base_rd_for(self.active_stepper) * self.multiplier_low
            self._set_rotation_distance(self.active_stepper, rd)

    def _disable(self):
        """Disable Belay, stop adjusting rotation_distance."""
        self.enabled = False

    # ======== gcode commands ========

    _cmd_QUERY_BELAY_help = "Report Belay sensor and active feeder state"

    def _cmd_QUERY_BELAY(self, gcmd):
        if self.active_stepper is self.left_stepper:
            feeder = "left"
        elif self.active_stepper is self.right_stepper:
            feeder = "right"
        else:
            feeder = "none"
        self.gcode.respond_info(
            "Belay '%s': feeder=%s, fast=%s, enabled=%s"
            % (self.name, feeder, self.fast_mode, self.enabled)
        )

    _cmd_ENABLE_BELAY_help = "Enable Belay extrusion multiplier adjustment"

    def _cmd_ENABLE_BELAY(self, gcmd):
        self._enable()
        if not self.enabled:
            raise self.printer.command_error(
                "Conditions not met to enable Belay '%s'" % self.name
            )

    _cmd_DISABLE_BELAY_help = "Disable Belay extrusion multiplier adjustment"

    def _cmd_DISABLE_BELAY(self, gcmd):
        self._disable()
        if self.enabled:
            raise self.printer.command_error(
                "Conditions not met to disable Belay '%s'" % self.name
            )

    _cmd_BELAY_SET_MULTIPLIER_help = (
        "Sets multiplier_high and/or multiplier_low. Does not persist across"
        " restarts."
    )

    def _cmd_BELAY_SET_MULTIPLIER(self, gcmd):
        self.multiplier_high = gcmd.get_float(
            "HIGH", self.multiplier_high, minval=1.0
        )
        self.multiplier_low = gcmd.get_float(
            "LOW", self.multiplier_low, minval=0.0, maxval=1.0
        )

    _cmd_BELAY_CLEAR_OVERRIDE_help = (
        "Clears any user override that would prevent the Belay from being"
        " automatically enabled"
    )

    def _cmd_BELAY_CLEAR_OVERRIDE(self, gcmd):
        pass  # no override logic in this version

    def get_status(self, eventtime):
        return {
            "enabled": self.enabled,
            "fast_mode": self.fast_mode,
        }


def load_config(config):
    return Belay(config)
