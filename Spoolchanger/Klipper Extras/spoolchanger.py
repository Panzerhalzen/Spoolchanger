# Belay extruder-syncing + dual-feeder switching
#
# Derivated from:
#   Copyright (C) 2023-2025 Ryan Ghosh <rghosh776@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

import logging


class Belay:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.name = config.get_name()

        # extruder stepper names – left & right feeders
        self.extruder_stepper_left = config.get("extruder_stepper_left")
        self.extruder_stepper_right = config.get("extruder_stepper_right")

        # parameters
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
        # real rotation_distance from stepper config (populated in _handle_connect)
        self.left_base_rd = None
        self.right_base_rd = None

        # pin states
        self.left_pin_pressed = False
        self.right_pin_pressed = False

        # mode flags
        self.enabled = False
        self.fast_mode = False

        # register buttons
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
        self.printer.register_event_handler("klippy:ready", self._handle_ready)

    # ---- connection / ready ----

    def _handle_connect(self):
        self.toolhead = self.printer.lookup_object("toolhead")

        # lookup left feeder – save full object and raw stepper
        left_obj = self.printer.lookup_object(
            "extruder_stepper {}".format(self.extruder_stepper_left)
        )
        self.left_extruder_stepper = left_obj.extruder_stepper
        self.left_stepper = self.left_extruder_stepper.stepper
        self.left_base_rd = self.left_stepper.get_rotation_distance()[0]

        # lookup right feeder – save full object and raw stepper
        right_obj = self.printer.lookup_object(
            "extruder_stepper {}".format(self.extruder_stepper_right)
        )
        self.right_extruder_stepper = right_obj.extruder_stepper
        self.right_stepper = self.right_extruder_stepper.stepper
        self.right_base_rd = self.right_stepper.get_rotation_distance()[0]

    def _handle_ready(self):
        pass

    # ---- stepper helpers ----

    def _sync_stepper(self, stepper, enable):
        """Sync or unsync an extruder_stepper with the main extruder.
        Caller must flush_step_generation() before calling this."""
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
                        "Spoolchanger: '%s' syncing with extruder" % name
                    )
            else:
                stepper_obj.sync_to_extruder("")
                if self.debug_level >= 1:
                    self.gcode.respond_info(
                        "Spoolchanger: '%s' unsynced from extruder" % name
                    )
        except Exception as e:
            logging.exception("Spoolchanger sync failed: %s", e)

    def _set_rotation_distance(self, stepper, distance):
        """Set rotation_distance on a stepper directly.
        Caller must flush_step_generation() before calling this."""
        try:
            stepper.set_rotation_distance(distance)
        except Exception as e:
            logging.exception("Belay set_rotation_distance failed: %s", e)

    # ---- feeder callbacks ----

    def _left_feeder_callback(self, eventtime, state):
        with self.mutex:
            if self.debug_level >= 2:
                self.gcode.respond_info(
                    "Belay: left_feeder state=%s" % ("PRESSED" if state else "RELEASED")
                )
            if state:
                self.left_pin_pressed = True
                if self.active_stepper is None:
                    self.active_stepper = self.left_stepper
                    self.toolhead.flush_step_generation()
                    self._sync_stepper(self.active_stepper, True)
                    self._enable()
            else:
                self.left_pin_pressed = False

    def _right_feeder_callback(self, eventtime, state):
        with self.mutex:
            if self.debug_level >= 2:
                self.gcode.respond_info(
                    "Belay: right_feeder state=%s" % ("PRESSED" if state else "RELEASED")
                )
            if state:
                self.right_pin_pressed = True
                if self.active_stepper is None:
                    self.active_stepper = self.right_stepper
                    self.toolhead.flush_step_generation()
                    self._sync_stepper(self.active_stepper, True)
                    self._enable()
            else:
                self.right_pin_pressed = False

    def _base_rd_for(self, stepper):
        """Return the base rotation_distance for the given stepper."""
        if stepper is self.left_stepper:
            return self.left_base_rd
        elif stepper is self.right_stepper:
            return self.right_base_rd
        return self.left_base_rd  # fallback

    def _hub_callback(self, eventtime, state):
        with self.mutex:
            if self.debug_level >= 2:
                self.gcode.respond_info(
                    "Belay: hub state=%s" % ("PRESSED" if state else "RELEASED")
                )
            if state:
                return

            if self.active_stepper is None:
                return

            if self.active_stepper is self.left_stepper:
                other_stepper = self.right_stepper
                other_pressed = self.right_pin_pressed
            else:
                other_stepper = self.left_stepper
                other_pressed = self.left_pin_pressed

            if not other_pressed:
                self._disable()
                self.toolhead.flush_step_generation()
                self._sync_stepper(self.active_stepper, False)
                self.active_stepper = None
                self.fast_mode = False
                if self.debug_level >= 1:
                    self.gcode.respond_info(
                        "Belay: no filament in spare – feeder disabled"
                    )
                return

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
                    "Belay: switched feeder – fast mode (rd = %.3f)" % rd_fast
                )

    # ---- buffer sensor callbacks ----

    def _buffer_full_callback(self, eventtime, state):
        """Buffer full sensor – triggers on PRESS (state == True).
           Increases rotation_distance → reduces feed."""
        with self.mutex:
            if self.debug_level >= 2:
                self.gcode.respond_info(
                    "Belay: buffer_full state=%s" % ("PRESSED" if state else "RELEASED")
                )

            # react only on PRESS
            if not state:
                return

            if not self.enabled or self.active_stepper is None:
                return

            # exit fast mode on buffer_full trigger
            if self.fast_mode:
                self.fast_mode = False
                rd = self._base_rd_for(self.active_stepper) * self.multiplier_high
                self._set_rotation_distance(self.active_stepper, rd)
                if self.debug_level >= 1:
                    self.gcode.respond_info(
                        "Belay: fast mode ended – buffer full (rd = %.3f)" % rd
                    )
                return

            # normal operation: buffer full → RD up → feed down
            rd = self._base_rd_for(self.active_stepper) * self.multiplier_high
            self._set_rotation_distance(self.active_stepper, rd)
            if self.debug_level >= 1:
                self.gcode.respond_info(
                    "Belay: rotation_distance = %.3f (buffer full)" % rd
                )

    def _buffer_empty_callback(self, eventtime, state):
        """Buffer empty sensor – triggers on PRESS (state == True).
           Decreases rotation_distance → increases feed."""
        with self.mutex:
            if self.debug_level >= 2:
                self.gcode.respond_info(
                    "Belay: buffer_empty state=%s" % ("PRESSED" if state else "RELEASED")
                )

            # react only on PRESS
            if not state:
                return

            if not self.enabled or self.active_stepper is None:
                return

            # buffer empty → RD down → feed up
            rd = self._base_rd_for(self.active_stepper) * self.multiplier_low
            self._set_rotation_distance(self.active_stepper, rd)
            if self.debug_level >= 1:
                self.gcode.respond_info(
                    "Belay: rotation_distance = %.3f (buffer empty)" % rd
                )

    # ---- enable / disable ----

    def _enable(self):
        if self.active_stepper is None:
            return
        self.enabled = True
        # don't override rd if fast mode is active
        if not self.fast_mode:
            self._set_rotation_distance(
                self.active_stepper,
                self._base_rd_for(self.active_stepper)
            )

    def _disable(self):
        self.enabled = False

    # ---- gcode commands ----

    _cmd_QUERY_BELAY_help = "Report Belay sensor and active feeder state"

    def _cmd_QUERY_BELAY(self, gcmd):
        if self.active_stepper is self.left_stepper:
            feeder = "left"
        elif self.active_stepper is self.right_stepper:
            feeder = "right"
        else:
            feeder = "none"
        self.gcode.respond_info(
            "belay {}: feeder={}, fast={}, enabled={}".format(
                self.name, feeder, self.fast_mode, self.enabled
            )
        )

    _cmd_ENABLE_BELAY_help = "Enable Belay extrusion multiplier adjustment"

    def _cmd_ENABLE_BELAY(self, gcmd):
        self._enable()
        if not self.enabled:
            raise self.printer.command_error(
                "Conditions not met to enable belay {}".format(self.name)
            )

    _cmd_DISABLE_BELAY_help = "Disable Belay extrusion multiplier adjustment"

    def _cmd_DISABLE_BELAY(self, gcmd):
        if gcmd.get_int("OVERRIDE", 0):
            self._disable()
        else:
            self._disable()
        if self.enabled:
            raise self.printer.command_error(
                "Conditions not met to disable belay {}".format(self.name)
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
