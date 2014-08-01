#!/usr/bin/env python
'''
Pre-flight module for ACS
Mike Clement
July 2014
Inspired by the Checklist module by Stephen Dade
'''

import sys, os, time
from pymavlink import mavutil
from MAVProxy.modules.lib import mp_preflight
from MAVProxy.modules.lib import mp_module

class PreFlightModule(mp_module.MPModule):
    def __init__(self, mpstate):
        super(PreFlightModule, self).__init__(mpstate, "preflight", "preflight module", public=True)
        self.preflight = mp_preflight.PreFlightUI()

    def mavlink_packet(self, msg):
        '''handle an incoming mavlink packet'''
        if not isinstance(self.preflight, mp_preflight.PreFlightUI):
            return
        if not self.preflight.is_alive():
            return
        
        type = msg.get_type()
        master = self.master

        if type == 'STATUSTEXT':
            if str(msg.text) == "zero airspeed calibrated":
                self.preflight.send_value("zeroize", "")

        elif type == 'VFR_HUD':
            self.preflight.send_value("airspeed", msg.airspeed)

        elif type == 'ATTITUDE':
            self.preflight.send_value("pitch", msg.pitch)
            self.preflight.send_value("roll", msg.roll)

        elif type == 'HEARTBEAT':
            armed = msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
            self.preflight.send_value("armed", bool(armed))
            self.preflight.send_value("mode", master.flightmode)

        elif type == 'GLOBAL_POSITION_INT':
            self.preflight.send_value("relalt", int(msg.relative_alt / 1000))

        elif type == 'GPS_RAW_INT':
            locked = "NO LOCK (fix = %u)" % msg.fix_type
            if ((msg.fix_type == 3 and master.mavlink10()) or
                (msg.fix_type == 2 and not master.mavlink10())):
                locked = "LOCKED"
            self.preflight.send_value("gps", (locked, msg.satellites_visible))

        elif type == 'PARAM_VALUE':
            self.preflight.send_value("param", (msg.param_id, msg.param_value))

        elif type == 'RALLY_POINT':
            self.preflight.send_value("rally", (msg.alt, msg.break_alt))

        elif type == 'MISSION_ITEM':
            if msg.command == 22: # Takeoff type
                self.preflight.send_value("takeoff", msg.z)

    def idle_task(self):
        '''run periodically, handle messages from child (UI)'''
        while True:
            name, value = self.preflight.recv_value()
            if name is None:
                return

            if name == "cmd":
                self.mpstate.functions.process_stdin(str(value), True)

            elif name == "param_all":
                self.master.param_fetch_all()

            elif name == "param":
                if value in self.master.params:
                    self.preflight.send_value("param", (name, self.master.params[name]))
                else:
                    self.master.param_fetch_one(name)

def init(mpstate):
    '''initialise module'''
    return PreFlightModule(mpstate)
