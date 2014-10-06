#!/usr/bin/env python
'''
UI for Pre-flight module for ACS
Mike Clement
July 2014
Inspired by the Checklist module by Stephen Dade
'''

import wx, sys, math, time

class NameValue():
    # Simple name-value pair struct for pipe IPC
    def __init__(self, name, value):
        self.name = name
        self.value = value


class PreFlightUI():

    # API for parent process (MAVProxy module)

    def __init__(self, title='MAVProxy: ACS Pre-Flight', no_timer=False):
        import multiprocessing, threading
        self.title  = title
        self.no_timer = no_timer
        self.menu_callback = None
        self.parent_pipe,self.child_pipe = multiprocessing.Pipe()
        self.close_event = multiprocessing.Event()
        self.close_event.clear()
        self.child = multiprocessing.Process(target=self.child_task)
        self.child.start()

    def child_task(self):
        '''child process - this holds all the GUI elements'''
        import wx
        
        app = wx.PySimpleApp()
        app.frame = PreFlightFrame(state=self, title=self.title, no_timer=self.no_timer)
        app.frame.Show()
        app.MainLoop()

    def close(self):
        '''close the UI'''
        self.close_event.set()
        if self.is_alive():
            self.child.join(2)

    def is_alive(self):
        '''check if child is still going'''
        return self.child.is_alive()
        
    def send_value(self, name, value):
        '''send a name-value pair to child'''
        if self.child.is_alive():
            self.parent_pipe.send(NameValue(name, value))
        
    def recv_value(self):
        '''attempt to receive a name-value pair from child'''
        if self.child.is_alive():
            if self.parent_pipe.poll():
                nv = self.parent_pipe.recv()
                if isinstance(nv, NameValue):
                    return (nv.name, nv.value)
        return None
       
        
class PreFlightFrame(wx.Frame):
    """ The main frame of the console"""

    # Fixed values for the UI
    RED = (255,0,0)
    GREEN = (0,200,0)
    SMALL = 5
    LARGE = 15
    PLACEHOLDER_TEXT = "xxxxxx"
    FULL_SIZE = (750, 600)
    HALF_SIZE = (FULL_SIZE[0]/2, FULL_SIZE[1]-80)

    # Degrees that aircraft must be rotated about axes to test IMU
    MIN_DEGREES = 30

    # Parameters with expected (fixed) values
    # Note: some are ACS-specific and won't be in master branch,
    #  this will result in endless re-requests unless commented out
    PARAMS_FIXED = {
        'ACS_WATCH_HB' : 1,
        'AHRS_EKF_USE' : 1,
        'AHRS_ORIENTATION' : 0,
        'ALT_MIX' : 1,
        'ARMING_CHECK' : 1,
        'ARMING_DIS_RUD' : 1,
        'ARMING_REQUIRE' : 2,
        'ARSPD_FBW_MAX' : 25,
        'ARSPD_FBW_MIN' : 15,
        'COMPASS_EXTERNAL' : 1,
        'COMPASS_LEARN' : 1,
        'COMPASS_ORIENT' : 0,
        'FENCE_ACTION' : 1,
        'FENCE_AUTOENABLE' : 1,
        'FENCE_CHANNEL' : 0,
        'FENCE_RET_RALLY' : 1,
        'FS_BATT_VOLTAGE' : 10.475,
        'FS_GCS_ENABL' : 2,
        'FS_LONG_ACTN' : 1,
        'FS_LONG_TIMEOUT' : 20,
        'FS_SHORT_ACTN' : 1,
        'FS_SHORT_TIMEOUT' : 5,
        'GPS_TYPE2' : 0,
        'LAND_BREAK_PATH' : 0,
        'LAND_FLARE_ALT' : 15,
        'LAND_FLARE_SEC' : 4,
        'LAND_MAX_TURNS' : 1,
        'LAND_PITCH_CD' : -50,
        'LAND_WING_LEVEL' : 0,
        'LOG_BITMASK' : 65535,
        'RALLY_LIMIT_KM' : 3,
        'SKIP_GYRO_CAL' : 0,
        'STICK_MIXING' : 0,
        'TECS_LAND_ARSPD' : 16,
        'TECS_LAND_SPDWGT' : 1,
        'TECS_LAND_THR' : 30,
        'TERRAIN_ENABLE' : 0,
        'TERRAIN_FOLLOW' : 0,
        'THROTTLE_NUDGE' : 0,
        'THR_FAILSAFE' : 0,
        'THR_MAX' : 100,
        'THR_MIN' : 0,
        'THR_PASS_STAB' : 0,
        'TKOFF_THR_DELAY' : 9,
        'TKOFF_THR_MINACC' : 20,
        'TKOFF_THR_MINSPD' : 5,
        'TRIM_AUTO' : 0,
        'TRIM_ARSPD_CM' : 1800,
        'TRIM_THROTTLE' : 45
    }

    # Parameters with minimum values
    PARAMS_MIN = {
        'FENCE_TOTAL' : 4,
        'MIS_TOTAL' : 2,
        'RALLY_TOTAL' : 1
    }

    def __init__(self, state, title, no_timer=False):
        self.state = state
        wx.Frame.__init__(self, None, title=title, size=PreFlightFrame.FULL_SIZE,
                          style=wx.DEFAULT_FRAME_STYLE ^ wx.RESIZE_BORDER)

        # Place to store parameter widgets used in GUI
        self.params_wgt = {}

        # Variables to handle requests of parameters
        self.params_waiting = {}  # Requested params (false = received)
        self.params_lastseen = time.time()  # Last time we saw any param

        #use tabs for the individual checklists
        self.panel = wx.Panel(self)
        self.nb = wx.Choicebook(self.panel, wx.ID_ANY)
        self.createWidgets()

        #add in the pipe from MAVProxy and timed event handling
        self.timer = None
        if not no_timer:
            self.timer = wx.Timer(self)
            self.Bind(wx.EVT_TIMER, lambda evt,
                      notebook=self.nb: self.on_timer(evt, notebook), self.timer)
            self.timer.Start(20)

        # make sure MAVProxy instance will exit when commanded
        #self.state.child_pipe.send(NameValue("cmd", "set requireexit True"))
        
        # finally, put the notebook in a sizer for the panel to manage
        # the layout
        sizer = wx.BoxSizer()
        sizer.Add(self.nb, 1, wx.EXPAND)
        self.panel.SetSizer(sizer)
  
        self.Show(True)

        # Ask autopilot to provide all its parameters
        self.state.child_pipe.send(NameValue("param_all", ''))

    ### Widget creation helper functions ###

    def createPanelBox(self, panelName):
        '''create a panel and box object'''
        pnl = wx.Panel(self.nb)
        box = wx.BoxSizer(wx.VERTICAL)
        pnl.SetAutoLayout(True)
        pnl.SetSizer(box)
        pnl.Layout()
        self.nb.AddPage(pnl, panelName)
        return (pnl, box)

    def createDynamicLabel(self, panel, box, staticText,
                           dynamicText=PLACEHOLDER_TEXT,
                           dynamicColour=RED):
        '''create a two-part label, returning reference to second part'''
        subbox = wx.BoxSizer(wx.HORIZONTAL)
        if staticText != '':
            s_wgt = wx.StaticText(panel, wx.ID_ANY, "%s: " % staticText)
            subbox.Add(s_wgt)
        d_wgt = wx.StaticText(panel, wx.ID_ANY, dynamicText)
        d_wgt.SetForegroundColour(dynamicColour)
        subbox.Add(d_wgt)
        box.Add(subbox)
        return d_wgt

    def createParamLabel(self, panel, box, param):
        '''create a two-part label for a parameter'''
        wgt = self.createDynamicLabel(panel, box, param)
        self.params_wgt[param] = wgt
        self.params_waiting[param] = True

    def createTextboxLabel(self, panel, box, labelText, tboxEvent=None, tboxText=''):
        '''create a textbox + label, returning reference to textbox'''
        subbox = wx.BoxSizer(wx.HORIZONTAL)
        t_wgt = wx.TextCtrl(panel, wx.ID_ANY)
        t_wgt.SetValue(tboxText)
        if tboxEvent is not None:
            self.Bind(wx.EVT_TEXT, tboxEvent, t_wgt)
        subbox.Add(t_wgt)
        subbox.AddSpacer(PreFlightFrame.SMALL)
        l_wgt = wx.StaticText(panel, wx.ID_ANY, labelText)
        subbox.Add(l_wgt)
        box.Add(subbox)
        return t_wgt

    def createButtonRow(self, panel, box, buttonTupleList):
        '''create a row of buttons, specified by [("text", event_func), ...]'''
        subbox = wx.BoxSizer(wx.HORIZONTAL)
        for txt, evt in buttonTupleList:
            wgt = wx.Button(panel, wx.ID_ANY, txt)
            self.Bind(wx.EVT_BUTTON, evt, wgt)
            subbox.Add(wgt)
            subbox.AddSpacer(PreFlightFrame.LARGE)
        box.Add(subbox)
        box.AddSpacer(PreFlightFrame.LARGE)

    ### Main UI build-out ###

    def createWidgets(self):
        '''create controls on form - labels, buttons, etc'''

        # Parameter verification panel

        pnl, box = self.createPanelBox("Parameter Verification")

        box_l = wx.BoxSizer(wx.VERTICAL)
        box_l.SetMinSize(wx.Size(*PreFlightFrame.HALF_SIZE))
        box_r = wx.BoxSizer(wx.VERTICAL)
        box_r.SetMinSize(wx.Size(*PreFlightFrame.HALF_SIZE))
        param_count_half = math.ceil(len(PreFlightFrame.PARAMS_FIXED) / 2.0)
        for p in sorted(PreFlightFrame.PARAMS_FIXED):
            if param_count_half > 0:
                self.createParamLabel(pnl, box_l, p)
                param_count_half -= 1
            else:
                self.createParamLabel(pnl, box_r, p)
        subbox = wx.BoxSizer(wx.HORIZONTAL)
        subbox.Add(box_l)
        subbox.Add(box_r)
        box.Add(subbox)
        box.AddSpacer(PreFlightFrame.LARGE)

        self.createButtonRow(pnl, box,
                             [("Proceed to Mission Config", self.btn_NextPage),
                              ("Refresh Data", self.btn_Refresh)])

        # Mission configuration panel

        pnl, box = self.createPanelBox("Mission Configuration")

        box_l = wx.BoxSizer(wx.VERTICAL)
        box_l.SetMinSize(wx.Size(*PreFlightFrame.HALF_SIZE))
        box_r = wx.BoxSizer(wx.VERTICAL)
        box_r.SetMinSize(wx.Size(*PreFlightFrame.HALF_SIZE))

        wgt = wx.StaticText(pnl, wx.ID_ANY, "Confirm and deconflict waypoints, rally, and fence")
        box_l.Add(wgt)
        box_l.AddSpacer(PreFlightFrame.SMALL)

        self.createButtonRow(pnl, box_l,
                             [("Load Map and Editor", self.btn_LoadMap),
                              ("Unload", self.btn_UnloadMap)])

        wgt = wx.StaticText(pnl, wx.ID_ANY, "Check max terrain altitude < parameters (m AGL)")
        box_l.Add(wgt)
        box_l.AddSpacer(PreFlightFrame.SMALL)

        self.txt_TerrainAlt = self.createTextboxLabel(pnl, box_l,
                                  "Max terrain altitude (m AGL)",
                                  self.txt_TerrainAlt_Change)
        box_l.AddSpacer(PreFlightFrame.SMALL)
        self.createParamLabel(pnl, box_l, 'ALT_HOLD_RTL')
        self.createParamLabel(pnl, box_l, 'FENCE_RETALT')
        self.lbl_rally_alt = self.createDynamicLabel(pnl, box_l, "Rally Alt")
        self.lbl_rally_break_alt = self.createDynamicLabel(pnl, box_l, "Rally Break Alt")
        box_l.AddSpacer(PreFlightFrame.LARGE)

        wgt = wx.StaticText(pnl, wx.ID_ANY, "Check max operating altitude == parameters (m AGL)")
        box_l.Add(wgt)
        box_l.AddSpacer(PreFlightFrame.SMALL)

        self.txt_FenceCeiling = self.createTextboxLabel(pnl, box_l,
                                    "Max operating altitude (m AGL)",
                                    self.txt_FenceCeiling_Change)
        box_l.AddSpacer(PreFlightFrame.SMALL)
        self.createParamLabel(pnl, box_l, 'FENCE_MAXALT')
        box_l.AddSpacer(PreFlightFrame.LARGE)

        wgt = wx.StaticText(pnl, wx.ID_ANY, "Check fence minimum alt < takeoff waypoint alt")
        box_l.Add(wgt)
        box_l.AddSpacer(PreFlightFrame.SMALL)

        self.txt_FenceMin = self.createTextboxLabel(pnl, box_l,
                                "Fence minimum altitude (m AGL)",
                                self.txt_FenceMinTakeoff_Change)
        box_l.AddSpacer(PreFlightFrame.SMALL)
        self.txt_Takeoff = self.createTextboxLabel(pnl, box_l,
                               "Takeoff waypoint altitude (m AGL)",
                               self.txt_FenceMinTakeoff_Change)
        box_l.AddSpacer(PreFlightFrame.SMALL)
        self.createParamLabel(pnl, box_l, 'FENCE_MINALT')
        self.lbl_takeoff_alt = self.createDynamicLabel(pnl, box_l, "Takeoff Alt")
        box_l.AddSpacer(PreFlightFrame.LARGE)

        wgt = wx.StaticText(pnl, wx.ID_ANY, "Check minimum mission configuration loaded")
        box_r.Add(wgt)
        box_r.AddSpacer(PreFlightFrame.SMALL)

        self.lbl_total_wp = self.createParamLabel(pnl, box_r, 'MIS_TOTAL')
        self.lbl_total_fen = self.createParamLabel(pnl, box_r, 'FENCE_TOTAL')
        self.lbl_total_ral = self.createParamLabel(pnl, box_r, 'RALLY_TOTAL')
        box_r.AddSpacer(PreFlightFrame.LARGE)

        wgt = wx.StaticText(pnl, wx.ID_ANY, "Check battery capacity and failsafe (20% of cap)")
        box_r.Add(wgt)
        box_r.AddSpacer(PreFlightFrame.SMALL)

        self.txt_BattCapacity = self.createTextboxLabel(pnl, box_r,
                                    "Single battery capacity (mAh, for 2 batts)",
                                    self.txt_BattCapacity_Change)
        box_r.AddSpacer(PreFlightFrame.SMALL)
        self.createParamLabel(pnl, box_r, 'BATT_CAPACITY')
        self.createParamLabel(pnl, box_r, 'FS_BATT_MAH')
        box_r.AddSpacer(PreFlightFrame.LARGE)

        subbox = wx.BoxSizer(wx.HORIZONTAL)
        subbox.Add(box_l)
        subbox.Add(box_r)
        box.Add(subbox)
        box.AddSpacer(PreFlightFrame.LARGE)

        self.createButtonRow(pnl, box,
                             [("Proceed to Final Checks", self.btn_NextPage),
                              ("Refresh Data", self.btn_Refresh)])
        
        # Aircraft Final Run-Up panel

        pnl, box = self.createPanelBox("Final Checks")

        wgt = wx.StaticText(pnl, wx.ID_ANY, "Cover airspeed sensor with bottle and zeroize")
        box.Add(wgt)
        box.AddSpacer(PreFlightFrame.SMALL)

        subbox = wx.BoxSizer(wx.HORIZONTAL)
        self.lbl_zeroize = self.createDynamicLabel(pnl, subbox, "", "Not Done")
        subbox.AddSpacer(PreFlightFrame.LARGE)
        self.createButtonRow(pnl, subbox, [("Zeroize", self.btn_Zeroize)])
        box.Add(subbox)
        box.AddSpacer(PreFlightFrame.LARGE)

        wgt = wx.StaticText(pnl, wx.ID_ANY, "Check that -2 m/s < airspeed < 2 m/s when still, and that it registers positive airspeed")
        box.Add(wgt)
        box.AddSpacer(PreFlightFrame.SMALL)

        self.lbl_airspeed = self.createDynamicLabel(pnl, box, "Current Airspeed")
        box.AddSpacer(PreFlightFrame.LARGE)

        wgt = wx.StaticText(pnl, wx.ID_ANY, "Pitch and roll aircraft > %d degrees, checking that IMU registers" % PreFlightFrame.MIN_DEGREES)
        box.Add(wgt)
        box.AddSpacer(PreFlightFrame.SMALL)

        subbox = wx.BoxSizer(wx.HORIZONTAL)
        self.lbl_up = self.createDynamicLabel(pnl, subbox, "", "UP    ", PreFlightFrame.GREEN)
        self.lbl_down = self.createDynamicLabel(pnl, subbox, "", "DOWN    ", PreFlightFrame.GREEN)
        self.lbl_left = self.createDynamicLabel(pnl, subbox, "", "LEFT    ", PreFlightFrame.GREEN)
        self.lbl_right = self.createDynamicLabel(pnl, subbox, "", "RIGHT", PreFlightFrame.GREEN)
        box.Add(subbox)
        box.AddSpacer(PreFlightFrame.LARGE)

        wgt = wx.StaticText(pnl, wx.ID_ANY, "Arm and test throttle")
        box.Add(wgt)
        box.AddSpacer(PreFlightFrame.SMALL)

        subbox = wx.BoxSizer(wx.HORIZONTAL)
        self.lbl_armed = self.createDynamicLabel(pnl, subbox, "", "**UNKNOWN**")
        subbox.AddSpacer(PreFlightFrame.LARGE)
        self.createButtonRow(pnl, subbox,
                             [("ARM", self.btn_Arm),
                              ("DISARM", self.btn_Disarm)])
        box.Add(subbox)
        box.AddSpacer(PreFlightFrame.LARGE)

        wgt = wx.StaticText(pnl, wx.ID_ANY, "Check manual controls in MANUAL and FBW modes")
        box.Add(wgt)
        box.AddSpacer(PreFlightFrame.SMALL)

        self.lbl_mode = self.createDynamicLabel(pnl, box, "Current Mode")
        box.AddSpacer(PreFlightFrame.LARGE)

        wgt = wx.StaticText(pnl, wx.ID_ANY, "Check that relative altitude is within +/-10m")
        box.Add(wgt)
        box.AddSpacer(PreFlightFrame.SMALL)

        self.lbl_relalt = self.createDynamicLabel(pnl, box, "Relative Altitude")
        box.AddSpacer(PreFlightFrame.LARGE)

        wgt = wx.StaticText(pnl, wx.ID_ANY, "Check that GPS is locked and sees at least 6 satellites")
        box.Add(wgt)
        box.AddSpacer(PreFlightFrame.SMALL)

        self.lbl_gps = self.createDynamicLabel(pnl, box, "GPS Status")
        box.AddSpacer(PreFlightFrame.LARGE)

        self.createButtonRow(pnl, box,
                             [("Preflight Complete", self.btn_LastPage)])

    ### Textbox-change event handlers ###

    def txt_TerrainAlt_Change(self, event=None):
        '''want max terrain alt to be *less* than these settings'''
        c_rtl = PreFlightFrame.RED
        c_fen = PreFlightFrame.RED
        c_ral = PreFlightFrame.RED
        c_brk = PreFlightFrame.RED

        try:
            alt = float(self.txt_TerrainAlt.GetValue())
            if alt < float(self.params_wgt['ALT_HOLD_RTL'].GetLabelText()):
                c_rtl = PreFlightFrame.GREEN 
            if alt < float(self.params_wgt['FENCE_RETALT'].GetLabelText()):
                c_fen = PreFlightFrame.GREEN 
            if alt < float(self.lbl_rally_alt.GetLabelText()):
                c_ral = PreFlightFrame.GREEN 
            if alt < float(self.lbl_rally_break_alt.GetLabelText()):
                c_brk = PreFlightFrame.GREEN 
        except Exception as ex:
            pass

        self.params_wgt['ALT_HOLD_RTL'].SetForegroundColour(c_rtl)
        self.params_wgt['FENCE_RETALT'].SetForegroundColour(c_fen)
        self.lbl_rally_alt.SetForegroundColour(c_ral)
        self.lbl_rally_break_alt.SetForegroundColour(c_brk)
        
    def txt_FenceCeiling_Change(self, event=None):
        '''want max operating alt to *equal* fence ceiling'''
        c_fen = PreFlightFrame.RED

        try:
            alt = float(self.txt_FenceCeiling.GetValue())
            if alt == float(self.params_wgt['FENCE_MAXALT'].GetLabelText()):
                c_fen = PreFlightFrame.GREEN
        except:
            pass

        self.params_wgt['FENCE_MAXALT'].SetForegroundColour(c_fen)

    def txt_FenceMinTakeoff_Change(self, event=None):
        '''want FENCE_MINALT < Takeoff Alt, and confirm against textboxes'''
        clr = PreFlightFrame.RED
        try:
            to_alt = float(self.txt_Takeoff.GetValue())
            fm_alt = float(self.txt_FenceMin.GetValue())
            if float(self.lbl_takeoff_alt.GetLabelText()) \
             > float(self.params_wgt['FENCE_MINALT'].GetLabelText()) and \
             to_alt == float(self.lbl_takeoff_alt.GetLabelText()) and \
             fm_alt == float(self.params_wgt['FENCE_MINALT'].GetLabelText()):
                clr = PreFlightFrame.GREEN
        except:
            pass
        self.lbl_takeoff_alt.SetForegroundColour(clr)
        self.params_wgt['FENCE_MINALT'].SetForegroundColour(clr)

    def txt_BattCapacity_Change(self, event=None):
        '''want to confirm battery/failsafe against textbox'''
        c_cap = PreFlightFrame.RED
        c_fs = PreFlightFrame.RED

        try:
            cap = float(self.txt_BattCapacity.GetValue()) * 2.0
            if cap == float(self.params_wgt['BATT_CAPACITY'].GetLabelText()):
                c_cap = PreFlightFrame.GREEN
            if (cap * 0.2) == float(self.params_wgt['FS_BATT_MAH'].GetLabelText()):
                c_fs = PreFlightFrame.GREEN
        except:
            pass

        self.params_wgt['BATT_CAPACITY'].SetForegroundColour(c_cap)
        self.params_wgt['FS_BATT_MAH'].SetForegroundColour(c_fs)

    ### Button-click event handlers ###

    def btn_LoadMap(self, event):
        '''load map and mission editor, refresh view'''
        self.state.child_pipe.send(NameValue("cmd", "module load map"))
        self.state.child_pipe.send(NameValue("cmd", "module load misseditor"))
        self.state.child_pipe.send(NameValue("cmd", "wp list"))
        self.state.child_pipe.send(NameValue("cmd", "rally list"))
        self.state.child_pipe.send(NameValue("cmd", "fence list"))
        
    def btn_UnloadMap(self, event):
        '''unload map and mission editor'''
        self.state.child_pipe.send(NameValue("cmd", "module unload map"))
        self.state.child_pipe.send(NameValue("cmd", "module unload misseditor"))

    def btn_Zeroize(self, event):
        '''zeroize airspeed sensor'''
        self.lbl_zeroize.SetLabel("Waiting")
        self.lbl_zeroize.SetForegroundColour(PreFlightFrame.RED)
        self.state.child_pipe.send(NameValue("cmd", "calpress"))

    def btn_Arm(self, event):
        '''arm throttle'''
        self.state.child_pipe.send(NameValue("cmd", "arm throttle"))

    def btn_Disarm(self, event):
        '''disarm throttle'''
        self.state.child_pipe.send(NameValue("cmd", "disarm"))

    #do a final check of the current panel and move to the next
    def btn_NextPage(self, event):
        '''go to next panel (page)'''
        win = (event.GetEventObject()).GetParent()
        win.GetParent().AdvanceSelection()

    #Special implementation of the above function, but for the last tab
    def btn_LastPage(self, event):
        '''on last panel (page), close out module'''
        # Tear down this window
        if self.timer is not None:
            self.timer.Stop()
        self.Destroy()

        # Unload this module (twice to be sure)
        self.state.child_pipe.send(NameValue("cmd", "module unload preflight"))
        self.state.child_pipe.send(NameValue("cmd", "module unload preflight"))

        # If 'requireexit' is used and this is a secondary (flight tech)
        #   MAVProxy instance, can close out entirely
        # TODO: Decide how to mark aircraft as 'ready'
        #self.state.child_pipe.send(NameValue("cmd", "exit"))
        #self.state.child_pipe.send(NameValue("cmd", "exit"))

    def btn_Refresh(self, event):
        '''Reload all params, etc (useful when multiple MAVProxies used)'''
        # Reset the re-request timer
        self.params_lastseen = time.time()

        # Note that we're now waiting for all params
        self.params_waiting = {k:True for k in self.params_waiting}

        # Reset visible state of all widgets
        for k in self.params_wgt:
            self.params_wgt[k].SetLabel(PreFlightFrame.PLACEHOLDER_TEXT)
            self.params_wgt[k].SetForegroundColour(PreFlightFrame.RED)
        self.lbl_rally_alt.SetLabel(PreFlightFrame.PLACEHOLDER_TEXT)
        self.lbl_rally_alt.SetForegroundColour(PreFlightFrame.RED)
        self.lbl_rally_break_alt.SetLabel(PreFlightFrame.PLACEHOLDER_TEXT)
        self.lbl_rally_break_alt.SetForegroundColour(PreFlightFrame.RED)
        self.lbl_takeoff_alt.SetLabel(PreFlightFrame.PLACEHOLDER_TEXT)
        self.lbl_takeoff_alt.SetForegroundColour(PreFlightFrame.RED)

        # Finally, request all data
        self.state.child_pipe.send(NameValue("param_all", ""))
        self.state.child_pipe.send(NameValue("cmd", "rally list"))
        self.state.child_pipe.send(NameValue("cmd", "wp list"))

    ### Periodic event handling ###

    def on_timer(self, event, notebook):
        '''handle feedback from MAVProxy side of the module'''
        state = self.state
        win = notebook.GetPage(notebook.GetSelection()) 
        if state.close_event.wait(0.001):
            self.timer.Stop()
            self.Destroy()
            return

        # Re-request any missing params
        if time.time() > self.params_lastseen + 2.0:
            for p in [p for p in self.params_waiting if self.params_waiting[p]]:
                print "preflight: re-requesting param " + p
                self.state.child_pipe.send(NameValue("param", p))

            # Also check if rally points are missing
            if self.lbl_rally_alt.GetLabelText() == PreFlightFrame.PLACEHOLDER_TEXT:
                print "preflight: re-requesting rally point(s)"
                self.state.child_pipe.send(NameValue("cmd", "rally list"))

            # Also check if waypoints are missing
            if self.lbl_takeoff_alt.GetLabelText() == PreFlightFrame.PLACEHOLDER_TEXT:
                print "preflight: re-requesting waypoint(s)"
                self.state.child_pipe.send(NameValue("cmd", "wp list"))

            # Delay before another re-request
            self.params_lastseen = time.time()

        # Process any queued messages (see mavproxy_preflight.py)
        while state.child_pipe.poll():
            obj = state.child_pipe.recv()
            if not isinstance(obj, NameValue):
                continue

            if obj.name == "param":
                # Params still arriving, delay re-requesting
                self.params_lastseen = time.time()
                param, value = obj.value
                param = str(param)
                value = "%0.03f" % value
                if param not in self.params_wgt:
                    return
                if param not in PreFlightFrame.PARAMS_FIXED:
                    # params with no expected value
                    self.params_wgt[param].SetLabel(value)
                    # handle special cases
                    if param in PreFlightFrame.PARAMS_MIN:
                        if float(value) >= PreFlightFrame.PARAMS_MIN[param]:
                            self.params_wgt[param].SetForegroundColour(PreFlightFrame.GREEN)
                        else:
                            self.params_wgt[param].SetForegroundColour(PreFlightFrame.RED)
                    elif param == 'FENCE_MINALT':
                        self.txt_FenceMinTakeoff_Change()
                    elif param in ['FENCE_RETALT', 'ALT_HOLD_RTL']:
                        self.txt_TerrainAlt_Change()
                    elif param == 'FENCE_MAXALT':
                        self.txt_FenceCeiling_Change()
                    elif param in ['BATT_CAPACITY', 'FS_BATT_MAH']:
                        self.txt_BattCapacity_Change()
                elif float(value) == float(PreFlightFrame.PARAMS_FIXED[param]):
                    # param matched expected value
                    self.params_wgt[param].SetLabel(value)
                    self.params_wgt[param].SetForegroundColour(PreFlightFrame.GREEN)
                else:
                    # param does NOT match expected value
                    self.params_wgt[param].SetLabel("%s (expected %0.03f)" % \
                        (value, PreFlightFrame.PARAMS_FIXED[param]))
                    self.params_wgt[param].SetForegroundColour(PreFlightFrame.RED)
                # No longer waiting for this parameter value
                self.params_waiting[param] = False

            elif obj.name == "rally":
                alt, break_alt = obj.value
                self.lbl_rally_alt.SetLabel("%0.03f" % alt)
                self.lbl_rally_break_alt.SetLabel("%0.03f" % break_alt)
                # Trigger recalculation of textbox value
                self.txt_TerrainAlt_Change(None)

            elif obj.name == "takeoff":
                self.lbl_takeoff_alt.SetLabel("%0.03f" % obj.value)
                self.txt_FenceMinTakeoff_Change()

            elif obj.name == "zeroize":
                self.lbl_zeroize.SetLabel("Done")
                self.lbl_zeroize.SetForegroundColour(PreFlightFrame.GREEN)

            elif obj.name == "airspeed":
                self.lbl_airspeed.SetLabel(str(int(obj.value)))
                if -2.0 < obj.value < 2.0:
                    self.lbl_airspeed.SetForegroundColour(PreFlightFrame.GREEN)
                else:
                    self.lbl_airspeed.SetForegroundColour(PreFlightFrame.RED)

            elif obj.name == "pitch":
                deg = math.degrees(obj.value)
                if deg < -1.0 * PreFlightFrame.MIN_DEGREES:
                    self.lbl_up.SetForegroundColour(PreFlightFrame.GREEN)
                    self.lbl_down.SetForegroundColour(PreFlightFrame.RED)
                elif deg > PreFlightFrame.MIN_DEGREES:
                    self.lbl_up.SetForegroundColour(PreFlightFrame.RED)
                    self.lbl_down.SetForegroundColour(PreFlightFrame.GREEN)
                else:
                    self.lbl_up.SetForegroundColour(PreFlightFrame.GREEN)
                    self.lbl_down.SetForegroundColour(PreFlightFrame.GREEN)

            elif obj.name == "roll":
                deg = math.degrees(obj.value)
                if deg < -1.0 * PreFlightFrame.MIN_DEGREES:
                    self.lbl_left.SetForegroundColour(PreFlightFrame.RED)
                    self.lbl_right.SetForegroundColour(PreFlightFrame.GREEN)
                elif deg > PreFlightFrame.MIN_DEGREES:
                    self.lbl_left.SetForegroundColour(PreFlightFrame.GREEN)
                    self.lbl_right.SetForegroundColour(PreFlightFrame.RED)
                else:
                    self.lbl_left.SetForegroundColour(PreFlightFrame.GREEN)
                    self.lbl_right.SetForegroundColour(PreFlightFrame.GREEN)

            elif obj.name == "armed":
                if obj.value:
                    self.lbl_armed.SetLabel("ARMED")
                    self.lbl_armed.SetForegroundColour(PreFlightFrame.RED)
                else:
                    self.lbl_armed.SetLabel("disarmed")
                    self.lbl_armed.SetForegroundColour(PreFlightFrame.GREEN)

            elif obj.name == "mode":
                self.lbl_mode.SetLabel(obj.value)
                if obj.value in ["MANUAL", "FBWA", "FBWB"]:
                    self.lbl_mode.SetForegroundColour(PreFlightFrame.GREEN)
                else:
                    self.lbl_mode.SetForegroundColour(PreFlightFrame.RED)

            elif obj.name == "relalt":
                self.lbl_relalt.SetLabel(str(obj.value))
                if -10.0 < obj.value < 10.0:
                    self.lbl_relalt.SetForegroundColour(PreFlightFrame.GREEN)
                else:
                    self.lbl_relalt.SetForegroundColour(PreFlightFrame.RED)

            elif obj.name == "gps":
                locked, visible = obj.value
                self.lbl_gps.SetLabel("%s / %u Sats" % (locked, visible))
                if locked == 'LOCKED' and visible >= 6:
                    self.lbl_gps.SetForegroundColour(PreFlightFrame.GREEN)
                else:
                    self.lbl_gps.SetForegroundColour(PreFlightFrame.RED)

if __name__ == "__main__":
    # test the console
    import time

    preflight = PreFlightUI(no_timer=True)

    # dummy loop
    while preflight.is_alive():
        time.sleep(0.5)
        
        
