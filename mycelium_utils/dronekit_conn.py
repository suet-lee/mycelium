#!/usr/bin/env python3

import time
import dronekit

class DronekitConnector:

    def __init__(self, 
        connection_string,
        connection_baudrate=921600,
        timeout=30,
        source_system=1,
        source_component=0):

        self.conn = dronekit.connect(ip=connection_string,
            baud=connection_baudrate,
            timeout=timeout, 
            source_system=source_system,
            source_component=source_component)
        self.mission = None

    def arm(self, timeout=10):
        i = 0
        while not self.conn.is_armable and i < timeout:
            i += 1
            time.sleep(1)
        
        self.set_mode('GUIDED')
        self.conn.arm()

    def disarm(self, timeout=10):
        self.conn.disarm(timeout=timeout)

    def set_mode(self, mode):
        self.conn.mode = dronekit.VehicleMode(mode)
        time.sleep(1)
        return self.conn.mode.name

    def get_mode(self):
        return self.conn.mode.name

    def reboot(self):
        self.reboot()
        # check if rebooted

    def get_mission(self, update=False):            
        if update:
            self.mission = self.conn.commands
            self.mission.download()
            self.mission.wait_ready()

        return self.mission

    def fetch_mission(self):
        return self.get_mission(update=True)

    def send_to_waypoint(self, waypoint):
        if self.mission is None:
            self.fetch_mission()

        # check if waypoint is valid

        self.mission.next = waypoint

    def get_gps(self):
        gps = self.conn.location.global_frame
        if gps:
            return [gps.lat, gps.lon]
        return [None, None]

    def upload_mission(self, waypoints_file):
        """
        Upload a mission from a file.
        """
        #Read mission from file
        missionlist = self._readmission(waypoints_file)

        #Clear existing mission from vehicle
        cmds = self.conn.commands
        cmds.clear()
        #Add new mission to vehicle
        for command in missionlist:
            cmds.add(command)
            
        self.conn.commands.upload()

    def _readmission(self, file_name):
        """
        Load a mission from a file into a list.

        This function is used by upload_mission().
        """
        cmds = self.conn.commands
        missionlist=[]
        with open(file_name) as f:
            for i, line in enumerate(f):
                if i==0:
                    if not line.startswith('QGC WPL 110'):
                        raise Exception('File is not supported WP version')
                else:
                    linearray=line.split('\t')
                    ln_index=int(linearray[0])
                    ln_currentwp=int(linearray[1])
                    ln_frame=int(linearray[2])
                    ln_command=int(linearray[3])
                    ln_param1=float(linearray[4])
                    ln_param2=float(linearray[5])
                    ln_param3=float(linearray[6])
                    ln_param4=float(linearray[7])
                    ln_param5=float(linearray[8])
                    ln_param6=float(linearray[9])
                    ln_param7=float(linearray[10])
                    ln_autocontinue=int(linearray[11].strip())
                    cmd = dronekit.Command( 0, 0, 0, ln_frame, ln_command, ln_currentwp, ln_autocontinue, ln_param1, ln_param2, ln_param3, ln_param4, ln_param5, ln_param6, ln_param7)
                    missionlist.append(cmd)
        return missionlist

    def disconnect(self):
        self.conn.close()

