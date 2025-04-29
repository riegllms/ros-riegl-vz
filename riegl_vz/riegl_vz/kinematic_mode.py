from .ssh import RieglVzSSH
import yaml

class NavigationFrame():
    def __init__(self):
        self.roll = 180.0
        self.pitch = 0.0
        self.yaw = 0.0

class StaticScanParams():
    def __init__(self):
        self.scan_pattern = ""
        self.measurement_program = 3

class KinematicFixAngleParamss():
    def __init__(self):
        self.theta_start = 30.0
        self.theta_stop = 130.0
        self.theta_increment = 0.034
        self.phi = 90
        self.measurement_program = 3

class KinematicRotatingParams():
    def __init__(self):
        self.theta_start = 30.0
        self.theta_stop = 130.0
        self.theta_increment = 0.034
        self.rotation_speed = 150
        self.measurement_program = 3
        self.change_direction = False
        self.change_direction_interval = 30

class KinematicParams():
    def __init__(self):
        self.kinematic_fix_angle = KinematicFixAngleParamss()
        self.static_scan = StaticScanParams()
        self.navigation_frame = NavigationFrame()
        self.kinematic_rotating = KinematicRotatingParams()
        self.project = "MyProject001"
        self.gnss_configuration = "GNSS On Top"
        self.force_rtk_fix = False
        self.force_gnss_time = True
        self.gnss_post_processing = False

class Status():
    def __init__(self):
        self.params = KinematicParams()
        self.aktiv = ""
        self.recording = ""
        self.error = ""
        self.waiting_for_gnss_fix = False
        self.waiting_for_gnss_time = False
        self.waiting_for_rtk_fix = False


class KinematicMode():
    def __init__(self,node):
        self._node = node
        self._logger = node.get_logger()
        self._hostname = node.hostname
        self._workingDir = node.workingDir
        self._connectionString = self._hostname + ':20000'
        self._ssh: RieglVzSSH = RieglVzSSH(self._node)

    def _bool_to_str(self,data: bool):
        if data:
            return "true"
        return "false"

    def start_kinematic_mode(self,Parameter: KinematicParams):
        cmd = ("riappctl send-request com.riegl.vzi-apps.kinematicapp start '{\"force_rtk_fix\":"+self._bool_to_str(Parameter.force_rtk_fix)+
        ",\"gnss_configuration\":\""+ Parameter.gnss_configuration +
        "\",\"gnss_post_processing\":"+ self._bool_to_str(Parameter.gnss_post_processing)+
        ",\"kinematic\":{\"measurement_program\":"+str(Parameter.kinematic_fix_angle.measurement_program)+
            ",\"phi\":"+str(Parameter.kinematic_fix_angle.phi)+
            ",\"theta_increment\":"+str(Parameter.kinematic_fix_angle.theta_increment)+
            ",\"theta_start\":"+str(Parameter.kinematic_fix_angle.theta_start)+
            ",\"theta_stop\":"+str(Parameter.kinematic_fix_angle.theta_stop)+
        "},\"kinematic_rotating\":{\"change_direction\":"+self._bool_to_str(Parameter.kinematic_rotating.change_direction)+
            ",\"change_direction_interval\":"+str(Parameter.kinematic_rotating.change_direction_interval)+
            ",\"measurement_program\":"+str(Parameter.kinematic_rotating.measurement_program)+
            ",\"rotation_speed\":"+str(Parameter.kinematic_rotating.rotation_speed)+
            ",\"theta_increment\":"+str(Parameter.kinematic_rotating.theta_increment)+
            ",\"theta_start\":"+str(Parameter.kinematic_rotating.theta_start)+
            ",\"theta_stop\":"+str(Parameter.kinematic_rotating.theta_stop)+
        "},\"navigation_frame\":{\"pitch\":"+str(Parameter.navigation_frame.pitch)+
            ",\"roll\":"+str(Parameter.navigation_frame.roll)+
            ",\"yaw\":"+str(Parameter.navigation_frame.yaw)+
        "},\"project\":\""+Parameter.project+
        "\",\"static\":{\"measurement_program\":"+str(Parameter.static_scan.measurement_program)+
            ",\"scan_pattern\":\""+Parameter.static_scan.scan_pattern+
        "\"},\"force_gnss_time\":"+self._bool_to_str(Parameter.force_gnss_time)+"}'")
        self._node.get_logger().debug("send comand to Scaner: "+cmd)
        rc,return_msg, errors = self._ssh.executeCommand(cmd,True)
        self._node.get_logger().debug("RC: "+str(rc))
        if rc != 0:
            if "Method not found" in str(errors):
                error_msg = "Update kinematic app on scanner"
                self._node.get_logger().error(error_msg)
                return False, error_msg
            error_msg = "could not start kinematic mode"
            self._node.get_logger().error(error_msg)
            self._node.get_logger().info("return msg: "+str(return_msg))
            self._node.get_logger().warn("error msg: "+str(errors))
            return False, error_msg
        self._node.get_logger().debug("return msg: "+str(return_msg))
        return True, ""

    def start_recording(self,mode: str,rotating: bool):
        cmd="riappctl send-request com.riegl.vzi-apps.kinematicapp startRecording '{\"mode\": \""+mode+"\", \"rotating\": "+self._bool_to_str(rotating)+"}'"
        self._node.get_logger().debug("send comand to Scaner: "+cmd)
        rc,return_msg, errors = self._ssh.executeCommand(cmd,True)
        self._node.get_logger().debug("RC: "+str(rc))
        if rc != 0:
            error_msg = "could not start kinematic recording"
            self._node.get_logger().error(error_msg)
            self._node.get_logger().info("return msg: "+str(return_msg))
            self._node.get_logger().warn("error msg: "+str(errors))
            return False, error_msg
        self._node.get_logger().debug("return msg: "+str(return_msg))
        return True, ""

    def get_status(self):
        cmd = ("riappctl send-request com.riegl.vzi-apps.kinematicapp status")
        self._node.get_logger().debug("send comand to Scaner: "+cmd)
        rc,return_msg, errors = self._ssh.executeCommand(cmd,True)
        self._node.get_logger().debug("RC: "+str(rc))
        status=Status()
        if rc != 0:
            self._node.get_logger().error("could not get kinematic status")
            self._node.get_logger().info("return msg: "+str(return_msg))
            self._node.get_logger().warn("error msg: "+str(errors))
            return status
        self._node.get_logger().debug("return msg: "+str(return_msg))
        dct = yaml.safe_load(return_msg[0])
        status.aktiv = dct['active']
        status.recording = dct['recording']
        status.waiting_for_gnss_fix=dct['waiting_for_gnss_fix']
        status.waiting_for_gnss_time=dct['waiting_for_gnss_time']
        status.waiting_for_rtk_fix=dct['waiting_for_rtk_fix']
        status.params.project = dct['configuration']['project']
        status.params.force_gnss_time = dct['configuration']['force_gnss_time']
        status.params.force_rtk_fix = dct['configuration']['force_rtk_fix']
        status.params.gnss_configuration = dct['configuration']['gnss_configuration']
        status.params.gnss_post_processing = dct['configuration']['gnss_post_processing']
        status.params.kinematic_fix_angle.measurement_program = dct['configuration']['kinematic']['measurement_program']
        status.params.kinematic_fix_angle.phi = dct['configuration']['kinematic']['phi']
        status.params.kinematic_fix_angle.theta_increment = dct['configuration']['kinematic']['theta_increment']
        status.params.kinematic_fix_angle.theta_start = dct['configuration']['kinematic']['theta_start']
        status.params.kinematic_fix_angle.theta_stop = dct['configuration']['kinematic']['theta_stop']
        status.params.kinematic_rotating.measurement_program = dct['configuration']['kinematic_rotating']['measurement_program']
        status.params.kinematic_rotating.rotation_speed = dct['configuration']['kinematic_rotating']['rotation_speed']
        status.params.kinematic_rotating.theta_increment = dct['configuration']['kinematic_rotating']['theta_increment']
        status.params.kinematic_rotating.theta_start = dct['configuration']['kinematic_rotating']['theta_start']
        status.params.kinematic_rotating.theta_stop = dct['configuration']['kinematic_rotating']['theta_stop']
        status.params.kinematic_rotating.change_direction = dct['configuration']['kinematic_rotating']['change_direction']
        status.params.kinematic_rotating.change_direction_interval = dct['configuration']['kinematic_rotating']['change_direction_interval']
        status.params.navigation_frame.pitch = dct['configuration']['navigation_frame']['pitch']
        status.params.navigation_frame.roll = dct['configuration']['navigation_frame']['roll']
        status.params.navigation_frame.yaw = dct['configuration']['navigation_frame']['yaw']
        status.params.static_scan.measurement_program = dct['configuration']['static']['measurement_program']
        status.params.static_scan.scan_pattern = dct['configuration']['static']['scan_pattern']
        self._node.get_logger().debug("return msg: "+str(return_msg)+"\n")

        if 'error_message' in dct.keys():
            status.error = dct['error_message']
        else:
            status.error = ""
        return status

    def stop_kinematic_mode(self):
        cmd = ("riappctl send-request com.riegl.vzi-apps.kinematicapp stop")
        self._node.get_logger().debug("send comand to Scaner: "+cmd)
        rc,return_msg, errors = self._ssh.executeCommand(cmd,True)
        self._node.get_logger().debug("RC: "+str(rc))
        if rc != 0:
            error_msg = "could not stop kinematic mode"
            self._node.get_logger().error(error_msg)
            self._node.get_logger().info("return msg: "+str(return_msg))
            self._node.get_logger().warn("error msg: "+str(errors))
            return False, error_msg
        self._node.get_logger().debug("return msg: "+str(return_msg))
        return True, ""

    def stop_recording(self):
        cmd = ("riappctl send-request com.riegl.vzi-apps.kinematicapp stopRecording")
        self._node.get_logger().debug("send comand to Scaner: "+cmd)
        rc,return_msg, errors = self._ssh.executeCommand(cmd,True)
        self._node.get_logger().debug("RC: "+str(rc))
        self._node.get_logger().debug("return msg: "+str(return_msg))
        if rc != 0:
            error_msg="could not stop kinematic recording"
            self._node.get_logger().error(error_msg)
            self._node.get_logger().info("return msg: "+str(return_msg))
            self._node.get_logger().warn("error msg: "+str(errors))
            return False, error_msg
        self._node.get_logger().debug("return msg: "+str(return_msg))
        return True, ""
