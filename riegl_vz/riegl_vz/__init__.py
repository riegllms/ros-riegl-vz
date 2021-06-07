from .ssh import RemoteClient

import sys

from std_srvs.srv import Trigger

import rclpy
from rclpy.node import Node

from vzi_services.controlservice import ControlService

def scp():
    host = 'H2222273'
    user = 'user'
    password = 'user'
    remote_path = '/media/intern/'
    cli = RemoteClient(host=host, user=user, password=password)
    cli.download_file(file="/media/intern/gsm_kolomela.gsfx")
    cli.disconnect()

class ScanService(Node):

    def __init__(self, cs):
        super().__init__('scan')
        self.vzi_control_service = cs
        self.srv = self.create_service(Trigger, 'scan', self.scan_callback)

    def scan_callback(self, request, response):
        self.vzi_control_service = cs.StartAcquisition()
        response.success = True
        response.message = "SUCCESS"
        return response

def main(args=None):
    rclpy.init(args=args)

    cs = ControlService(sys.argv[1])
    scan_service = ScanService(cs)
    rclpy.spin(scan_service)

    rclpy.shutdown()

if __name__ == "__main__":
    main()

