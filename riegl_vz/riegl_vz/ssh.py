"""Client to handle connections and actions executed against a remote host."""
from typing import List

from paramiko import AutoAddPolicy, SSHClient
from scp import SCPClient

class RemoteClient:
    """Client to interact with a remote host via SSH & SCP."""

    def __init__(
        self,
        host: str,
        user: str,
        password: str
    ):
        self.host = host
        self.user = user
        self.password = password
        self.client = None

    @property
    def connection(self):
        """Open SSH connection to remote host."""
        if self.client:
            return self.client
        try:
            self.client = SSHClient()
            self.client.load_system_host_keys()
            self.client.set_missing_host_key_policy(AutoAddPolicy())
            self.client.connect(
                self.host,
                username=self.user,
                password=self.password,
                timeout=5000
            )
            return self.client
        except Exception as e:
            raise e

    @property
    def scp(self) -> SCPClient:
        return SCPClient(self.connection.get_transport())

    def disconnect(self):
        """Close SSH connection."""
        if self.client:
            self.client.close()

    def uploadFile(self, localpath: str, remotepath: str):
        """
        Upload file to a remote directory.
        """
        try:
            scp = self.scp
            scp.put(localpath, remotepath)
            scp.close()
        except Exception as e:
            raise e

    def downloadFile(self, filepath: str, localpath: str = "./"):
        """Download file from remote host."""
        scp = self.scp
        scp.get(filepath, localpath)
        scp.close()

    def executeCommand(self, command: str):
        """
        Execute a single command.

        :param command: unix command as strings.
        :type command: str
        """
        stdin, stdout, stderr = self.connection.exec_command(command)
        stdout.channel.recv_exit_status()
        return stdout.readlines()

    def executeCommands(self, commands: List[str]):
        """
        Execute multiple commands in succession.

        :param commands: List of unix commands as strings.
        :type commands: List[str]
        """
        for cmd in commands:
            stdin, stdout, stderr = self.connection.exec_command(cmd)
            stdout.channel.recv_exit_status()
            response = stdout.readlines()

class RieglVzSSH:
    def __init__(self, node):
        self._node = node
        self._hostname = node.hostname
        self._sshUser = node.sshUser
        self._sshPwd = node.sshPwd
        self._logger = node.get_logger()

    def downloadFile(self, remoteFile: str, localFile: str):
        self._logger.debug("Downloading file..")
        self._logger.debug("remote file = {}".format(remoteFile))
        self._logger.debug("local file  = {}".format(localFile))
        ssh = RemoteClient(host=self._hostname, user=self._sshUser, password=self._sshPwd)
        ssh.downloadFile(filepath=remoteFile, localpath=localFile)
        ssh.disconnect()
        self._logger.debug("File download finished")

    def uploadFile(self, localFile: str, remoteDir: str):
        self._logger.debug("Uploading file..")
        self._logger.debug("local file = {}".format(localFile))
        self._logger.debug("remote dir = {}".format(remoteDir))
        ssh = RemoteClient(host=self._hostname, user=self._sshUser, password=self._sshPwd)
        ssh.uploadFile(localpath=localFile, remotepath=remoteDir)
        ssh.disconnect()
        self._logger.debug("File upload finished")

    def executeCommand(self, cmd):
        self._logger.debug("CMD = {}".format(cmd))
        ssh = RemoteClient(host=self._hostname, user=self._sshUser, password=self._sshPwd)
        response = ssh.executeCommand(cmd)
        ssh.disconnect()
        self._logger.debug("RESP = {}".format(" ".join(response)))
        return response

    def listFiles(self, remotePath, grepFilter, fullPath = None):
        fullPathOpt = ""
        if fullPath is None:
            fullPathOpt = "-d "
        cmd = "ls -1 " + fullPathOpt + " " + remotePath, "/* " + grepFilter
        return self.executeCommand(cmd)
