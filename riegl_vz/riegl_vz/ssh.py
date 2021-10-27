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
