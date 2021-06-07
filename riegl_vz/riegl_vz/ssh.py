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
            LOGGER.error("Connect!")
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

    def bulk_upload(self, files: List[str], remote_path: str):
        """
        Upload multiple files to a remote directory.

        :param files: List of local files to be uploaded.
        :type files: List[str]
        """
        try:
            scp = self.scp
            scp.put(files, remote_path=remote_path, recursive=True).close()
            scp.close()
        except Exception as e:
            raise e

    def download_file(self, file: str):
        """Download file from remote host."""
        scp = self.scp
        scp.get(file)
        scp.close()

    def execute_commands(self, commands: List[str]):
        """
        Execute multiple commands in succession.

        :param commands: List of unix commands as strings.
        :type commands: List[str]
        """
        for cmd in commands:
            stdin, stdout, stderr = self.connection.exec_command(cmd)
            stdout.channel.recv_exit_status()
            response = stdout.readlines()
