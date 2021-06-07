from .ssh import RemoteClient

def main(args=None):
    host = 'H2222273'
    user = 'user'
    password = 'user'
    remote_path = '/media/intern/'
    cli = RemoteClient(host=host, user=user, password=password)
    cli.download_file(file="/media/intern/gsm_kolomela.gsfx")
    cli.disconnect()

if __name__ == "__main__":
    main()

