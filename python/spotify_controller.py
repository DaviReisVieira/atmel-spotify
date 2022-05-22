import pyautogui
import serial
import argparse
import time
import logging
import unidecode
from functions.auxiliar import hex2int
from functions.spotify import Spotify

class SerialControllerInterface:

    def __init__(self, port, baudrate):        
        print("Iniciando interface serial...")
        self.ser = serial.Serial(port, baudrate=baudrate)
        print("Interface serial iniciada com sucesso!")
        self.spotify_controller = Spotify()
        pyautogui.PAUSE = 0  ## remove delay
        self.current_volume = None
        self.handshake_complete = False


    def read_payload(self, data):
        header = data[:1]
        payload = data[1:5]
        eop = data[5:]
        if eop != b'X':
            self.handshake_complete = False
            return None, None, None, None
        payload_value = hex2int(payload)
        print("-----------------------------------------------------")
        print("Header: {}".format(header))
        print("Payload: {}".format(payload))
        print("EOP: {}".format(eop))
        print("Payload Value: {}".format(payload_value))
        print("-----------------------------------------------------")
        return header, payload, eop, payload_value

    def send_data(self, data):
        # print('enviando dados...')
        self.ser.write(data)


    def send_music_informations(self):
        # char is_playing
        # char is_shuffle
        # char repeat_mode
        # char volume
        # char music_name[32]
        # char artist[16]
        # uint32_t duration
        # char eop[3]
        
        is_playing = '1' if self.spotify_controller.getCurrentSongStatus() else '0'

        is_shuffle = '1' if self.spotify_controller.getShuffleStatus() else '0'

        repeat_mode = self.spotify_controller.getRepeatMode()

        volume = self.spotify_controller.getVolume()

        music_name = self.spotify_controller.getCurrentSongName()
        music_name = unidecode.unidecode(music_name)
        music_name = music_name.encode()
        music_name = music_name[:32]+b'~'*(32-len(music_name))

        artist = self.spotify_controller.getCurrentSongArtist()
        artist = unidecode.unidecode(artist)
        artist = artist.encode()
        artist = artist[:16]+b'~'*(16-len(artist))

        duration = self.spotify_controller.getCurrentSongDuration()
        duration = str(duration / 1000)
        duration = duration[:duration.find('.')]
        duration = '0' + duration if len(duration) != 3 else (duration)

        eop = 'ARMC'

        data = b'PPPP' + is_playing.encode('ascii') + is_shuffle.encode('ascii') + repeat_mode.encode('ascii') + volume.encode('ascii') + music_name + artist + duration.encode('ascii') + eop.encode('ascii')
        print("\nEnviando: {}\nTamanho: {}\n".format(data,len(data)))
        self.send_data(data)


    def calculate_volume(self, volume):
        volume_max = 4088
        volume_min = 20
        volume_range = volume_max - volume_min
        volume_percentage = (volume - volume_min) / volume_range
        volume_percentage = round(volume_percentage * 100)
        return volume_percentage


    def select_action(self, data):
        header, payload, eop, payload_value = self.read_payload(data)

        if eop != b'X':
            print('PAYLOAD ERRADO MANDAR NOVAMENTE')

        if header == b'P':
            print('Play/Pause Song Command')
            self.spotify_controller.playPauseSong()

        if header == b'N':
            print('Next Song Command')
            self.spotify_controller.nextSong()
        
        if header == b'B':
            print('Previous Song Command')
            self.spotify_controller.prevSong()
        
        if header == b'S':
            print('Shuffle Command')
            self.spotify_controller.shuffleOnOffChange()

        if header == b'L':
            print('Loop Command')
            self.spotify_controller.repeatOnOff()

        if header == b'V':
            print('Volume Command')
            payload_value_percentage = self.calculate_volume(payload_value)
            if self.current_volume != payload_value_percentage:
                self.current_volume = payload_value_percentage
                print('Volume: {}%'.format(payload_value_percentage))
                self.spotify_controller.setVolume(payload_value_percentage)

        if header == b'O':
            print('Power On/Off')

    def send_handshake(self):
        data = ''
        send_data = 'H'.encode('ascii')
        print('Enviando handshake...')
        while not self.handshake_complete:
            print('Aguardando handshake...')
            if data != b'H':
                self.send_data(send_data)
                time.sleep(0.1)
                data = self.ser.read()
                if data == b'H':
                    self.handshake_complete = True
        print('Handshake completo!')

    def update(self):
        while not self.handshake_complete:
            self.send_handshake()

        data = self.ser.read(6)

        logging.debug("\nRecebido: {}\n".format(data))

        self.select_action(data)
        self.send_music_informations()

        logging.debug("Received DATA: {}".format(data))



class DummyControllerInterface:
    def __init__(self):
        print("Iniciando interface dummy...")
        self.spotify_controller = Spotify()
        print("Interface dummy iniciada com sucesso!")

    def update(self):
        print("Testando interface dummy...")
        self.spotify_controller.playPauseSong()
        time.sleep(2)



if __name__ == '__main__':
    interfaces = ['dummy', 'serial']
    argparse = argparse.ArgumentParser()
    argparse.add_argument('serial_port', type=str)
    argparse.add_argument('-b', '--baudrate', type=int, default=9600)
    argparse.add_argument('-c', '--controller_interface', type=str, default='serial', choices=interfaces)
    argparse.add_argument('-d', '--debug', default=False, action='store_true')
    args = argparse.parse_args()
    if args.debug:
        logging.basicConfig(level=logging.DEBUG)

    print("Connection to {} using {} interface ({})".format(args.serial_port, args.controller_interface, args.baudrate))
    if args.controller_interface == 'dummy':
        controller = DummyControllerInterface()
    else:
        controller = SerialControllerInterface(port=args.serial_port, baudrate=args.baudrate)

    while True:
        controller.update()
