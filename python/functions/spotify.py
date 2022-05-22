import os
import spotipy
from spotipy.oauth2 import SpotifyOAuth

CLIENT_ID = "0b4995069ef9464baf4873c547558030"
CLIENT_SECRET = "bd40f9c2b0a6410390aa1c9ebb346f30"
REDIRECT_URI = "http://localhost:8888/callback"
SCOPE = "user-modify-playback-state user-read-recently-played streaming user-read-currently-playing user-read-playback-state user-read-email user-read-private"
USERNAME = 'davi_reis_vieira'

class Spotify():

    def __init__(self):
        print("Iniciando ARM V Control Spotify Session...")
        self.sp = spotipy.Spotify(auth_manager=SpotifyOAuth(client_id=CLIENT_ID, client_secret=CLIENT_SECRET, redirect_uri=REDIRECT_URI, scope=SCOPE))
        while not self.sp.current_playback():
            print("Iniciando Spotify...")
            while True:
                if os.system("tasklist | find /i /c /n \"spotify.exe\"") == 0:
                    break
                else:
                    os.system("spotify.exe")
        print("Spotify iniciado com sucesso!")
        print("ARM V Control Spotify Session iniciada com sucesso!")
        devices = self.getDevices()
        print("Dispositivo utilizado: " + devices[0])
        self.current_playback = self.getCurrentPlayback(True)
    
    # function to reduce current_callback callback
    def getCurrentPlayback(self, refresh=False):
        if refresh:
            self.current_playback = self.sp.current_playback()
        return self.current_playback

    def getDevices(self):
        dev = []
        for i in range(len(self.sp.devices()['devices'])):
            dev.append(self.sp.devices()['devices'][i]['name'])
        return dev

    def getCurrentDevice(self, refresh=False):
        return self.getCurrentPlayback(refresh)['device']
    
    def getCurrentSongName(self, refresh=False):
        return self.getCurrentPlayback(refresh)['item']['name']

    def getCurrentSongArtist(self, refresh=False):
        return self.getCurrentPlayback(refresh)['item']['artists'][0]['name']
    
    def getCurrentSongDuration(self, refresh=False):
        return self.getCurrentPlayback(refresh)['item']['duration_ms']

    def getCurrentSongStatus(self, refresh=False):
        return self.getCurrentPlayback(refresh)['is_playing']

    def getShuffleStatus(self, refresh=False):
        return self.getCurrentPlayback(refresh)['shuffle_state']

    def getRepeatMode(self, refresh=False):
        repeat_state = self.getCurrentPlayback(refresh)['repeat_state']
        if repeat_state == 'track':
            return '1'
        elif repeat_state == 'context':
            return '2'
        else:
            return '0'

    def selectDevice(self, device):
        devices = self.getDevices()
        print("Dispositivos:")
        for i in range(len(devices)):
            print(str(i+1) + " - " + devices[i])
        print("")
        print("Selecione um dispositivo:")
        device = int(input())
        self.sp.transfer_playback(device_id=self.sp.devices()['devices'][device-1]['id'])
        print("Dispositivo selecionado: " + self.sp.devices()['devices'][device-1]['name'])
    
    def playPauseSong(self):
        if self.getCurrentSongStatus(True):
            self.sp.pause_playback()
        else:
            self.sp.start_playback()

    def prevSong(self):
        self.sp.previous_track()

    def nextSong(self):
        self.sp.next_track()

    def shuffleOnOffChange(self):
        if self.getShuffleStatus():
            self.sp.shuffle(False)
        else:
            self.sp.shuffle(True)


    def repeatOnOff(self):
        if self.getRepeatMode(True) == '1':
            self.sp.repeat('context')
        elif self.getRepeatMode(True) == '2':
            self.sp.repeat('off')
        else:
            self.sp.repeat('track')

    def setVolume(self, volume):
        self.sp.volume(volume)

    def getVolume(self):
        volume = self.getCurrentDevice(True)['volume_percent']
        # format volume to 3 digits
        if volume < 10:
            volume = "00" + str(volume)
        elif volume < 100:
            volume = "0" + str(volume)
        else:
            volume = str(volume)
        return volume

