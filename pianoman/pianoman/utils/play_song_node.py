import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from me134_interfaces.msg import SongMsg

import mido
import pianoman.utils.midi_helper as midi_helper

import time

names = mido.get_input_names()

RATE = 200.0 # Hz
SONGS_DIR = "/home/robot134/songs/"

notes_to_songs = {tuple(['E5', 'E5', 'F5']) : ("Beethhoven_edited.mid", 45),
                  tuple(['E4', 'E4', 'F4']) : ("Beethhoven_short.mid", 45),
                  tuple(['C6', 'C6', 'C6']) : ("kanye_lower.mid", 140),
                  tuple(['G4', 'D5', 'E5']) : ("lalalandTheme_edited.mid", 30),
                  tuple(['C4', 'C4', 'D4']) : ("leanOnMe.mid", 30),
                  tuple(['C4', 'F4', 'A4']) : ("amazingGrace.mid", 45),
                  tuple(['E4', 'G4', 'G4']) : ("hallelujah.mid", 35),
                  tuple(['D4', 'D4', 'D4']) : ("hotlineBling.mid", 30),
                  tuple(['E4', 'F#4', 'A#4']) : ("dancingQueen.mid", 35),
                  tuple(['F4', 'G4', 'C4']) : ("rickRoll.mid", 10),
                  tuple(['C4', 'C5', 'A4']) : ("ballgame.mid", 45),
                  tuple(['B4', 'B4', 'D5']) : ("bonJovi.mid", 35)
                  }


class PlaySongNode(Node):

    def __init__(self):
        super().__init__('play_song_node')

        self.pub = self.create_publisher(SongMsg, '/song_cmd', 30)
        self.note_pub = self.create_publisher(String, '/note', 30)

        self.last_three_notes = []
        simon_mode = False

        # msg = SongMsg()
        # msg.song_name = SONGS_DIR + "/lalalandTheme_edited.mid"
        # msg.bpm = 30
        # msg.simon_mode = False
        # self.pub.publish(msg)
        # return

        with mido.open_input(names[1]) as inport:
            for msg in inport:
                print(msg)
                if msg.type == 'control_change' and msg.channel == 0 and msg.control == 48:
                    simon_mode = msg.value == 127
                    m = SongMsg()
                    m.song_name = ""
                    m.bpm = 0
                    m.simon_mode = simon_mode
                    self.pub.publish(m)
                if msg.type == 'note_off':
                    # else:
                        print("piano note played")
                        note_string = midi_helper._midi_value_to_note(msg.note)

                        if len(self.last_three_notes) == 3:
                            self.last_three_notes.pop(0)
                        self.last_three_notes.append(note_string)

                        if tuple(self.last_three_notes) in notes_to_songs:
                            song, bpm = notes_to_songs[tuple(self.last_three_notes)]
                            song_name = song[:song.find('.')]

                            filename = SONGS_DIR + song
                            msg = SongMsg()
                            msg.song_name = filename
                            msg.bpm = bpm
                            msg.simon_mode = simon_mode
                            self.pub.publish(msg)

                            print(f"Playing {song_name}")

                        s = String()
                        s.data = note_string
                        self.note_pub.publish(s)


def main(args=None):
    # intialize ROS node.
    rclpy.init(args=args)
    node = PlaySongNode()

    rclpy.spin(node)

    # Destroy the node explicitly
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
