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

notes_to_songs = {tuple(['E5', 'E5', 'F5']) : ("Beethhoven_editedSimple.mid", 45),
                  tuple(['C6', 'C6', 'C6']) : ("kanye_lower.mid", 45)}


class PlaySongNode(Node):

    def __init__(self):
        super().__init__('play_song_node')

        self.pub = self.create_publisher(SongMsg, '/song_cmd', 30)

        self.last_three_notes = []

        with mido.open_input(names[1]) as inport:
            for msg in inport:
                if msg.type == 'note_off':
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
                        self.pub.publish(msg)
                        print(f"Playing {song_name}")


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
