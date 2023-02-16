from mido import MidiFile, Message

from typing import List, Tuple, Optional


# Indicates the x and y position of a single note relative to fixed position on the keyboard.
NotePosition = Tuple[float, float]

# A start and end time indicating when a note is start being played and when it will stop being played.
TimeSegment = Tuple[float, float]

# A list of tuple of TimeSegments and NotePositions, indicating what position should be pressed and for how long.
TrackTrajectory = List[Tuple[TimeSegment, NotePosition]]

#fixed distances (IN INCHES, I'M SORRY)
btwnSpacing = 0.05536
keyWidth = 0.875
halfKey = keyWidth/2
outerSpacing = 0.1
edge2C2 = 6.175 + outerSpacing + halfKey

NOTES = ['C', 'C#', 'D', 'D#', 'E', 'F', 'F#', 'G', 'G#', 'A', 'A#', 'B']
noteDistance = [0.0, #C
               0.0,
               keyWidth + btwnSpacing, #D
               0.0,
               (keyWidth + btwnSpacing)*2, #E
               (keyWidth + btwnSpacing)*3, #F
               0.0,
               (keyWidth + btwnSpacing)*4, #G
               0.0,
               (keyWidth + btwnSpacing)*5,
               0.0,
               (keyWidth + btwnSpacing)*6] #note distances in a local frame

octaveDist = (keyWidth + btwnSpacing)*7



def note_to_position(note: int) -> NotePosition:
   """
      TODO: need map between notes and keyboard positions.
      Given a message.note as specified in the Mido library, returns the x and y position of the note relative to a fixed position on the keyboard: 
   """
   #we define (0,0) as the lower left hand corner of the keyboard
   #we will also assume that the lowest ley is C2 and the highest key is C6

   if note >= 36 and note <= 84:
      octave = (note-36)//12 #determines which local octave we're working in
      localNote = (note - 36) % 12

      x = edge2C2 + octave*octaveDist + noteDistance[localNote]
      y = 1.25 #constant offset for now, may change later depending on which kind of key we're playing (ALSO IN INCHES I'M SORRY)
   
   else:
      print("not a valid note, double check octave settings")
      x = edge2C2
      y = 1.25

   #convert to metric (in cm)
   x = x*2.54
   y = y*2.54

   return (x, y) #in cm


def note_trajectories(midi_file, bpm: int) -> Tuple[TrackTrajectory, TrackTrajectory]:
   """
      Given the path to a MIDI file returns the trajectories for the left and right hands relative to a position on the keyboard:

   Args:
     midi_file: string indicating the location of the MIDI file. The MIDI must contain at least two tracks explicitly labelled LeftHand and RightHand.
     bpm: An integer specifying the beats per minute at which the song would be played.

   Returns:
     A tuple of two different track trajectories specifying the movements for the left and right hands over time.
   """
   mid = MidiFile(midi_file)
   tracks = mid.tracks
   for track in tracks:
      for msg in track:
         if msg.is_meta and msg.type == 'track_name':
            if msg.name == 'RightHand':
               right_track = track
               break
            elif msg.name == 'LeftHand':
               left_track = track
               break

   assert right_track is not None, 'No right track in file {}'.format(midi_file)
   assert left_track is not None, 'No left track in file {}'.format(midi_file)

   left_trajectory = _get_trajectory(left_track, mid.ticks_per_beat, bpm)
   right_trajectory = _get_trajectory(right_track, mid.ticks_per_beat, bpm)
   return left_trajectory, right_trajectory


def _get_trajectory(track: List[Message],
                    ticks_per_beat: int,
                    bpm: int) -> TrackTrajectory:
   """
      Given a MIDI track - which is just a list of MIDI messages - returns the corresponding times and positions of the notes being played.

   Args:
      track: A list of MIDI messages. This will always be one of the MIDI file tracks.
      ticks_per_beat: How many ticks are played per beat - this is defined in the midi file.
      bpm: The beats per minute at which the song should be played.
   """
   trajectory = []
   for msg in track:
      if not msg.is_meta:
         delta_sec = msg.time / ticks_per_beat / bpm * 60
         if msg.type == 'note_on':
            start_time += delta_sec
         elif msg.type == 'note_off':
            # TODO: convert this into a relative keyboard position instead
            # note, octave = number_to_note(msg.note)
            x, y = note_to_position(msg.note)

            end_time = start_time + delta_sec
            trajectory.append((start_time, end_time), (x, y))

            start_time = end_time
   return trajectory

print(note_to_position(60))
