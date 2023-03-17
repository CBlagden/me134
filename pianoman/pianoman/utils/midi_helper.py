from mido import MidiFile, Message
import numpy as np

from typing import List, Tuple, Optional


# Indicates the x and y position of a single note relative to fixed position on the keyboard.
NotePosition = Tuple[float, float, float]

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
blackKeyWidth = 0.56
halfBlackKey = blackKeyWidth/2
halfBtwnSpacing = btwnSpacing/2

C = -0.75 #was previously 0.75, i don't think we need this anymore though
offset = 0
offset2 = 0
offset3 = 0

NOTES = ['C', 'C#', 'D', 'D#', 'E', 'F', 'F#', 'G', 'G#', 'A', 'A#', 'B']
noteDistance = [0.0, # C
               0.0 + halfKey + 0.1775 - halfBlackKey - offset, # C#
               keyWidth + btwnSpacing, # D
               keyWidth + btwnSpacing + halfKey + halfBtwnSpacing + 0.3625 - halfBlackKey - offset, # D#
               (keyWidth + btwnSpacing)*2, # E
               (keyWidth + btwnSpacing)*3, # F
               (keyWidth + btwnSpacing)*3 + halfKey + halfBtwnSpacing + 0.1305 - halfBlackKey - offset, # F#
               (keyWidth + btwnSpacing)*4, # G
               (keyWidth + btwnSpacing)*4 + halfKey + halfBtwnSpacing + 0.275 - halfBlackKey - offset, # G#
               (keyWidth + btwnSpacing)*5, # A
               (keyWidth + btwnSpacing)*5 + halfKey + halfBtwnSpacing + 0.3770 - halfBlackKey - offset, # A#
               (keyWidth + btwnSpacing)*6] # B #note distances in a local frame 

octaveDist = (keyWidth + btwnSpacing)*7

def _note_to_midi_value(note: str) -> int:
    octave = int(note[-1])
    if "#" in note:
       letter = note[:2]
    else:
       letter = note[0]
 
    midi_value = NOTES.index(letter)
    midi_value += len(NOTES) * (octave + 1)

    return midi_value

def _midi_value_to_note(number: int) -> tuple:
    octave = number // len(NOTES)
    note = NOTES[number % len(NOTES)]

    return f"{note}{octave}"


def note_to_position(note) -> NotePosition:
   """
      Maps between notes and keyboard positions.
      Given a message.note as specified in the Mido library, returns the x and y position of the note relative to a fixed position on the keyboard: 
   """
   #we define (0,0) as the lower left hand corner of the keyboard
   #we will also assume that the lowest ley is C2 and the highest key is C6
   if isinstance(note, str):
       note = _note_to_midi_value(note)

   if note >= 36 and note <= 84: #determines if this is a valid note
      octave = (note-36)//12 #determines which local octave we're working in
      localNote = (note - 36) % 12

      x = edge2C2 + octave*octaveDist + noteDistance[localNote] # # TODO: remove?
      if localNote in [1, 3, 6, 8, 10]: #if it's a black key, adjust the y
         y = 3 + C
         z = 1.15 + 0.9605 + 0.395 + offset2
      else: #note is a white key
         y = 1.25 + C#(ALSO IN INCHES I'M SORRY)
         z = 1.15 + 0.9605 + offset3
   else:
      print("not a valid note, double check octave settings")
      #play C instead
      x = edge2C2
      y = 1.25 + C
      z = 1.15+0.9605

   #convert to metric (in m)
   x = x*0.0254
   y = y*0.0254
   z = z*0.0254

   return (x, y, z) #in m


def note_trajectories(midi_file, left_bpm: int,
                                 right_bpm) -> Tuple[TrackTrajectory, TrackTrajectory]:
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
            name = msg.name.replace(' ', '')
            if name == 'RightHand':
               right_track = track
               break
            elif name == 'LeftHand':
               left_track = track
               break

   assert right_track is not None, 'No right track in file {}'.format(midi_file)
   assert left_track is not None, 'No left track in file {}'.format(midi_file)

   left_trajectory = _get_trajectory2(left_track, mid.ticks_per_beat, left_bpm, True)
   right_trajectory = _get_trajectory2(right_track, mid.ticks_per_beat, right_bpm, False)

   return left_trajectory, right_trajectory


# def _get_trajectory(track: List[Message],
#                     ticks_per_beat: int,
#                     bpm: int,
#                     isLeft) -> TrackTrajectory:
#    """
#       Given a MIDI track - which is just a list of MIDI messages - returns the corresponding times and positions of the notes being played.

#    Args:
#       track: A list of MIDI messages. This will always be one of the MIDI file tracks.
#       ticks_per_beat: How many ticks are played per beat - this is defined in the midi file.
#       bpm: The beats per minute at which the song should be played.
#    """
#    move_time = 0.4 + 0.3 + 0.2
#    #move_time = 0.3
#    trajectory = []
#    start_time = 0.0
#    for idx, msg in enumerate(track):

#       if not msg.is_meta:
#          #print(msg)
#          #print("ticks:", ticks_per_beat)
#          #print(msg.time)
#          delta_sec = msg.time / ticks_per_beat / bpm * 60
#          #print("delta_sec:", delta_sec)
#          if msg.type == 'note_on':
#             start_time += delta_sec
#             #print(start_time)
            

#          elif msg.type == 'note_off':
#             if isLeft:
#                # delta_sec -= (move_time+0.05) #theoretically this is constant
#                delta_sec -= move_time
#             else:
#                delta_sec -= move_time 

#             if delta_sec <= 0.0: #give it a lower bound in case the note is really short
#                delta_sec = 0.2 #make it play really short
#                print(f"Note {idx}: not enough time to move between notes. Hand timing will not work. Try making BPM smaller.")

#             # x, y, z = note_to_position(msg.note)
#             note_string = _midi_value_to_note(msg.note)
#             # end_time = start_time + delta_sec
#             # trajectory.append([(start_time, end_time), (x, y, z)])
#             # trajectory.append([delta_sec, (x, y, z)])
#             trajectory.append((note_string, delta_sec))
#             # start_time = end_time
   
#    return trajectory


def note_dist_to_movetime(noteA, noteB, notes=True):
   if (notes):
      xA, yA, zA = note_to_position(noteA)
      posA = np.array([xA, yA, zA])
      xB, yB, zB = note_to_position(noteB)
      posB = np.array([xB, yB, zB])
   else:
      posA = np.array([noteA[0,0], noteA[1,0], noteA[2,0]])
      posB = np.array([noteB[0,0], noteB[1,0], noteB[2,0]])

   dist = np.linalg.norm(posA - posB)

   speed = 0.15 # m/s
   time = dist / speed

   return time



def _get_trajectory2(track: List[Message],
                    ticks_per_beat: int,
                    bpm: int,
                    isLeft) -> TrackTrajectory:
   """
      Given a MIDI track - which is just a list of MIDI messages - returns the corresponding times and positions of the notes being played.

   Args:
      track: A list of MIDI messages. This will always be one of the MIDI file tracks.
      ticks_per_beat: How many ticks are played per beat - this is defined in the midi file.
      bpm: The beats per minute at which the song should be played.
   """
   # FIRST: loop through all notes to create a list of notes to be played
   notelist = len(track) * ['']
   last_note_idx = 0
   for idx, msg in enumerate(track):
      if not msg.is_meta:
         if (msg.type == 'note_off'):
            notelist[idx] = _midi_value_to_note(msg.note)
            last_note_idx = idx

   # SECOND: loop through notes and adjust hold time based on distance to next note
   fixed_move_time = 0.3 + 0.2
   trajectory = []
   start_time = 0.0
   for idx, msg in enumerate(track):
      # for all other notes:
      if not msg.is_meta:
         delta_sec = msg.time / ticks_per_beat / bpm * 60
         if msg.type == 'note_on':
            start_time += delta_sec
         
         elif msg.type =='note_off':
            # don't adjust the hold time of the last note
            if (idx == last_note_idx):
               delta_sec = msg.time / ticks_per_beat / bpm * 60
               note_string = _midi_value_to_note(msg.note)
               trajectory.append((note_string, delta_sec))
               break

            note_string = _midi_value_to_note(msg.note)

            delta_sec -= fixed_move_time
            # compute distance to next note
            delta_sec -= note_dist_to_movetime(note_string, next(n for n in notelist[idx+1:] if n))

            if delta_sec <= 0.0: #give it a lower bound in case the note is really short
               print(f"Note {idx}, {note_string}, {isLeft}: delta_sec {delta_sec}")
               print("ohno")
               print(isLeft)
               print(ticks_per_beat)
               delta_sec = 0.2 #make it play really short
               # print(f"Note {idx}: not enough time to move between notes. Hand timing will not work. Try making BPM smaller.")

            trajectory.append((note_string, delta_sec))
   
   return trajectory




if __name__ == '__main__':
   filename = '/home/robot134/songs/kanye_lower.mid'
   b = 140
   left_traj, right_traj = note_trajectories(filename, left_bpm=b, right_bpm=b)
   print(left_traj)
   print("__________________________________________________________")
   print(right_traj)








