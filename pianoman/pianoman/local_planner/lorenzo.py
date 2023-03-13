def neutral_above_piano(self):
    # get keyboard position
    R_kb_world = Rotz(-np.pi/2) @ self.kb_rot
    delta = np.array([*list(self.kb_pos[:2].flatten()), 0.0]).reshape((3, 1))

    # chill somewhere above the piano
    neutral_kb = [np.array([0.1, 0.0, 0.5]).reshape([3,1]), np.array([0.6, 0.0, 0.5]).reshape([3,1])]

    # spline from current position
    for LRidx in range(2):
        if (self.playsplines[LRidx]):
            # pull last position from last spline
            tend = self.playsplines[LRidx][-1].get_duration()
            p_start, _ = self.playsplines[LRidx][-1].evaluate(tend)

            if (self.playsplines[LRidx][-1].get_space() == 'task'):
                self.get_logger().error("Tried to append a keyboard space spline to a task space spline.")
        else:
            # pull current position
            [p_start, _, _, _, _] = self.fbk.fkin()
            if (LRidx == 0):
                p_start = p_start[0:3, :]
                # initialize the clock
                self.play_clock_L = StateClock(self.get_clock().now(), rostime=True)
            else:
                p_start = p_start[3:, :]
                # initialize the clock
                self.play_clock_R = StateClock(self.get_clock().now(), rostime=True)
            # convert to kb frame
            p_start = self.robot_to_kb_frame(p_start, R_kb_world, delta)

        move_time = distance_to_movetime(p_start, neutral_kb[LRidx])
        spline = Goto5(p_start, neutral_kb[LRidx], move_time, space='keyboard')
        if (self.playsplines[LRidx] == None):
            self.playsplines[LRidx] = [spline]
        else:
            self.playsplines[LRidx].append(spline)

