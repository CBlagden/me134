
clap = Clap(5)
self.clap_clock = StateClock(self.get_clock().now(), rostime=True)

if (self.state = State.CLAP):
    # default to floating
    poscmd = 7 * [float("NaN")]
    velcmd = 7 * [float("NaN")]
    effcmd = self.gravity()

    # perform the "clap" trajectory
    t = self.get_clock().now()
    s = self.clap_clock.t_since_start(t, rostime=True)

    if (clap.completed(s)):
        self.state = State.FLOAT
    else:
        grip_cmd = clap.evaluate(s)

    cmdmsg = self.fbk.to_msg(self.get_clock().now(), poscmd, velcmd, effcmd, self.grip_effcmd)
    self.pub_jtcmd.publish(cmdmsg)