class P2P_Trajectory:
    """ point to point trajectory generation in fifth order
    """

    def __init__(self, start_p: float, end_p: float, start_v: float, end_v: float,
                 start_a: float, end_a: float, start_t: float, end_t: float):
        """ Initialize the trajectory
                Params
                ======
                start_p (float): start position
                end_p (float): end position
                start_v (float): start velocity
                end_v (float): end velocity
                start_a (float): start acceleration
                end_a (float): end acceleration
                start_a (float): trajectory start time
                end_t (float): trajectory end time
        """
        self.start_p = start_p
        self.end_p = end_p
        self.start_v = start_v
        self.end_v = end_v
        self.start_a = start_a
        self.end_a = end_a
        self.start_t = start_t
        self.end_t = end_t

        total_t = end_t - start_t
        p1 = total_t
        p2 = p1 * p1
        p3 = p2 * p1
        p4 = p3 * p1
        p5 = p4 * p1

        self.a0 = start_p
        self.a1 = start_v
        self.a2 = 0.5 * start_a
        self.a3 = (20. * (end_p - start_p) -
                   (8. * end_v + 12. * start_v) * p1 -
                   (3. * start_a - end_a) * p2) / (2. * p3)
        self.a4 = (30. * (start_p - end_p) +
                   (14. * end_v + 16. * start_v) * p1 +
                   (3. * start_a - 2. * end_a) * p2) / (2. * p4)
        self.a5 = (12. * (end_p - start_p) - 6. * (end_v + start_v) * p1 -
                   (end_a - start_a) * p2) / (2. * p5)

    def reset(self, start_p, end_p, start_v, end_v, start_a, end_a, start_t,
              end_t):
        """ generate a new trajectory
                Params
                ======
                start_p (float): start position
                end_p (float): end position
                start_v (float): start velocity
                end_v (float): end velocity
                start_a (float): start acceleration
                end_a (float): end acceleration
                start_a (float): trajectory start time
                end_t (float): trajectory end time
        """
        self.start_p = start_p
        self.end_p = end_p
        self.start_v = start_v
        self.end_v = end_v
        self.start_a = start_a
        self.end_a = end_a
        self.start_t = start_t
        self.end_t = end_t

        total_t = end_t - start_t
        p1 = total_t
        p2 = p1 * p1
        p3 = p2 * p1
        p4 = p3 * p1
        p5 = p4 * p1

        self.a0 = start_p
        self.a1 = start_v
        self.a2 = 0.5 * start_a
        self.a3 = (20. * (end_p - start_p) -
                   (8. * end_v + 12. * start_v) * p1 -
                   (3. * start_a - end_a) * p2) / (2. * p3)
        self.a4 = (30. * (start_p - end_p) +
                   (14. * end_v + 16. * start_v) * p1 +
                   (3. * start_a - 2. * end_a) * p2) / (2. * p4)
        self.a5 = (12. * (end_p - start_p) - 6. * (end_v + start_v) * p1 -
                   (end_a - start_a) * p2) / (2. * p5)

    def get_point(self, current_time):
        """ get the reference point according to the current time
                Params
                ======
                current_time (float): current servo time
        """
        pos_cmd = 0.
        vel_cmd = 0.
        acc_cmd = 0.
        if current_time <= self.start_t:
            pos_cmd = self.start_p
            vel_cmd = self.start_v
            acc_cmd = self.start_a
        elif current_time > self.end_t:
            pos_cmd = self.end_p
            vel_cmd = self.end_v
            acc_cmd = self.end_a
        else:
            t = current_time - self.start_t
            t2 = t * t
            t3 = t2 * t
            t4 = t3 * t
            t5 = t4 * t
            pos_cmd = (self.a0 + self.a1 * t + self.a2 * t2 + self.a3 * t3 +
                       self.a4 * t4 + self.a5 * t5)
            vel_cmd = (self.a1 + 2. * self.a2 * t + 3. * self.a3 * t2 + 4. *
                       self.a4 * t3 + 5. * self.a5 * t4)
            acc_cmd = (2. * self.a2 + 6. * self.a3 * t + 12. * self.a4 * t2 +
                       20. * self.a5 * t3)

        return pos_cmd, vel_cmd, acc_cmd
