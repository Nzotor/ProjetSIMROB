import math

from rclpy.duration import Duration


class TrajectoryController:
    def __init__(self, clock, publish_cmd):
        self._clock = clock
        self._publish_cmd = publish_cmd
        self._steps = None
        self._index = 0
        self._step_end = None
        self.sent_stop = True

    def cancel(self):
        # Stoppe la trajectoire en cours.
        self._steps = None
        self._index = 0
        self._step_end = None

    def start(self, key, linear_speed, angular_speed, traj_side_len, traj_radius):
        # Construit une liste de segments de trajectoire.
        side_len = float(traj_side_len)
        radius = float(traj_radius)
        side_time = side_len / float(linear_speed)
        turn_time = (math.pi / 2.0) / float(angular_speed)
        circle_omega = float(linear_speed) / max(radius, 0.01)
        circle_time = (2.0 * math.pi * radius) / float(linear_speed)

        if key == 'a':  # Square
            self._steps = [
                (side_time, linear_speed, 0.0, 0.0, 0.0),
                (turn_time, 0.0, 0.0, 0.0, angular_speed),
                (side_time, linear_speed, 0.0, 0.0, 0.0),
                (turn_time, 0.0, 0.0, 0.0, angular_speed),
                (side_time, linear_speed, 0.0, 0.0, 0.0),
                (turn_time, 0.0, 0.0, 0.0, angular_speed),
                (side_time, linear_speed, 0.0, 0.0, 0.0),
                (turn_time, 0.0, 0.0, 0.0, angular_speed),
            ]
        elif key == 'c':  # Circle
            self._steps = [
                (circle_time, linear_speed, 0.0, 0.0, circle_omega)
            ]
        elif key == 'h':  # Figure eight
            self._steps = [
                (circle_time, linear_speed, 0.0, 0.0, circle_omega),
                (circle_time, linear_speed, 0.0, 0.0, -circle_omega),
            ]
        else:
            return False

        self._index = 0
        self._step_end = self._clock.now()
        self._advance_step()
        return True

    def _advance_step(self):
        # Passe au segment suivant de la trajectoire.
        if not self._steps or self._index >= len(self._steps):
            self.cancel()
            self._publish_cmd(0.0, 0.0, 0.0, 0.0)
            self.sent_stop = True
            return False

        duration, lin_x, lin_y, lin_z, ang_z = self._steps[self._index]
        now = self._clock.now()
        self._step_end = now + Duration(seconds=float(duration))
        self._publish_cmd(lin_x, lin_y, lin_z, ang_z)
        self.sent_stop = False
        return True

    def tick(self):
        if not self._steps:
            return False

        now = self._clock.now()
        if now >= self._step_end:
            self._index += 1
            self._advance_step()
        else:
            duration, lin_x, lin_y, lin_z, ang_z = self._steps[self._index]
            self._publish_cmd(lin_x, lin_y, lin_z, ang_z)
        return True
