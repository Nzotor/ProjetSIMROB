import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from tello_msgs.srv import TelloAction

from .keyboard_io import TerminalKeyboard
from .ring_game import RingGame
from .trajectory import TrajectoryController

# Controle clavier + jeu d'anneau pour le drone Tello dans Gazebo.


class TelloKeyboardNode(Node):
    def __init__(self):
        super().__init__('tello_keyboard')

        # Publisher de vitesse et service d'actions (takeoff/land).
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.tello_client = self.create_client(TelloAction, 'tello_action')

        # Parametres de vitesse et trajectoires predefinies.
        self.linear_speed = self.declare_parameter('linear_speed', 0.3).value
        self.angular_speed = self.declare_parameter('angular_speed', 0.6).value
        self.traj_side_len = self.declare_parameter('traj_side_len', 0.5).value
        self.traj_radius = self.declare_parameter('traj_radius', 0.5).value

        # Parametres du jeu d'anneau (taille, zone, detection).
        self.arena_half_size = self.declare_parameter('arena_half_size', 6.0).value
        self.wall_margin = self.declare_parameter('wall_margin', 0.8).value
        self.ring_outer = self.declare_parameter('ring_outer', 1.2).value
        self.ring_thickness = self.declare_parameter('ring_thickness', 0.1).value
        self.ring_height = self.declare_parameter('ring_height', 1.0).value
        self.ring_pass_depth = self.declare_parameter('ring_pass_depth', 0.25).value
        self.auto_kp = self.declare_parameter('auto_kp', 0.8).value
        self.auto_kp_z = self.declare_parameter('auto_kp_z', 0.6).value
        self.auto_max_speed = self.declare_parameter('auto_max_speed', 0.6).value
        self.auto_cmd_in_world = self.declare_parameter('auto_cmd_in_world', False).value
        self.drone_model_prefix = self.declare_parameter('drone_model_prefix', 'tello').value
        self.drone_model_name = self.declare_parameter('drone_model_name', '').value
        self.ring_seed = self.declare_parameter('ring_seed', 0).value

        # Delai avant l'envoi automatique d'un stop si aucune touche.
        self.stop_timeout_sec = self.declare_parameter('stop_timeout_sec', 0.25).value

        # Configuration du terminal pour lecture des touches.
        self._keyboard = TerminalKeyboard(self.get_logger())

        self.get_logger().info(
            'Keyboard control: arrows=move, space=stop, t=takeoff, l=land, '
            'a=square, c=circle, h=figure8, k=ring game, g=auto ring, q=quit'
        )

        # Etat de commandes manuelles.
        self._last_cmd_time = self.get_clock().now()
        self._sent_stop = True

        # Controle de trajectoires automatiques.
        self._trajectory = TrajectoryController(self.get_clock(), self._publish_cmd)

        # Controle du jeu d'anneau et de l'autopilot.
        self._ring_game = RingGame(
            node=self,
            publish_cmd=self._publish_cmd,
            arena_half_size=self.arena_half_size,
            wall_margin=self.wall_margin,
            ring_outer=self.ring_outer,
            ring_thickness=self.ring_thickness,
            ring_height=self.ring_height,
            ring_pass_depth=self.ring_pass_depth,
            auto_kp=self.auto_kp,
            auto_kp_z=self.auto_kp_z,
            auto_max_speed=self.auto_max_speed,
            auto_cmd_in_world=self.auto_cmd_in_world,
            drone_model_prefix=self.drone_model_prefix,
            drone_model_name=self.drone_model_name,
            ring_seed=self.ring_seed,
        )

        # Boucle principale.
        self.create_timer(0.05, self._tick)

    def destroy_node(self):
        # Restaure le terminal a la fermeture.
        self._keyboard.close()
        super().destroy_node()

    def _send_action(self, cmd):
        # Envoi d'une commande Tello via service ROS.
        if not self.tello_client.service_is_ready():
            self.get_logger().warn('tello_action service not ready')
            return

        req = TelloAction.Request()
        req.cmd = cmd
        self.tello_client.call_async(req)

    def _publish_cmd(self, lin_x, lin_y, lin_z, ang_z):
        # Publication de la vitesse.
        msg = Twist()
        msg.linear.x = lin_x
        msg.linear.y = lin_y
        msg.linear.z = lin_z
        msg.angular.z = ang_z
        self.cmd_vel_pub.publish(msg)

    def _handle_arrow(self, code):
        # Fleches: avance/recul + rotation yaw.
        if code == 'A':  # Up
            self._publish_cmd(self.linear_speed, 0.0, 0.0, 0.0)
        elif code == 'B':  # Down
            self._publish_cmd(-self.linear_speed, 0.0, 0.0, 0.0)
        elif code == 'C':  # Right
            self._publish_cmd(0.0, 0.0, 0.0, -self.angular_speed)
        elif code == 'D':  # Left
            self._publish_cmd(0.0, 0.0, 0.0, self.angular_speed)
        else:
            return

        self._last_cmd_time = self.get_clock().now()
        self._sent_stop = False

    def _handle_key(self, c):
        # Commandes clavier simples.
        if c in ('a', 'A', 'c', 'C', 'h', 'H'):
            self._start_trajectory(c.lower())
            return
        if c == ' ':
            self._cancel_trajectory()
            self._publish_cmd(0.0, 0.0, 0.0, 0.0)
            self._last_cmd_time = self.get_clock().now()
            self._sent_stop = True
            return
        if c == 't':
            self._cancel_trajectory()
            self._ring_game.disable_auto()
            self._send_action('takeoff')
            return
        if c == 'l':
            self._cancel_trajectory()
            self._ring_game.disable_auto()
            self._send_action('land')
            return
        if c in ('k', 'K'):
            self._ring_game.toggle_game()
            return
        if c in ('g', 'G'):
            self._ring_game.toggle_auto()
            return
        if c == 'q':
            rclpy.shutdown()
            return

    def _cancel_trajectory(self):
        # Stoppe la trajectoire en cours.
        self._trajectory.cancel()

    def _start_trajectory(self, key):
        # Demarre une trajectoire predefinie.
        if self._trajectory.start(
            key,
            self.linear_speed,
            self.angular_speed,
            self.traj_side_len,
            self.traj_radius,
        ):
            self._sent_stop = self._trajectory.sent_stop
            self.get_logger().info(f'Trajectory started: {key}')

    def _tick(self):
        # Boucle periodique: lecture clavier, jeu d'anneau, trajectoires, stop.
        c = self._keyboard.read_key()
        if c:
            if c == '\x1b':
                c2 = self._keyboard.read_key()
                if c2 == '[':
                    c3 = self._keyboard.read_key()
                    if c3:
                        self._cancel_trajectory()
                        self._ring_game.disable_auto()
                        self._handle_arrow(c3)
            else:
                self._handle_key(c)

        # Jeu d'anneau + autopilot (prioritaire).
        if self._ring_game.tick():
            return

        # Execution des trajectoires automatiques.
        if self._trajectory.tick():
            self._sent_stop = self._trajectory.sent_stop
            return

        # Envoie un stop si aucune commande recente.
        if not self._sent_stop:
            elapsed = (self.get_clock().now() - self._last_cmd_time).nanoseconds / 1e9
            if elapsed > float(self.stop_timeout_sec):
                self._publish_cmd(0.0, 0.0, 0.0, 0.0)
                self._sent_stop = True


def main():
    # Point d'entree ROS2.
    rclpy.init()
    node = TelloKeyboardNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
