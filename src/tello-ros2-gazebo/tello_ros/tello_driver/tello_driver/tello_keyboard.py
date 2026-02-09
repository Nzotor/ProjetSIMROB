import math
import os
import random
import sys
import termios
import tty

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import PointStamped, Pose, Twist
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import DeleteEntity, SpawnEntity
from tello_msgs.srv import TelloAction

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
        self._orig_term = None
        self._configure_terminal()

        self.get_logger().info(
            'Keyboard control: arrows=move, space=stop, t=takeoff, l=land, '
            'a=square, c=circle, h=figure8, k=ring game, g=auto ring, q=quit'
        )

        # Etat de commandes manuelles.
        self._last_cmd_time = self.get_clock().now()
        self._sent_stop = True

        # Etat des trajectoires automatiques (formes).
        self._traj_steps = None
        self._traj_index = 0
        self._traj_step_end = None

        # Etat du jeu d'anneau et du mode auto.
        self._ring_enabled = False
        self._ring_auto_enabled = False
        self._drone_pose = None
        self._ring_name = None
        self._ring_pose = None
        self._ring_yaw = 0.0
        self._last_ring_x = None
        self._ring_passed = False
        self._last_model_warn = self.get_clock().now()

        # RNG pour positions aleatoires.
        self._rng = random.Random(int(self.ring_seed))

        # Publisher pour la cible de l'anneau.
        self._ring_center_pub = self.create_publisher(PointStamped, 'ring_center', 10)

        # Services Gazebo pour spawn/suppression + abon. a la pose du drone.
        # QoS "sensor" (best effort) pour recevoir les messages Gazebo.
        self._spawn_client = self.create_client(SpawnEntity, '/spawn_entity')
        self._delete_client = self.create_client(DeleteEntity, '/delete_entity')
        self._model_sub = self.create_subscription(
            ModelStates, '/gazebo/model_states', self._on_model_states, qos_profile_sensor_data
        )
        # Fallback si le topic est publie sous /model_states.
        self._model_sub_alt = self.create_subscription(
            ModelStates, '/model_states', self._on_model_states, qos_profile_sensor_data
        )

        # Boucle principale.
        self.create_timer(0.05, self._tick)

    def destroy_node(self):
        # Restaure le terminal a la fermeture.
        self._restore_terminal()
        super().destroy_node()

    def _configure_terminal(self):
        # Active lecture non bloquante en mode cbreak.
        if not sys.stdin.isatty():
            self.get_logger().warn('stdin is not a TTY; keyboard input disabled')
            return

        self._orig_term = termios.tcgetattr(sys.stdin.fileno())
        tty.setcbreak(sys.stdin.fileno())

    def _restore_terminal(self):
        # Restaure l'etat original du terminal.
        if self._orig_term is not None:
            termios.tcsetattr(sys.stdin.fileno(), termios.TCSANOW, self._orig_term)

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
            self._ring_auto_enabled = False
            self._send_action('takeoff')
            return
        if c == 'l':
            self._cancel_trajectory()
            self._ring_auto_enabled = False
            self._send_action('land')
            return
        if c in ('k', 'K'):
            self._toggle_ring_game()
            return
        if c in ('g', 'G'):
            self._toggle_ring_auto()
            return
        if c == 'q':
            rclpy.shutdown()
            return

    def _cancel_trajectory(self):
        # Stoppe la trajectoire en cours.
        self._traj_steps = None
        self._traj_index = 0
        self._traj_step_end = None

    def _start_trajectory(self, key):
        # Construit une liste de segments de trajectoire.
        side_len = float(self.traj_side_len)
        radius = float(self.traj_radius)
        side_time = side_len / float(self.linear_speed)
        turn_time = (math.pi / 2.0) / float(self.angular_speed)
        circle_omega = float(self.linear_speed) / max(radius, 0.01)
        circle_time = (2.0 * math.pi * radius) / float(self.linear_speed)

        if key == 'a':  # Square
            self._traj_steps = [
                (side_time, self.linear_speed, 0.0, 0.0, 0.0),
                (turn_time, 0.0, 0.0, 0.0, self.angular_speed),
                (side_time, self.linear_speed, 0.0, 0.0, 0.0),
                (turn_time, 0.0, 0.0, 0.0, self.angular_speed),
                (side_time, self.linear_speed, 0.0, 0.0, 0.0),
                (turn_time, 0.0, 0.0, 0.0, self.angular_speed),
                (side_time, self.linear_speed, 0.0, 0.0, 0.0),
                (turn_time, 0.0, 0.0, 0.0, self.angular_speed),
            ]
        elif key == 'c':  # Circle
            self._traj_steps = [
                (circle_time, self.linear_speed, 0.0, 0.0, circle_omega)
            ]
        else:  # Figure eight
            self._traj_steps = [
                (circle_time, self.linear_speed, 0.0, 0.0, circle_omega),
                (circle_time, self.linear_speed, 0.0, 0.0, -circle_omega),
            ]

        self._traj_index = 0
        self._traj_step_end = self.get_clock().now()
        self._advance_trajectory_step()
        self.get_logger().info(f'Trajectory started: {key}')

    def _advance_trajectory_step(self):
        # Passe au segment suivant de la trajectoire.
        if not self._traj_steps or self._traj_index >= len(self._traj_steps):
            self._cancel_trajectory()
            self._publish_cmd(0.0, 0.0, 0.0, 0.0)
            self._sent_stop = True
            return

        duration, lin_x, lin_y, lin_z, ang_z = self._traj_steps[self._traj_index]
        now = self.get_clock().now()
        self._traj_step_end = now + Duration(seconds=float(duration))
        self._publish_cmd(lin_x, lin_y, lin_z, ang_z)
        self._sent_stop = False

    def _toggle_ring_game(self):
        # Active/desactive le jeu d'anneau.
        if self._ring_enabled:
            self._ring_enabled = False
            self._ring_auto_enabled = False
            self._delete_ring()
            self.get_logger().info('Ring game: OFF')
            return

        self._ring_enabled = True
        self._spawn_ring()
        self.get_logger().info('Ring game: ON')

    def _toggle_ring_auto(self):
        # Active/desactive l'autopilot vers le centre de l'anneau.
        if self._ring_auto_enabled:
            self._ring_auto_enabled = False
            self._publish_cmd(0.0, 0.0, 0.0, 0.0)
            self.get_logger().info('Ring auto: OFF')
            return

        if not self._ring_enabled:
            self._ring_enabled = True
            self._spawn_ring()
        self._ring_auto_enabled = True
        self.get_logger().info('Ring auto: ON')

    def _on_model_states(self, msg: ModelStates):
        # Recupere la pose du drone depuis Gazebo.
        if not msg.name:
            return

        target_name = str(self.drone_model_name).strip()
        for i, name in enumerate(msg.name):
            if target_name:
                if name == target_name:
                    self._drone_pose = msg.pose[i]
                    return
            else:
                if self.drone_model_prefix in name:
                    self._drone_pose = msg.pose[i]
                    return

        now = self.get_clock().now()
        if (now - self._last_model_warn).nanoseconds > 2e9:
            self.get_logger().warn(
                f'No drone model found. Set drone_model_name or drone_model_prefix. '
                f'Available: {", ".join(msg.name)}'
            )
            self._last_model_warn = now

    def _spawn_ring(self):
        # Cree un anneau a une position aleatoire.
        if not self._spawn_client.service_is_ready():
            self.get_logger().info('waiting for spawn_entity service...')
            self._spawn_client.wait_for_service()

        self._delete_ring()

        half = float(self.arena_half_size)
        outer = float(self.ring_outer)
        margin = float(self.wall_margin) + outer * 0.5
        span = max(0.1, half - margin)

        x = self._rng.uniform(-span, span)
        y = self._rng.uniform(-span, span)
        z = float(self.ring_height)
        yaw = self._rng.uniform(-math.pi, math.pi)

        self._ring_pose = (x, y, z)
        self._ring_yaw = yaw
        self._last_ring_x = None
        self._ring_passed = False
        self._ring_name = f'ring_{int(self.get_clock().now().nanoseconds / 1e6)}'

        sdf = self._build_ring_sdf(self._ring_name, outer, float(self.ring_thickness))
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        pose.orientation.z = math.sin(yaw * 0.5)
        pose.orientation.w = math.cos(yaw * 0.5)

        req = SpawnEntity.Request()
        req.name = self._ring_name
        req.xml = sdf
        req.initial_pose = pose
        self._spawn_client.call_async(req)
        self._publish_ring_center()

    def _delete_ring(self):
        # Supprime l'anneau courant.
        if not self._ring_name:
            return

        if not self._delete_client.service_is_ready():
            self._delete_client.wait_for_service()

        req = DeleteEntity.Request()
        req.name = self._ring_name
        self._delete_client.call_async(req)
        self._ring_name = None
        self._ring_pose = None
        self._last_ring_x = None
        self._ring_passed = False

    def _build_ring_sdf(self, name, outer, thickness):
        # Genere un SDF simple de "cadre" pour l'anneau.
        half = outer * 0.5
        inset = half - thickness * 0.5
        color = '0.2 0.8 1.0 1'
        return f"""<?xml version='1.0'?>
<sdf version='1.6'>
  <model name='{name}'>
    <static>true</static>
    <link name='link'>
      <collision name='top_collision'>
        <pose>0 0 {inset} 0 0 0</pose>
        <geometry><box><size>{thickness} {outer} {thickness}</size></box></geometry>
      </collision>
      <visual name='top'>
        <pose>0 0 {inset} 0 0 0</pose>
        <geometry><box><size>{thickness} {outer} {thickness}</size></box></geometry>
        <material><ambient>{color}</ambient><diffuse>{color}</diffuse></material>
      </visual>
      <collision name='bottom_collision'>
        <pose>0 0 {-inset} 0 0 0</pose>
        <geometry><box><size>{thickness} {outer} {thickness}</size></box></geometry>
      </collision>
      <visual name='bottom'>
        <pose>0 0 {-inset} 0 0 0</pose>
        <geometry><box><size>{thickness} {outer} {thickness}</size></box></geometry>
        <material><ambient>{color}</ambient><diffuse>{color}</diffuse></material>
      </visual>
      <collision name='left_collision'>
        <pose>0 {-inset} 0 0 0</pose>
        <geometry><box><size>{thickness} {thickness} {outer}</size></box></geometry>
      </collision>
      <visual name='left'>
        <pose>0 {-inset} 0 0 0</pose>
        <geometry><box><size>{thickness} {thickness} {outer}</size></box></geometry>
        <material><ambient>{color}</ambient><diffuse>{color}</diffuse></material>
      </visual>
      <collision name='right_collision'>
        <pose>0 {inset} 0 0 0</pose>
        <geometry><box><size>{thickness} {thickness} {outer}</size></box></geometry>
      </collision>
      <visual name='right'>
        <pose>0 {inset} 0 0 0</pose>
        <geometry><box><size>{thickness} {thickness} {outer}</size></box></geometry>
        <material><ambient>{color}</ambient><diffuse>{color}</diffuse></material>
      </visual>
    </link>
  </model>
</sdf>
"""

    def _publish_ring_center(self):
        # Publie la position du centre de l'anneau (repere monde).
        if not self._ring_pose:
            return
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'
        msg.point.x = float(self._ring_pose[0])
        msg.point.y = float(self._ring_pose[1])
        msg.point.z = float(self._ring_pose[2])
        self._ring_center_pub.publish(msg)

    def _get_yaw_from_pose(self, pose: Pose) -> float:
        # Extrait le yaw (rotation autour de Z) d'un quaternion.
        q = pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def _auto_ring_control(self):
        # Controle automatique vers le centre de l'anneau.
        if not self._ring_auto_enabled or not self._ring_pose or self._drone_pose is None:
            return False

        dx = self._ring_pose[0] - self._drone_pose.position.x
        dy = self._ring_pose[1] - self._drone_pose.position.y
        dz = self._ring_pose[2] - self._drone_pose.position.z

        if bool(self.auto_cmd_in_world):
            x_body = dx
            y_body = dy
        else:
            yaw = self._get_yaw_from_pose(self._drone_pose)
            cy = math.cos(yaw)
            sy = math.sin(yaw)
            x_body = cy * dx + sy * dy
            y_body = -sy * dx + cy * dy

        vx = x_body * float(self.auto_kp)
        vy = y_body * float(self.auto_kp)
        vz = dz * float(self.auto_kp_z)

        max_speed = float(self.auto_max_speed)
        speed = math.hypot(vx, vy)
        if speed > max_speed:
            scale = max_speed / speed
            vx *= scale
            vy *= scale
        vz = max(-max_speed, min(max_speed, vz))

        self._publish_cmd(vx, vy, vz, 0.0)
        return True

    def _check_ring_pass(self):
        # Detecte le passage dans l'anneau et respawn si valide.
        if not self._ring_enabled or not self._ring_pose or self._drone_pose is None:
            return

        dx = self._drone_pose.position.x - self._ring_pose[0]
        dy = self._drone_pose.position.y - self._ring_pose[1]
        dz = self._drone_pose.position.z - self._ring_pose[2]

        cy = math.cos(-self._ring_yaw)
        sy = math.sin(-self._ring_yaw)
        x_local = dx * cy - dy * sy
        y_local = dx * sy + dy * cy
        z_local = dz

        # Ouverture carree: demi-taille interne = outer/2 - thickness.
        inner = max(0.05, float(self.ring_outer) * 0.5 - float(self.ring_thickness))
        pass_depth = max(0.02, float(self.ring_pass_depth))
        inside_opening = abs(y_local) < inner and abs(z_local) < inner
        if not self._ring_passed and inside_opening and abs(x_local) < pass_depth:
            self._ring_passed = True
            self.get_logger().info('Ring passed')
            self._spawn_ring()
            return

        self._last_ring_x = x_local

    def _read_key(self):
        # Lecture non bloquante d'une touche.
        if not sys.stdin.isatty():
            return ''

        import select
        if not select.select([sys.stdin], [], [], 0)[0]:
            return ''

        return os.read(sys.stdin.fileno(), 1).decode(errors='ignore')

    def _tick(self):
        # Boucle periodique: lecture clavier, jeu d'anneau, trajectoires, stop.
        c = self._read_key()
        if c:
            if c == '\x1b':
                c2 = self._read_key()
                if c2 == '[':
                    c3 = self._read_key()
                    if c3:
                        self._cancel_trajectory()
                        self._ring_auto_enabled = False
                        self._handle_arrow(c3)
            else:
                self._handle_key(c)

        # Re-spawn si l'anneau n'existe plus.
        if self._ring_enabled and self._ring_name is None:
            self._spawn_ring()

        # Publie la position de l'anneau.
        if self._ring_enabled and self._ring_pose:
            self._publish_ring_center()

        # Verifie le passage dans l'anneau.
        self._check_ring_pass()

        # Autopilot vers l'anneau (prioritaire).
        if self._auto_ring_control():
            return

        # Execution des trajectoires automatiques.
        if self._traj_steps:
            now = self.get_clock().now()
            if now >= self._traj_step_end:
                self._traj_index += 1
                self._advance_trajectory_step()
            else:
                duration, lin_x, lin_y, lin_z, ang_z = self._traj_steps[self._traj_index]
                self._publish_cmd(lin_x, lin_y, lin_z, ang_z)
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
