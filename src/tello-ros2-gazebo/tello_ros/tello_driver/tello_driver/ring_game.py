import math
import random

from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import PointStamped, Pose
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import DeleteEntity, SpawnEntity


class RingGame:
    def __init__(
        self,
        node,
        publish_cmd,
        arena_half_size,
        wall_margin,
        ring_outer,
        ring_thickness,
        ring_height,
        ring_pass_depth,
        auto_kp,
        auto_kp_z,
        auto_max_speed,
        auto_cmd_in_world,
        drone_model_prefix,
        drone_model_name,
        ring_seed,
    ):
        self._node = node
        self._publish_cmd = publish_cmd
        self._logger = node.get_logger()
        self._clock = node.get_clock()

        # Parametres du jeu d'anneau.
        self.arena_half_size = arena_half_size
        self.wall_margin = wall_margin
        self.ring_outer = ring_outer
        self.ring_thickness = ring_thickness
        self.ring_height = ring_height
        self.ring_pass_depth = ring_pass_depth
        self.auto_kp = auto_kp
        self.auto_kp_z = auto_kp_z
        self.auto_max_speed = auto_max_speed
        self.auto_cmd_in_world = auto_cmd_in_world
        self.drone_model_prefix = drone_model_prefix
        self.drone_model_name = drone_model_name

        # Etat du jeu d'anneau et du mode auto.
        self._ring_enabled = False
        self._ring_auto_enabled = False
        self._drone_pose = None
        self._ring_name = None
        self._ring_pose = None
        self._ring_yaw = 0.0
        self._last_ring_x = None
        self._ring_passed = False
        self._last_model_warn = self._clock.now()

        # RNG pour positions aleatoires.
        self._rng = random.Random(int(ring_seed))

        # Publisher pour la cible de l'anneau.
        self._ring_center_pub = node.create_publisher(PointStamped, 'ring_center', 10)

        # Services Gazebo pour spawn/suppression + abon. a la pose du drone.
        self._spawn_client = node.create_client(SpawnEntity, '/spawn_entity')
        self._delete_client = node.create_client(DeleteEntity, '/delete_entity')
        self._model_sub = node.create_subscription(
            ModelStates, '/gazebo/model_states', self._on_model_states, qos_profile_sensor_data
        )
        # Fallback si le topic est publie sous /model_states.
        self._model_sub_alt = node.create_subscription(
            ModelStates, '/model_states', self._on_model_states, qos_profile_sensor_data
        )

    def disable_auto(self):
        self._ring_auto_enabled = False

    def toggle_game(self):
        # Active/desactive le jeu d'anneau.
        if self._ring_enabled:
            self._ring_enabled = False
            self._ring_auto_enabled = False
            self._delete_ring()
            self._logger.info('Ring game: OFF')
            return

        self._ring_enabled = True
        self._spawn_ring()
        self._logger.info('Ring game: ON')

    def toggle_auto(self):
        # Active/desactive l'autopilot vers le centre de l'anneau.
        if self._ring_auto_enabled:
            self._ring_auto_enabled = False
            self._publish_cmd(0.0, 0.0, 0.0, 0.0)
            self._logger.info('Ring auto: OFF')
            return

        if not self._ring_enabled:
            self._ring_enabled = True
            self._spawn_ring()
        self._ring_auto_enabled = True
        self._logger.info('Ring auto: ON')

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

        now = self._clock.now()
        if (now - self._last_model_warn).nanoseconds > 2e9:
            self._logger.warn(
                f'No drone model found. Set drone_model_name or drone_model_prefix. '
                f'Available: {", ".join(msg.name)}'
            )
            self._last_model_warn = now

    def _spawn_ring(self):
        # Cree un anneau a une position aleatoire.
        if not self._spawn_client.service_is_ready():
            self._logger.info('waiting for spawn_entity service...')
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
        self._ring_name = f'ring_{int(self._clock.now().nanoseconds / 1e6)}'

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
        msg.header.stamp = self._clock.now().to_msg()
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
            self._logger.info('Ring passed')
            self._spawn_ring()
            return

        self._last_ring_x = x_local

    def tick(self):
        # Re-spawn si l'anneau n'existe plus.
        if self._ring_enabled and self._ring_name is None:
            self._spawn_ring()

        # Publie la position de l'anneau.
        if self._ring_enabled and self._ring_pose:
            self._publish_ring_center()

        # Verifie le passage dans l'anneau.
        self._check_ring_pass()

        # Autopilot vers l'anneau (prioritaire).
        return self._auto_ring_control()
