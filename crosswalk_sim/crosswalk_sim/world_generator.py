"""Generates the Gazebo (SDF 1.10) world for the crosswalk scenario.

World frame
-----------
* The street runs along the x axis, the crosswalk is centred at x = 0.
* Lane y < 0 carries traffic towards +x, lane y > 0 towards -x
  (right-hand traffic).
* The robot starts on the south sidewalk (y < 0) and has to reach the north
  sidewalk (y > 0).

Everything that moves (pedestrians, cars, lit lamps) is pre-created here as a
pool and animated by the CrosswalkScenario system plugin.
"""

import math
import os
import random
from xml.sax.saxutils import escape

ASSET_DIR = os.environ.get('CROSSWALK_ASSETS', '/opt/crosswalk_assets')

# Car meshes (Gazebo Fuel, OpenRobotics). yaw rotates the mesh so that the car
# front points to +x; length/width/offset describe the resulting footprint
# (offset = footprint centre along +x relative to the model origin).
CAR_TYPES = {
    'hatchback_blue': dict(mesh='hatchback_blue/meshes/hatchback.obj', scale=0.0254,
                           yaw=math.pi / 2, length=4.00, width=1.95, offset=0.34),
    'suv': dict(mesh='suv/meshes/suv.obj', scale=0.06,
                yaw=-math.pi / 2, length=5.02, width=2.30, offset=0.07),
    'hatchback_red': dict(mesh='hatchback_red/meshes/hatchback.obj', scale=0.0254,
                          yaw=math.pi / 2, length=4.00, width=1.95, offset=0.34),
    'pickup': dict(mesh='pickup/meshes/pickup.dae', scale=1.0,
                   yaw=-math.pi / 2, length=5.66, width=2.20, offset=-0.12),
    'prius': dict(mesh='prius/meshes/Hybrid.obj', scale=0.01,
                  yaw=math.pi / 2, length=4.62, width=1.85, offset=-0.04),
    'hatchback': dict(mesh='hatchback/meshes/hatchback.obj', scale=0.0254,
                      yaw=math.pi / 2, length=4.00, width=1.95, offset=0.34),
}
FALLBACK_CAR_COLORS = [
    (0.75, 0.10, 0.10), (0.10, 0.25, 0.70), (0.85, 0.85, 0.85),
    (0.15, 0.15, 0.15), (0.20, 0.50, 0.25), (0.85, 0.65, 0.10),
]


def _asset(rel):
    path = os.path.join(ASSET_DIR, rel)
    return path if os.path.isfile(path) else None


def _f(v):
    return f'{v:.4f}'.rstrip('0').rstrip('.') if isinstance(v, float) else str(v)


def _pose(x=0.0, y=0.0, z=0.0, r=0.0, p=0.0, yaw=0.0):
    return f'<pose>{_f(x)} {_f(y)} {_f(z)} {_f(r)} {_f(p)} {_f(yaw)}</pose>'


def _material(rgb, emissive=None, specular=0.1):
    r, g, b = rgb
    s = (f'<material><ambient>{r} {g} {b} 1</ambient><diffuse>{r} {g} {b} 1</diffuse>'
         f'<specular>{specular} {specular} {specular} 1</specular>')
    if emissive:
        er, eg, eb = emissive
        s += f'<emissive>{er} {eg} {eb} 1</emissive>'
    return s + '</material>'


def _box_visual(name, size, pose, rgb, collision=False, emissive=None):
    sx, sy, sz = size
    geom = f'<geometry><box><size>{_f(sx)} {_f(sy)} {_f(sz)}</size></box></geometry>'
    s = f'<visual name="{name}">{_pose(*pose)}{geom}{_material(rgb, emissive)}</visual>'
    if collision:
        s += f'<collision name="{name}_col">{_pose(*pose)}{geom}</collision>'
    return s


def _cyl_visual(name, radius, length, pose, rgb, collision=False, emissive=None):
    geom = (f'<geometry><cylinder><radius>{_f(radius)}</radius>'
            f'<length>{_f(length)}</length></cylinder></geometry>')
    s = f'<visual name="{name}">{_pose(*pose)}{geom}{_material(rgb, emissive)}</visual>'
    if collision:
        s += f'<collision name="{name}_col">{_pose(*pose)}{geom}</collision>'
    return s


def _static_model(name, body, pose=(0, 0, 0, 0, 0, 0)):
    return (f'<model name="{name}"><static>true</static>{_pose(*pose)}'
            f'<link name="link">{body}</link></model>\n')


def _local_to_world(base, local):
    """Compose planar pose base=(x,y,z,yaw) with local offset (dx,dy,dz)."""
    bx, by, bz, byaw = base
    dx, dy, dz = local
    c, s = math.cos(byaw), math.sin(byaw)
    return bx + c * dx - s * dy, by + s * dx + c * dy, bz + dz


class WorldBuilder:

    def __init__(self, cfg):
        self.cfg = cfg
        w = cfg['world']
        self.lane_w = float(w['lane_width'])
        self.side_w = float(w['sidewalk_width'])
        self.cw_w = float(w['crosswalk_width'])
        self.stop_off = float(w['stop_line_offset'])
        self.road_len = float(w['road_length'])
        self.R = self.lane_w                      # road half width (1 lane / direction)
        self.cw_half = self.cw_w / 2.0
        self.stop_x = self.cw_half + self.stop_off
        self.lamps = []                           # (name, group)
        self.cars = []                            # (name, type, length, width, offset)
        self.peds = []
        self.parts = []

    # ------------------------------------------------------------ static scene
    def ground(self):
        size = max(self.road_len + 100, 300)
        self.parts.append(f'''
    <model name="ground"><static>true</static>
      <link name="link">
        <collision name="col"><geometry><plane><normal>0 0 1</normal><size>{size} {size}</size></plane></geometry>
          <surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface></collision>
        <visual name="grass"><geometry><plane><normal>0 0 1</normal><size>{size} {size}</size></plane></geometry>
          {_material((0.30, 0.45, 0.22), specular=0.0)}</visual>
      </link>
    </model>\n''')

    def street(self):
        R, L, sw = self.R, self.road_len, self.side_w
        asphalt, pave, curb, white = (0.17, 0.17, 0.18), (0.62, 0.61, 0.58), (0.78, 0.78, 0.76), (0.95, 0.95, 0.95)
        v = []
        v.append(_box_visual('asphalt', (L, 2 * R, 0.01), (0, 0, -0.0035), asphalt))
        for side, name in ((1, 'north'), (-1, 'south')):
            v.append(_box_visual(f'sidewalk_{name}', (L, sw, 0.01),
                                 (0, side * (R + sw / 2), -0.0030), pave))
            # paving joints every 2 m make the sidewalk readable in the camera
            for k in range(int(L / 2)):
                x = -L / 2 + 1 + 2 * k
                if abs(x) < 60:
                    v.append(_box_visual(f'joint_{name}_{k}', (0.03, sw, 0.002),
                                         (x, side * (R + sw / 2), 0.0021), (0.5, 0.5, 0.48)))
        # zebra stripes (bars parallel to the traffic)
        n = int((2 * R - 0.5) // 1.0) + 1
        y0 = -(n - 1) / 2.0
        for i in range(n):
            v.append(_box_visual(f'zebra_{i}', (self.cw_w, 0.5, 0.004),
                                 (0, y0 + i, 0.0015), white))
        # stop lines
        v.append(_box_visual('stop_line_e', (0.4, R - 0.1, 0.004),
                             (-self.stop_x - 0.2, -R / 2, 0.0015), white))
        v.append(_box_visual('stop_line_w', (0.4, R - 0.1, 0.004),
                             (self.stop_x + 0.2, R / 2, 0.0015), white))
        # centre line: solid near the crossing, dashed further away
        solid_to = self.stop_x + 15.0
        for sgn in (-1, 1):
            v.append(_box_visual(f'centre_solid_{sgn}', (solid_to - self.cw_half - 0.6, 0.12, 0.004),
                                 (sgn * (self.cw_half + 0.6 + solid_to) / 2, 0, 0.0015), white))
            x = solid_to + 2.0
            k = 0
            while x < L / 2 - 2:
                v.append(_box_visual(f'centre_dash_{sgn}_{k}', (3.0, 0.12, 0.004),
                                     (sgn * (x + 1.5), 0, 0.0015), white))
                x += 6.0
                k += 1
        street = ''.join(v)
        self.parts.append(_static_model('street', street))

        # curbs (with collision) except for the lowered section at the crosswalk
        ch = float(self.cfg['world']['curb_height'])
        gap = self.cw_half + 0.5
        seg = (L / 2 - gap)
        c = []
        for side, name in ((1, 'n'), (-1, 's')):
            for sx in (-1, 1):
                c.append(_box_visual(f'curb_{name}_{sx}', (seg, 0.15, ch),
                                     (sx * (gap + seg / 2), side * (R + 0.075), ch / 2),
                                     curb, collision=ch > 0.001))
            c.append(_box_visual(f'ramp_{name}', (2 * gap, 0.15, 0.006),
                                 (0, side * (R + 0.075), 0.0), curb))
        self.parts.append(_static_model('curbs', ''.join(c)))

    def buildings(self):
        rnd = random.Random(7)                   # fixed layout, independent of the scenario seed
        palette = [(0.78, 0.72, 0.62), (0.66, 0.42, 0.34), (0.85, 0.83, 0.78),
                   (0.55, 0.58, 0.62), (0.80, 0.68, 0.45), (0.62, 0.66, 0.58)]
        y_in = self.R + self.side_w + 2.5
        b = []
        for side, name in ((1, 'n'), (-1, 's')):
            x = -75.0
            k = 0
            while x < 75.0:
                w = rnd.uniform(10, 18)
                h = rnd.uniform(7, 19)
                d = rnd.uniform(8, 12)
                col = rnd.choice(palette)
                cx = x + w / 2
                cy = side * (y_in + d / 2)
                b.append(_box_visual(f'bld_{name}_{k}', (w, d, h), (cx, cy, h / 2), col, collision=True))
                # darker window bands on the street facade
                face = side * y_in - side * 0.03
                for fl in range(1, int(h / 3.2)):
                    b.append(_box_visual(f'win_{name}_{k}_{fl}', (w - 1.2, 0.04, 1.2),
                                         (cx, face, fl * 3.2 + 0.4), (0.18, 0.22, 0.28)))
                # shop front
                b.append(_box_visual(f'shop_{name}_{k}', (w - 1.0, 0.04, 2.2),
                                     (cx, face, 1.4), (0.25, 0.30, 0.33)))
                x += w + rnd.uniform(3, 7)
                k += 1
        self.parts.append(_static_model('buildings', ''.join(b)))

    def vegetation(self):
        tree = _asset('oak_tree/meshes/oak_tree.dae')
        lamp = _asset('lamp_post/meshes/lamp_post.dae')
        y_tree = self.R + self.side_w + 1.2
        k = 0
        for side in (1, -1):
            for x in (-44, -30, -17, 17, 30, 44):
                xx = x + (3 if side > 0 else -2)
                name = f'tree_{k}'
                k += 1
                if tree:
                    tex = os.path.join(ASSET_DIR, 'oak_tree', 'materials', 'textures')
                    body = ''
                    for sub, png, double in (('Branch', 'branch_diffuse.png', 'true'),
                                             ('Bark', 'bark_diffuse.png', 'false')):
                        body += (f'<visual name="{sub.lower()}"><geometry><mesh><uri>file://{tree}</uri>'
                                 f'<submesh><name>{sub}</name></submesh>'
                                 f'<scale>0.55 0.55 0.55</scale></mesh></geometry>'
                                 f'<material><double_sided>{double}</double_sided><diffuse>1 1 1</diffuse>'
                                 f'<pbr><metal><albedo_map>{tex}/{png}</albedo_map></metal></pbr>'
                                 f'</material></visual>')
                    body += (
                            f'<collision name="c">{_pose(0, 0, 1.5)}<geometry><cylinder><radius>0.25</radius>'
                            f'<length>3</length></cylinder></geometry></collision>')
                else:
                    body = (_cyl_visual('trunk', 0.2, 3.0, (0, 0, 1.5), (0.35, 0.25, 0.15), collision=True) +
                            f'<visual name="crown">{_pose(0, 0, 4.2)}<geometry><sphere><radius>2.0</radius>'
                            f'</sphere></geometry>{_material((0.2, 0.45, 0.18))}</visual>')
                self.parts.append(_static_model(name, body, (xx, side * y_tree, 0, 0, 0, 0)))
        k = 0
        for side in (1, -1):
            for x in (-38, -12, 12, 38):
                xx = x + (5 if side > 0 else 0)
                if lamp:
                    body = (f'<visual name="v"><geometry><mesh><uri>file://{lamp}</uri>'
                            f'<scale>3 3 3</scale></mesh></geometry></visual>'
                            f'<collision name="c">{_pose(0, 0, 2)}<geometry><cylinder><radius>0.12</radius>'
                            f'<length>4</length></cylinder></geometry></collision>')
                    yaw = -math.pi / 2 if side > 0 else math.pi / 2
                else:
                    body = _cyl_visual('pole', 0.08, 6.0, (0, 0, 3.0), (0.25, 0.25, 0.27), collision=True)
                    yaw = 0
                self.parts.append(_static_model(f'lamp_post_{k}', body,
                                                (xx, side * (self.R + self.side_w - 0.4), 0, 0, 0, yaw)))
                k += 1

    def overview_camera(self):
        """Static camera overlooking the crossing (videos, debugging): /overview_camera/image."""
        oc = self.cfg['world'].get('overview_camera_pose', [-13.0, -6.5, 7.0, 0.0, 0.45, 0.42])
        self.parts.append(f'''
    <model name="overview_camera"><static>true</static>{_pose(*[float(v) for v in oc])}
      <link name="link">
        <sensor name="overview" type="camera">
          <always_on>true</always_on><update_rate>{float(self.cfg['world'].get('overview_camera_rate', 5.0))}</update_rate>
          <topic>/overview_camera/image</topic>
          <camera><horizontal_fov>1.25</horizontal_fov>
            <image><width>1280</width><height>720</height><format>R8G8B8</format></image>
            <clip><near>0.1</near><far>300</far></clip></camera>
        </sensor>
      </link>
    </model>\n''')

    # ---------------------------------------------------------- traffic lights
    def _signal(self, name, x, y, face_yaw, lamps, head_size, pole_h):
        """A pole with a signal head facing face_yaw; lamps=[(group, z, rgb_off, rgb_on, radius)]."""
        dark = (0.08, 0.08, 0.09)
        hd, hw, hh = head_size
        head_z = sum(z for _, z, *_ in lamps) / len(lamps)
        base = (x, y, 0.0, face_yaw)
        body = _cyl_visual('pole', 0.06, pole_h, (0, 0, pole_h / 2), (0.30, 0.31, 0.33), collision=True)
        # head is mounted in front of the pole
        hx = 0.06 + hd / 2
        body += _box_visual('head', (hd, hw, hh), (hx, 0, head_z), dark)
        body += _box_visual('back_plate', (0.01, hw + 0.12, hh + 0.12), (hx - hd / 2 + 0.005, 0, head_z), dark)
        for i, (group, z, off, on, r) in enumerate(lamps):
            # lenses (dark when off)
            body += _cyl_visual(f'lens_{i}', r, 0.01, (hx + hd / 2 + 0.005, 0, z, 0, math.pi / 2, 0), off)
            # small visor
            body += _box_visual(f'visor_{i}', (0.12, 2 * r + 0.04, 0.01),
                                (hx + hd / 2 + 0.06, 0, z + r + 0.01), dark)
            # the lit lamp: separate model moved by the plugin
            lx, ly, lz = _local_to_world(base, (hx + hd / 2 + 0.013, 0, z))
            lname = f'lamp_{name}_{group}'
            lit = _cyl_visual('lit', r * 1.02, 0.008, (0, 0, 0, 0, math.pi / 2, 0),
                              on, emissive=on)
            self.parts.append(_static_model(lname, lit, (lx, ly, lz, 0, 0, face_yaw)))
            self.lamps.append((lname, group))
        self.parts.append(_static_model(f'signal_{name}', body, (x, y, 0, 0, 0, face_yaw)))

    def traffic_lights(self):
        R, cwh = self.R, self.cw_half
        red_off, red_on = (0.22, 0.03, 0.03), (1.0, 0.08, 0.05)
        yel_off, yel_on = (0.25, 0.18, 0.02), (1.0, 0.75, 0.05)
        grn_off, grn_on = (0.02, 0.17, 0.06), (0.10, 1.0, 0.35)
        ped = [('ped_red', 2.45, red_off, red_on, 0.11), ('ped_green', 2.15, grn_off, grn_on, 0.11)]
        car = [('car_red', 3.05, red_off, red_on, 0.10), ('car_yellow', 2.80, yel_off, yel_on, 0.10),
               ('car_green', 2.55, grn_off, grn_on, 0.10)]
        # pedestrian heads: on the far side of the street, facing the waiting people
        self._signal('ped_north', -(cwh + 0.6), R + 0.5, -math.pi / 2, ped, (0.16, 0.30, 0.64), 3.0)
        self._signal('ped_south', cwh + 0.6, -(R + 0.5), math.pi / 2, ped, (0.16, 0.30, 0.64), 3.0)
        # vehicle heads: right side of each approach, facing the oncoming cars
        self._signal('car_east', -(self.stop_x + 1.0), -(R + 0.6), math.pi, car, (0.18, 0.30, 0.82), 3.5)
        self._signal('car_west', self.stop_x + 1.0, R + 0.6, 0.0, car, (0.18, 0.30, 0.82), 3.5)

    # ------------------------------------------------------------------ pools
    def pedestrians(self):
        n = int(self.cfg['pedestrians']['max_count'])
        skin = _asset('actor/meshes/walk.dae')
        stand = _asset('actor/meshes/stand.dae')
        if not skin:
            raise RuntimeError(
                f'Pedestrian mesh not found in {ASSET_DIR}/actor - run '
                f'`ros2 run crosswalk_sim fetch_assets.sh {ASSET_DIR}` or set CROSSWALK_ASSETS.')
        stand = stand or skin
        for i in range(n):
            name = f'ped_{i}'
            self.peds.append(name)
            self.parts.append(f'''
    <actor name="{name}">
      <pose>0 0 1.0 0 0 0</pose>
      <skin><filename>file://{skin}</filename><scale>1.0</scale></skin>
      <animation name="walk"><filename>file://{skin}</filename><interpolate_x>true</interpolate_x></animation>
      <animation name="stand"><filename>file://{stand}</filename><interpolate_x>true</interpolate_x></animation>
    </actor>\n''')

    def car_pool(self):
        n = int(self.cfg['cars']['max_count'])
        types = list(CAR_TYPES.keys())
        for i in range(n):
            key = types[i % len(types)]
            t = CAR_TYPES[key]
            mesh = _asset(t['mesh'])
            name = f'car_{i}'
            if mesh:
                s = t['scale']
                body = (f'<visual name="body">{_pose(0, 0, 0, 0, 0, t["yaw"])}<geometry><mesh>'
                        f'<uri>file://{mesh}</uri><scale>{s} {s} {s}</scale></mesh></geometry></visual>')
                length, width, offset = t['length'], t['width'], t['offset']
            else:
                key = 'box_car'
                length, width, offset = 4.3, 1.8, 0.0
                col = FALLBACK_CAR_COLORS[i % len(FALLBACK_CAR_COLORS)]
                body = _box_visual('body', (4.3, 1.8, 0.75), (0, 0, 0.62), col)
                body += _box_visual('cabin', (2.3, 1.6, 0.6), (-0.3, 0, 1.28), (0.2, 0.25, 0.3))
                for wi, (wx, wy) in enumerate(((1.35, 0.8), (1.35, -0.8), (-1.35, 0.8), (-1.35, -0.8))):
                    body += _cyl_visual(f'wheel_{wi}', 0.33, 0.22, (wx, wy, 0.33, math.pi / 2, 0, 0),
                                        (0.05, 0.05, 0.05))
            self.cars.append((name, key, length, width, offset))
            self.parts.append(_static_model(name, body, (0, 0, -100 - i, 0, 0, 0)))

    # ----------------------------------------------------------------- plugin
    def plugin(self):
        cfg = self.cfg

        def group(name, keys):
            sec = cfg.get(name, {})
            inner = ''
            for k in keys:
                if k not in sec:
                    continue
                val = sec[k]
                if isinstance(val, bool):
                    val = 'true' if val else 'false'
                elif isinstance(val, (list, tuple)):
                    val = ' '.join(_f(float(x)) for x in val)
                inner += f'<{k}>{escape(str(val))}</{k}>'
            return f'<{name}>{inner}</{name}>'

        s = ['<plugin filename="CrosswalkScenario" name="crosswalk_sim::CrosswalkScenario">']
        s.append(f'<seed>{int(cfg.get("seed", 0))}</seed><warmup>{float(cfg.get("warmup", 30.0))}</warmup>')
        s.append(group('world', ['lane_width', 'sidewalk_width', 'crosswalk_width',
                                 'stop_line_offset', 'road_length']))
        s.append(group('traffic_light', ['car_green', 'car_yellow', 'all_red', 'ped_green',
                                         'ped_green_blink', 'blink_period', 'car_red_yellow',
                                         'start_phase']))
        s.append(group('pedestrians', ['crossing_rate', 'sidewalk_rate', 'speed_min', 'speed_max',
                                       'red_crossing_probability', 'avoid_robot', 'spawn_distance',
                                       'animation_factor']))
        s.append(group('cars', ['rate', 'speed_min', 'speed_max', 'yield_to_robot',
                                'yield_to_pedestrians']))
        s.append(group('red_runner', ['probability', 'window', 'speed_min', 'speed_max']))
        s.append(group('robot', ['name', 'spawn', 'footprint', 'goal']))
        s.append(group('referee', ['near_miss_distance', 'pause_on_collision']))
        for name in self.peds:
            s.append(f'<pedestrian name="{name}"/>')
        for name, key, length, width, offset in self.cars:
            s.append(f'<car name="{name}" type="{key}" length="{length}" width="{width}" offset="{offset}"/>')
        for name, grp in self.lamps:
            s.append(f'<lamp name="{name}" group="{grp}"/>')
        s.append('</plugin>')
        return '\n      '.join(s)

    # ------------------------------------------------------------------- gui
    def gui(self):
        def floating(name, filename, extra=''):
            return (f'<plugin filename="{filename}" name="{name}"><gz-gui>'
                    '<property key="resizable" type="bool">false</property>'
                    '<property key="width" type="double">5</property>'
                    '<property key="height" type="double">5</property>'
                    '<property key="state" type="string">floating</property>'
                    '<property key="showTitleBar" type="bool">false</property>'
                    f'</gz-gui>{extra}</plugin>')
        return f'''
    <gui fullscreen="0">
      <plugin filename="MinimalScene" name="3D View">
        <gz-gui><title>3D View</title><property type="bool" key="showTitleBar">false</property>
          <property type="string" key="state">docked</property></gz-gui>
        <engine>ogre2</engine><scene>scene</scene>
        <ambient_light>0.5 0.5 0.5</ambient_light><background_color>0.62 0.77 0.92</background_color>
        <camera_pose>-13 -6.5 7 0 0.45 0.42</camera_pose>
        <camera_clip><near>0.1</near><far>1000</far></camera_clip>
      </plugin>
      {floating('Entity context menu', 'EntityContextMenuPlugin')}
      {floating('Scene Manager', 'GzSceneManager')}
      {floating('Interactive view control', 'InteractiveViewControl')}
      {floating('Camera Tracking', 'CameraTracking')}
      {floating('Marker manager', 'MarkerManager')}
      {floating('Select Entities', 'SelectEntities')}
      {floating('Visualization Capabilities', 'VisualizationCapabilities')}
      <plugin filename="WorldControl" name="World control">
        <gz-gui><title>World control</title><property type="bool" key="showTitleBar">false</property>
          <property type="bool" key="resizable">false</property><property type="double" key="height">72</property>
          <property type="double" key="z">1</property><property type="string" key="state">floating</property>
          <anchors target="3D View"><line own="left" target="left"/><line own="bottom" target="bottom"/></anchors>
        </gz-gui>
        <play_pause>true</play_pause><step>true</step><start_paused>false</start_paused><use_event>true</use_event>
      </plugin>
      <plugin filename="WorldStats" name="World stats">
        <gz-gui><title>World stats</title><property type="bool" key="showTitleBar">false</property>
          <property type="bool" key="resizable">false</property><property type="double" key="height">110</property>
          <property type="double" key="width">290</property><property type="double" key="z">1</property>
          <property type="string" key="state">floating</property>
          <anchors target="3D View"><line own="right" target="right"/><line own="bottom" target="bottom"/></anchors>
        </gz-gui>
        <sim_time>true</sim_time><real_time>true</real_time><real_time_factor>true</real_time_factor><iterations>true</iterations>
      </plugin>
      <plugin filename="ImageDisplay" name="Robot camera">
        <gz-gui><title>Robot camera (/camera/image_raw)</title><property type="string" key="state">docked</property></gz-gui>
        <topic>/camera/image_raw</topic><topic_picker>false</topic_picker>
      </plugin>
      <plugin filename="EntityTree" name="Entity tree">
        <gz-gui><property type="string" key="state">docked_collapsed</property></gz-gui>
      </plugin>
      <plugin filename="ComponentInspector" name="Component inspector">
        <gz-gui><property type="string" key="state">docked_collapsed</property></gz-gui>
      </plugin>
    </gui>'''

    # ----------------------------------------------------------------- build
    def build(self):
        w = self.cfg['world']
        self.ground()
        self.street()
        if w.get('buildings', True):
            self.buildings()
        if w.get('trees', True):
            self.vegetation()
        self.traffic_lights()
        if w.get('overview_camera', False):
            self.overview_camera()
        self.pedestrians()
        self.car_pool()
        shadows = 'true' if w.get('shadows', True) else 'false'
        world_name = self.cfg.get('world_name', 'crosswalk')
        return f'''<?xml version="1.0"?>
<!-- Generated by crosswalk_sim/world_generator.py - do not edit, change the scenario YAML instead. -->
<sdf version="1.10">
  <world name="{world_name}">
    <physics name="2ms" type="ignored">
      <max_step_size>0.002</max_step_size>
      <real_time_factor>{float(w.get('real_time_factor', 1.0))}</real_time_factor>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
      {self.plugin()}
{self.gui()}
    <scene>
      <ambient>0.55 0.55 0.55 1</ambient>
      <background>0.62 0.77 0.92 1</background>
      <shadows>{shadows}</shadows>
      <grid>false</grid>
      <sky></sky>
    </scene>
    <light type="directional" name="sun">
      <cast_shadows>{shadows}</cast_shadows>
      <pose>0 0 30 0 0 0</pose>
      <diffuse>0.9 0.88 0.82 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.45 0.3 -0.85</direction>
    </light>
    <light type="directional" name="fill">
      <cast_shadows>false</cast_shadows>
      <diffuse>0.25 0.25 0.28 1</diffuse>
      <specular>0 0 0 1</specular>
      <direction>0.4 -0.5 -0.6</direction>
    </light>
{''.join(self.parts)}
  </world>
</sdf>
'''


def generate_world(cfg, path):
    sdf = WorldBuilder(cfg).build()
    with open(path, 'w') as f:
        f.write(sdf)
    return path
