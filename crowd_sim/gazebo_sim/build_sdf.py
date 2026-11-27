import yaml
from string import Template

# Static red cylinder matching person_sdf() in agent.cpp (radius 0.2, height 1),
# placed at the origin with its link at z = height / 2 so it rests on the ground.
TEST_PERSON_SDF = """      <model name="test_person">
        <static>true</static>
        <pose>0 0 0 0 0 0</pose>
        <link name="body">
          <pose>0 0 0.5 0 0 0</pose>
          <collision name="collision">
            <geometry><cylinder><radius>0.2</radius><length>1</length></cylinder></geometry>
          </collision>
          <visual name="visual">
            <geometry><cylinder><radius>0.2</radius><length>1</length></cylinder></geometry>
            <material><ambient>1 0 0 1</ambient><diffuse>1 0 0 1</diffuse></material>
          </visual>
        </link>
      </model>
"""


def build_sdf(world_yaml_path, template_path, output_path):
    """Fill sim_world.sdf.template with the wall geometry from world.yaml.

    Reads the `world:` block of `world_yaml_path`:
        x_min, x_max, y_min, y_max  - inner faces of the four walls
        wall_thickness, wall_height - wall box dimensions
        build_test_person           - if true, add a static red test cylinder at the origin
    and writes the substituted SDF to `output_path`.
    """
    with open(world_yaml_path) as f:
        world = yaml.safe_load(f)['world']

    init_pos = [float(v) for v in world['initial_robot_position']]
    init_yaw = float(world['initial_robot_yaw'])
    x_min = float(world['x_min'])
    x_max = float(world['x_max'])
    y_min = float(world['y_min'])
    y_max = float(world['y_max'])
    t = float(world['wall_thickness'])
    h = float(world['wall_height'])
    build_test_person = bool(world.get('build_test_person', False))

    width = x_max - x_min    # extent along x (north/south walls)
    length = y_max - y_min   # extent along y (east/west walls)
    x_mid = (x_min + x_max) / 2
    y_mid = (y_min + y_max) / 2
    z = h / 2                # box pose is at its center

    test_person = TEST_PERSON_SDF if build_test_person else ''

    # Walls are pushed outward by half their thickness so the inner face
    # sits exactly on the x_min/x_max/y_min/y_max boundary.
    with open(template_path) as f:
        sdf = Template(f.read()).substitute(
            north_pose=f"{x_mid} {y_max + t / 2} {z} 0 0 0",
            south_pose=f"{x_mid} {y_min - t / 2} {z} 0 0 0",
            east_pose=f"{x_max + t / 2} {y_mid} {z} 0 0 0",
            west_pose=f"{x_min - t / 2} {y_mid} {z} 0 0 0",
            north_south_size=f"{width + 2 * t} {t} {h}",
            east_west_size=f"{t} {length} {h}",
            initial_robot_pose=f'{init_pos[0]} {init_pos[1]} 0.105 0 0 {init_yaw}',
            test_person=test_person,
        )

    with open(output_path, 'w') as f:
        f.write(sdf)
    return output_path
