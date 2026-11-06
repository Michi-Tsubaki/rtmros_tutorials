"""One-arm ros2_control transport; defaults to inactive commissioning mode."""
from pathlib import Path
import tempfile
import xml.etree.ElementTree as ET
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def setup(context):
    arm = LaunchConfiguration('arm').perform(context)
    if arm not in ('larm', 'rarm'):
        raise ValueError('arm must be larm or rarm')
    side = 'left' if arm == 'larm' else 'right'
    frame = arm.upper() + '_FTSENSOR_SENSOR_Link'
    share = Path(get_package_share_directory('nextage_tutorials'))
    root = ET.parse(share / 'urdf/JSKNextageOpenGripper.urdf').getroot()
    system = ET.SubElement(root, 'ros2_control', name=arm+'_topic_system', type='system')
    hardware = ET.SubElement(system, 'hardware')
    ET.SubElement(hardware, 'plugin').text = 'nextage_tutorials/TopicSystem'
    for key, value in {'sensor_frame': frame, 'command_topic': '/fullbody_controller/command',
                       'wrench_topic': f'/{side}_ft/force_torque_compensated'}.items():
        ET.SubElement(hardware, 'param', name=key).text = value
    for i in range(6):
        name = f'{arm.upper()}_JOINT{i}'
        limit = root.find(f"joint[@name='{name}']/limit")
        joint = ET.SubElement(system, 'joint', name=name)
        for bound in ('lower', 'upper'):
            ET.SubElement(joint, 'param', name=bound).text = limit.attrib[bound]
        ET.SubElement(joint, 'command_interface', name='position')
        ET.SubElement(joint, 'state_interface', name='position')
    sensor = ET.SubElement(system, 'sensor', name=side+'_ft_sensor')
    for axis in ('force.x','force.y','force.z','torque.x','torque.y','torque.z'):
        ET.SubElement(sensor, 'state_interface', name=axis)
    description = ET.tostring(root, encoding='unicode')
    manager = '/admittance/controller_manager'
    controller = arm+'_admittance_controller'
    config = yaml.safe_load((share / 'config/nextage_admittance.yaml').read_text())
    params = config['/**/'+controller]['ros__parameters']
    # The transport consumes fully gravity/bias-compensated data.
    params['gravity_compensation']['CoG']['force'] = 0.0
    # Start with just one translational degree of freedom.
    params['admittance']['selected_axes'] = [True, False, False, False, False, False]
    # Resolve fixed transforms in the six-joint chain; chest is monitored stationary.
    params['fixed_world_frame']['frame']['id'] = 'CHEST_JOINT0_Link'
    with tempfile.NamedTemporaryFile(mode='w', prefix=arm+'_admittance_', suffix='.yaml', delete=False) as stream:
        yaml.safe_dump({'/**/'+controller: {'ros__parameters': params}}, stream)
        param_path = stream.name
    inactive = LaunchConfiguration('inactive').perform(context).lower() == 'true'
    return [
        Node(package='robot_state_publisher', executable='robot_state_publisher', namespace='admittance',
             name='control_description', output='screen',
             parameters=[{'robot_description': ParameterValue(description, value_type=str)}],
             remappings=[('joint_states', '/joint_states'), ('/tf', '/admittance/model_tf'),
                         ('/tf_static', '/admittance/model_tf_static')]),
        Node(package='controller_manager', executable='ros2_control_node', namespace='admittance',
             output='screen', parameters=[{'update_rate': 50,
                 controller+'.type': 'admittance_controller/AdmittanceController'}]),
        Node(package='controller_manager', executable='spawner', output='screen',
             arguments=[controller, '-c', manager, '--param-file', str(param_path),
                        '--controller-manager-timeout', '30'] + (['--inactive'] if inactive else [])),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('arm', default_value='larm', choices=['larm','rarm']),
        DeclareLaunchArgument('inactive', default_value='true', choices=['true','false']),
        OpaqueFunction(function=setup),
    ])
