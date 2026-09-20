from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    # --- CONFIGURACIÓN ---
    pkg_name = 'final_challenge'
    
    # 1. Definición de los Nodos para ejecución simultánea
    camera_publisher = Node(
        package=pkg_name,
        executable='camera_publisher',
        name='camera_publisher',
        output='screen'
    )

    traffic_light_node = Node(
        package=pkg_name,
        executable='traffic_light_node',
        name='traffic_light_node',
        output='screen'
    )

    line_detector = Node(
        package=pkg_name,
        executable='line_detector',
        name='line_detector',
        output='screen'
    )
    
    intersection_detector = Node(
        package=pkg_name,
        executable='intersection_detector',
        name='intersection_detector',
        output='screen'
    )

    line_controller = Node(
        package=pkg_name,
        executable='line_controller_signals',  # Asegúrate de que este nombre coincida con tu setup.py
        name='line_controller_signals',
        output='screen'
    )

    localisation_node = Node(
        package=pkg_name,
        executable='localisation_node',  # Asegúrate de que este nombre coincida con tu setup.py
        name='localisation_node',
        output='screen'
    )

    # Cuando inicie camera_publisher -> Iniciar traffic_light_node
    run_traffic_light = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=camera_publisher,
            on_start=[traffic_light_node],
        )
    )


    run_line_detector = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=traffic_light_node,
            on_start=[line_detector],
        )
    )

    run_intersection_detector = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=line_detector,
            on_start=[intersection_detector],
        )
    )

    run_localisation_node = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=intersection_detector,
            on_start=[localisation_node],
        )
    )

    run_line_controller = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=localisation_node,
            on_start=[line_controller],
        )
    )


    # 2. Lanzamiento directo en paralelo
    return LaunchDescription([
        camera_publisher,
        run_traffic_light,
        run_line_detector,
        run_intersection_detector,
        run_localisation_node,
        run_line_controller
    ])
