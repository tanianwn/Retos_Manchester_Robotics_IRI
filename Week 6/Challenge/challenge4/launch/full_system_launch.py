from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart

def generate_launch_description():
    
    pkg_name = 'challenge4'
    
    #Definición de los Nodos
    camera_publisher = Node(
        package=pkg_name,
        executable='camera_publisher',
        name='camera_publisher'
    )

    traffic_light_node = Node(
        package=pkg_name,
        executable='traffic_light_node',
        name='traffic_light_node'
    )

    line_detector = Node(
        package=pkg_name,
        executable='line_detector',
        name='line_detector'
    )

    line_controller = Node(
        package=pkg_name,
        executable='line_controller',
        name='line_controller'
    )

    #Encadenamiento de eventos 

    # Cuando inicie camera_publisher, iniciar traffic_light_node
    run_traffic_light = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=camera_publisher,
            on_start=[traffic_light_node],
        )
    )

    #Cuando inicie traffic_light_node, iniciar line_detector
    run_line_detector = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=traffic_light_node,
            on_start=[line_detector],
        )
    )

    #Cuando inicie line_detector, iniciar line_controller
    run_line_controller = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=line_detector,
            on_start=[line_controller],
        )
    )

    #Lanzamiento
    #Solo añadimos el primer nodo y los manejadores de eventos
    return LaunchDescription([
        camera_publisher,
        run_traffic_light,
        run_line_detector,
        run_line_controller
    ])
