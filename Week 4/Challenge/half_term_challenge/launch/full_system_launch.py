from launch import LaunchDescription
from launch.actions import RegisterEventHandler, LogInfo
from launch_ros.actions import Node
from launch.event_handlers import OnProcessStart

def generate_launch_description():
    
    # --- CONFIGURACIÓN ---
    pkg_name = 'half_term_challenge'

    node_debugging = Node(
        package=pkg_name,
        executable='traffic_light_node2',
        name='traffic_debugging_node',
        output='screen'
    )

    node_localisation = Node(
        package=pkg_name,
        executable='localisation',
        name='localisation_node',
        output='screen'
    )

    node_controller = Node(
        package=pkg_name,
        executable='controller_v4',
        name='controller_v4',
        output='screen'
    )

    node_path_gen = Node(
        package=pkg_name,
        executable='path_generator',
        name='path_generator',
        output='screen',
        prefix=["xterm -geometry 80x24 -hold -e"],
    )

    # 2. Nodo de RQT Image View
    node_rqt = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        name='rqt_image_view'
    )

    # 3. Cadena de Eventos (Secuencial)

    # A. Debugging -> Localisation
    event_1 = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=node_debugging,
            on_start=[node_localisation]
        )
    )

    # B. Localisation -> Controller
    event_2 = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=node_localisation,
            on_start=[node_controller]
        )
    )

    # C. Controller -> PathGen (El último paso de la cadena)
    event_3 = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=node_controller,
            on_start=[node_path_gen]
        )
    )

    # 4. Lanzamiento
    return LaunchDescription([
        # Nodos que arrancan de inmediato
        node_debugging,
        #node_rqt,
        
        # Manejadores que disparan la reacción en cadena
        event_1,
        event_2,
        event_3,
        
        LogInfo(msg="Launcher completo...")
    ])
