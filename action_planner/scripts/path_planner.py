#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionClient
from std_srvs.srv import Trigger
from std_msgs.msg import String
from harpia_msgs.srv import PlanPath
import json
import numpy as np


class PathPlannerNode(Node):
    def __init__(self):
        super().__init__('path_planner_node')
        
        # Definição do mapa 2D (grid 10x10 como exemplo)
        # 0 = espaço livre, 1 = obstáculo
        self.grid_map = np.array([          # y
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0], # 0
            [0, 0, 0, 1, 1, 1, 0, 0, 0, 0], # 1
            [0, 0, 0, 1, 0, 0, 0, 0, 0, 0], # 2
            [0, 0, 0, 1, 0, 0, 0, 0, 0, 0], # 3
            [0, 0, 0, 0, 0, 0, 1, 1, 0, 0], # 4
            [0, 0, 0, 0, 0, 0, 1, 1, 0, 0], # 5
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0], # 6
            [0, 0, 1, 1, 0, 0, 0, 0, 0, 0], # 7
            [0, 0, 1, 1, 0, 0, 0, 0, 0, 0], # 8
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]  # 9
        #x-> 0, 1, 2, 3, 4, 5, 6, 7, 8, 9
        ])
        
        # Mapeamento de locais para coordenadas (x, y) no grid
        self.location_coords = {
            'base1': (1, 1),    # Base no canto superior esquerdo
            'casaA': (8, 2),    # Casa A no lado direito superior
            'casaB': (3, 8),    # Casa B no lado esquerdo inferior
            'casaC': (8, 8)     # Casa C no canto inferior direito
        }
        
        # Create service server
        self.plan_path_service = self.create_service(
            PlanPath,
            'path_planner/plan_path',
            self.plan_path_callback
        )
        
        self.get_logger().info('Path Planner Node Initialized')
        self.get_logger().info(f'Grid map size: {self.grid_map.shape}')
        self.get_logger().info(f'Locations: {list(self.location_coords.keys())}')
        self.get_logger().info('Service "path_planner/plan_path" ready')

    def plan_path_callback(self, request, response):
        """
        Callback para o serviço de planejamento de caminho.
        
        Args:
            request: PlanPath.Request com origem e destino
            response: PlanPath.Response com lista de waypoints
        """
        origem = request.origem
        destino = request.destino
        
        self.get_logger().info(f'Received path planning request: {origem} -> {destino}')
        
        try:
            # TODO: Implementar algoritmo de planejamento de caminho (A*, Dijkstra, etc.)
            # Por enquanto, retorna caminho direto (origem -> destino)
            waypoints = [self.location_coords[origem], self.location_coords[destino]]
            
            response.success = True
            response.waypoints = waypoints
            response.message = f'Path planned successfully with {len(waypoints)} waypoints'
            
            self.get_logger().info(f'Path planned: {waypoints}')
            
        except Exception as e:
            response.success = False
            response.waypoints = []
            response.message = f'Path planning failed: {str(e)}'
            self.get_logger().error(f'Path planning error: {e}')
        
        return response


def main(args=None):
    rclpy.init(args=args)
    
    path_planner_node = PathPlannerNode()
    
    executor = MultiThreadedExecutor()
    executor.add_node(path_planner_node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        path_planner_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()