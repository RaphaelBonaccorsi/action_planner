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
import heapq

# Importa o módulo de visualização
from path_visualizer import PathVisualizer, PathVisualizerServer


class PathPlannerNode(Node):
    def __init__(self):
        super().__init__('path_planner_node')
        
        # Definição do mapa 2D (grid 15x15 ampliado)
        # 0 = espaço livre, 1 = obstáculo
        self.grid_map = np.array([          # y
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], # 0
            [0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0], # 1
            [0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0], # 2
            [0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0], # 3
            [0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 1, 0, 1, 0, 0], # 4
            [0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 1, 1, 1, 0, 0], # 5
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], # 6
            [0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], # 7
            [0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], # 8
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], # 9
            [0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1], # 10 - Barreira horizontal
            [0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1], # 11 - Área isolada
            [0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1], # 12 - Área isolada
            [0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1], # 13 - Barreira horizontal
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]  # 14
        #x-> 0, 1, 2, 3, 4, 5, 6, 7, 8, 9,10,11,12,13,14
        ])
        
        # Mapeamento de locais para coordenadas (x, y) no grid
        self.location_coords = {
            'base1': (1, 1),     # Base no canto superior esquerdo
            'casaa': (13, 2),    # Casa A no lado direito superior
            'casab': (3, 14),    # Casa B no lado esquerdo inferior
            'casac': (13, 8),    # Casa C no lado direito meio
            'casad': (11, 12),   # Casa D - ISOLADA (dentro da área cercada)
            'deposito': (1, 7)   # Depósito no lado esquerdo
        }
        
        # Inicializa o visualizador (módulo separado)
        self.visualizer = PathVisualizer(self.grid_map, self.location_coords)
        self.visualizer_server = PathVisualizerServer(self.visualizer, port=5007, host='0.0.0.0')
        
        # Inicia o servidor web em thread separada
        if self.visualizer_server.start():
            self.get_logger().info('🌐 Path Visualizer Server started at http://0.0.0.0:5007')
            self.get_logger().info('   Access from host: http://localhost:5007')
        else:
            self.get_logger().warn('Failed to start Path Visualizer Server')
        
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

    def dijkstra(self, start, goal):
        """
        Implementa o algoritmo de Dijkstra para planejamento de caminho.
        
        Args:
            start: tupla (x, y) da posição inicial
            goal: tupla (x, y) da posição final
            
        Returns:
            Lista de coordenadas (x, y) representando o caminho, ou None se não houver caminho
        """
        rows, cols = self.grid_map.shape
        
        # Verifica se start e goal são válidos
        if not (0 <= start[0] < cols and 0 <= start[1] < rows):
            self.get_logger().error(f'Start position {start} is out of bounds')
            return None
        if not (0 <= goal[0] < cols and 0 <= goal[1] < rows):
            self.get_logger().error(f'Goal position {goal} is out of bounds')
            return None
        if self.grid_map[start[1], start[0]] == 1:
            self.get_logger().error(f'Start position {start} is an obstacle')
            return None
        if self.grid_map[goal[1], goal[0]] == 1:
            self.get_logger().error(f'Goal position {goal} is an obstacle')
            return None
        
        # Direções possíveis: cima, baixo, esquerda, direita, e diagonais
        directions = [
            (0, 1), (1, 0), (0, -1), (-1, 0),  # cardinal
            (1, 1), (1, -1), (-1, 1), (-1, -1)  # diagonal
        ]
        
        # Custos: 1.0 para movimentos cardinais, sqrt(2) para diagonais
        def get_cost(dx, dy):
            return 1.0 if (dx == 0 or dy == 0) else 1.414
        
        # Priority queue: (custo, (x, y))
        pq = [(0, start)]
        # Dicionário de custos
        costs = {start: 0}
        # Dicionário de predecessores para reconstruir o caminho
        came_from = {}
        
        while pq:
            current_cost, current = heapq.heappop(pq)
            
            # Se chegamos ao objetivo
            if current == goal:
                # Reconstrói o caminho
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(start)
                path.reverse()
                return path
            
            # Se já processamos este nó com custo menor, pula
            if current_cost > costs.get(current, float('inf')):
                continue
            
            # Explora vizinhos
            for dx, dy in directions:
                neighbor = (current[0] + dx, current[1] + dy)
                nx, ny = neighbor
                
                # Verifica limites do grid
                if not (0 <= nx < cols and 0 <= ny < rows):
                    continue
                
                # Verifica se é obstáculo
                if self.grid_map[ny, nx] == 1:
                    continue
                
                # Para movimentos diagonais, verifica se os lados adjacentes estão livres
                # (evita "cortar" cantos de obstáculos)
                if dx != 0 and dy != 0:
                    if (self.grid_map[current[1], current[0] + dx] == 1 or 
                        self.grid_map[current[1] + dy, current[0]] == 1):
                        continue
                
                # Calcula novo custo
                move_cost = get_cost(dx, dy)
                new_cost = current_cost + move_cost
                
                # Se encontramos um caminho melhor para o vizinho
                if new_cost < costs.get(neighbor, float('inf')):
                    costs[neighbor] = new_cost
                    came_from[neighbor] = current
                    heapq.heappush(pq, (new_cost, neighbor))
        
        # Não encontrou caminho
        return None

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
            # Verifica se os locais existem no mapeamento
            if origem not in self.location_coords:
                raise ValueError(f'Origin location "{origem}" not found in location mapping')
            if destino not in self.location_coords:
                raise ValueError(f'Destination location "{destino}" not found in location mapping')
            
            # Obtém coordenadas
            start_coords = self.location_coords[origem]
            goal_coords = self.location_coords[destino]
            
            self.get_logger().info(f'Planning path from {start_coords} to {goal_coords}')
            
            # Executa Dijkstra
            path = self.dijkstra(start_coords, goal_coords)
            
            if path is None:
                response.success = False
                response.waypoints = []
                response.message = f'No path found from {origem} to {destino}'
                self.get_logger().warn(response.message)
                
                # Adiciona ao visualizador (falha)
                self.visualizer.add_path_result(origem, destino, None, False, response.message)
                
                return response
            
            # Converte coordenadas de volta para nomes de locais
            # Primeira posição: origem com coordenadas
            waypoints = [f'{origem} {start_coords}']
            
            # Posições intermediárias: apenas coordenadas
            for coord in path[1:-1]:
                waypoints.append(f'{coord}')
            
            # Última posição: destino com coordenadas
            waypoints.append(f'{destino} {goal_coords}')
            
            response.success = True
            response.waypoints = waypoints
            response.message = f'Path planned successfully with {len(waypoints)} waypoints'
            
            self.get_logger().info(f'Path planned with {len(path)} points: {waypoints}')
            
            # Adiciona ao visualizador (sucesso)
            self.visualizer.add_path_result(origem, destino, path, True, response.message)
            
        except Exception as e:
            response.success = False
            response.waypoints = []
            response.message = f'Path planning failed: {str(e)}'
            self.get_logger().error(f'Path planning error: {e}')
            
            # Adiciona ao visualizador (erro)
            self.visualizer.add_path_result(origem, destino, None, False, str(e))
        
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