#!/usr/bin/env python3
"""
Path Visualizer Module
Módulo separado para visualização de caminhos planejados.
Mantém o histórico de execuções e fornece dados para a interface web.
"""
import json
import threading
from datetime import datetime
from flask import Flask, render_template, jsonify
from flask_cors import CORS
import os


class PathVisualizer:
    """
    Gerenciador de visualização de caminhos.
    Armazena histórico e fornece dados para visualização web.
    """
    
    def __init__(self, grid_map, location_coords):
        """
        Inicializa o visualizador.
        
        Args:
            grid_map: numpy array com o mapa do grid
            location_coords: dicionário com mapeamento de locais para coordenadas
        """
        self.grid_map = grid_map.tolist()  # Converte numpy array para lista
        self.location_coords = location_coords
        self.history = []
        self.lock = threading.Lock()
        
    def add_path_result(self, origem, destino, path, success, message):
        """
        Adiciona um resultado de planejamento ao histórico.
        
        Args:
            origem: nome do local de origem
            destino: nome do local de destino
            path: lista de coordenadas (x, y) do caminho encontrado, ou None
            success: booleano indicando se o planejamento foi bem-sucedido
            message: mensagem descritiva do resultado
        """
        with self.lock:
            result = {
                'id': len(self.history) + 1,
                'timestamp': datetime.now().isoformat(),
                'origem': origem,
                'destino': destino,
                'origem_coords': self.location_coords.get(origem),
                'destino_coords': self.location_coords.get(destino),
                'path': path if path else [],
                'success': success,
                'message': message,
                'path_length': len(path) if path else 0
            }
            self.history.append(result)
            
            # Mantém apenas os últimos 100 resultados
            if len(self.history) > 100:
                self.history = self.history[-100:]
    
    def get_history(self):
        """Retorna o histórico completo de planejamentos."""
        with self.lock:
            return list(self.history)
    
    def get_latest(self):
        """Retorna o resultado mais recente."""
        with self.lock:
            return self.history[-1] if self.history else None
    
    def get_grid_data(self):
        """Retorna os dados do grid para visualização."""
        return {
            'grid_map': self.grid_map,
            'location_coords': self.location_coords,
            'grid_size': {
                'rows': len(self.grid_map),
                'cols': len(self.grid_map[0]) if self.grid_map else 0
            }
        }


class PathVisualizerServer:
    """
    Servidor web Flask para visualização de caminhos.
    Roda em thread separada para não bloquear o nó ROS.
    """
    
    def __init__(self, visualizer, port=5007, host='0.0.0.0'):
        """
        Inicializa o servidor web.
        
        Args:
            visualizer: instância de PathVisualizer
            port: porta do servidor
            host: host do servidor (0.0.0.0 para aceitar conexões externas)
        """
        self.visualizer = visualizer
        self.port = port
        self.host = host
        self.app = Flask(__name__, 
                        template_folder=os.path.join(os.path.dirname(__file__), '..', 'templates'),
                        static_folder=os.path.join(os.path.dirname(__file__), '..', 'static'))
        CORS(self.app)  # Permite requisições de qualquer origem
        
        self._setup_routes()
        self.server_thread = None
        
    def _setup_routes(self):
        """Configura as rotas da API."""
        
        @self.app.route('/')
        def index():
            """Página principal."""
            return render_template('path_visualizer.html')
        
        @self.app.route('/api/grid')
        def get_grid():
            """Retorna dados do grid."""
            return jsonify(self.visualizer.get_grid_data())
        
        @self.app.route('/api/history')
        def get_history():
            """Retorna histórico completo."""
            return jsonify(self.visualizer.get_history())
        
        @self.app.route('/api/latest')
        def get_latest():
            """Retorna resultado mais recente."""
            latest = self.visualizer.get_latest()
            return jsonify(latest if latest else {})
        
        @self.app.route('/api/health')
        def health():
            """Health check."""
            return jsonify({'status': 'ok', 'message': 'Path Visualizer Server is running'})
    
    def start(self):
        """Inicia o servidor em uma thread separada."""
        if self.server_thread is None or not self.server_thread.is_alive():
            self.server_thread = threading.Thread(
                target=self._run_server,
                daemon=True
            )
            self.server_thread.start()
            return True
        return False
    
    def _run_server(self):
        """Executa o servidor Flask."""
        # Desabilita logs do Flask para não poluir os logs do ROS
        import logging
        log = logging.getLogger('werkzeug')
        log.setLevel(logging.ERROR)
        
        self.app.run(host=self.host, port=self.port, debug=False, use_reloader=False)
