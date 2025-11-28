# Path Planner Visualizer

Sistema de visualização web em tempo real para o planejador de caminhos ROS2.

## Arquitetura

### Módulos Criados

1. **path_visualizer.py** - Módulo de visualização modular
   - `PathVisualizer`: Gerencia histórico e dados de caminhos
   - `PathVisualizerServer`: Servidor Flask em thread separada
   - API REST para comunicação com a interface web

2. **Interface Web**
   - **templates/path_visualizer.html**: Interface HTML
   - **static/style.css**: Estilos responsivos
   - **static/visualizer.js**: Lógica de visualização com Canvas

## Funcionalidades

### Visualização em Tempo Real
- Atualização automática a cada 1 segundo
- Grid com obstáculos e espaços livres
- Marcação de locais conhecidos (base1, casaa, casab, casac)
- Destaque de origem (azul) e destino (vermelho)
- Caminho desenhado em verde

### Histórico
- Todas as execuções de `plan_path_callback`
- Timestamp de cada planejamento
- Status (sucesso/falha)
- Número de pontos no caminho
- Mensagem descritiva
- Histórico persistente (últimas 100 execuções)

### Navegação
- Clique em qualquer item do histórico para visualizá-lo
- Legenda colorida explicativa
- Interface responsiva
- Indicador de status de conexão

## Como Usar

### 1. Build do Container
```bash
docker build -t action_planner .
```

### 2. Executar Container
```bash
docker run -it -v ./pddl:/pddl -p 5007:5007 action_planner
```

**Importante**: A flag `-p 5007:5007` mapeia a porta do servidor web.

### 3. Iniciar o Sistema ROS2
```bash
ros2 launch launch/launch.py
```

### 4. Acessar Visualização
Abra no navegador: **http://localhost:5007**

## API REST

### Endpoints Disponíveis

- `GET /` - Interface web principal
- `GET /api/grid` - Dados do grid e localizações
- `GET /api/history` - Histórico completo de planejamentos
- `GET /api/latest` - Resultado mais recente
- `GET /api/health` - Health check do servidor

## Integração com path_planner.py

O código foi integrado de forma modular:

```python
# Importação do módulo
from path_visualizer import PathVisualizer, PathVisualizerServer

# Inicialização (no __init__)
self.visualizer = PathVisualizer(self.grid_map, self.location_coords)
self.visualizer_server = PathVisualizerServer(self.visualizer, port=5007)
self.visualizer_server.start()

# Registro de resultados (no plan_path_callback)
self.visualizer.add_path_result(origem, destino, path, success, message)
```

## Cores da Interface

- **Cinza Escuro (#34495e)**: Obstáculos
- **Cinza Claro (#ecf0f1)**: Espaço livre
- **Azul (#3498db)**: Origem
- **Vermelho (#e74c3c)**: Destino
- **Verde (#2ecc71)**: Caminho planejado
- **Laranja (#f39c12)**: Locais conhecidos

## Dependências Adicionadas

```dockerfile
flask==3.0.0
flask-cors==4.0.0
```

## Características Técnicas

- Servidor Flask em thread daemon (não bloqueia ROS2)
- Canvas HTML5 para renderização eficiente
- Polling JavaScript para atualizações em tempo real
- Thread-safe com locks para acesso ao histórico
- Logs Flask desabilitados para não poluir logs do ROS

## Manutenção

O código foi estruturado para facilitar manutenção:

1. **Separação de responsabilidades**: Lógica de negócio em `path_planner.py`, visualização em `path_visualizer.py`
2. **API REST**: Permite futuras integrações
3. **Modular**: Fácil adicionar novas funcionalidades
4. **Comentado**: Código bem documentado em português

## Troubleshooting

### Porta 5007 já em uso
```bash
# Altere a porta no path_planner.py e no docker run
docker run -it -v ./pddl:/pddl -p 8080:8080 action_planner
```

### Visualização não atualiza
- Verifique se o servidor está rodando (logs do ROS2)
- Teste o health check: http://localhost:5007/api/health
- Verifique o console do navegador (F12)
