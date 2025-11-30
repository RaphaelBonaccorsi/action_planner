# 🚁 Action Planner - Autonomous Drone Mission System

[![ROS2](https://img.shields.io/badge/ROS2-Humble-blue.svg)](https://docs.ros.org/en/humble/)
[![Docker](https://img.shields.io/badge/Docker-Ready-2496ED.svg?logo=docker)](https://www.docker.com/)
[![Python](https://img.shields.io/badge/Python-3.10-3776AB.svg?logo=python)](https://www.python.org/)

Sistema autônomo de planejamento e execução de missões para drones usando **ROS2**, **PDDL** (Planning Domain Definition Language) e **algoritmo de Dijkstra** para pathfinding com visualização web em tempo real.

![Demo](demo.gif)

## 📑 Índice

- [Pré-requisitos](#-pré-requisitos)
- [Instalação e Execução](#-instalação-e-execução)
- [Arquitetura](#-arquitetura)
- [Visualizador Web](#-visualizador-web)
- [Desenvolvimento](#-desenvolvimento)
- [Estrutura do Projeto](#-estrutura-do-projeto)

## 📋 Pré-requisitos

- **Docker** (recomendado) OU
- **ROS2 Humble**
- **Python 3.10+**
- **Navegador Web** (Chrome, Firefox, Safari, Edge)

## 🚀 Instalação e Execução

### Opção 1: Docker (Recomendado)

#### Build da imagem
```bash
docker build -t action_planner .
```

#### Execução Interativa
```bash
docker run -it -v ./pddl:/pddl -p 5007:5007 action_planner
# Dentro do container
ros2 launch launch/launch.py
```

#### Execução Automática
```bash
docker run -it -v ./pddl:/pddl -p 5007:5007 action_planner /usr/local/bin/start-ros2.sh
```

#### Execução em Background (Daemon)
```bash
# Iniciar
docker run -d --name action_planner_running \
  -v ./pddl:/pddl \
  -p 5007:5007 \
  action_planner /usr/local/bin/start-ros2.sh

# Ver logs
docker logs -f action_planner_running

# Acessar container
docker exec -it action_planner_running /bin/bash

# Parar e remover
docker stop action_planner_running
docker rm action_planner_running
```

### Opção 2: Build Local (Sem Docker)

```bash
# Clonar repositório
git clone https://github.com/RaphaelBonaccorsi/action_planner.git
cd action_planner

# Source ROS2
source /opt/ros/humble/setup.bash

# Build workspace
colcon build --symlink-install

# Source workspace
source install/setup.bash

# Executar
ros2 launch launch/launch.py
```

## 🏗️ Arquitetura

```
┌─────────────────────────────────────────────────────────────┐
│                     Mission Controller                      │
│              (Coordena execução da missão)                  │
└──────────────────────────────┬──────────────────────────────┘
                               │
                               ▼
┌─────────────────────────────────────────────────────────────┐
│                    Action Planner                           │
│   (Gera plano PDDL usando OPTIC/TFD solvers)                │
└──────────────────────────────┬──────────────────────────────┘
                               │
                               ▼
┌─────────────────────────────────────────────────────────────┐
│               Action Planner Executor                       │
│        (Gerencia execução sequencial de ações)              │
└───┬──────────────┬──────────────┬──────────────┬────────────┘
    │              │              │              │
    ▼              ▼              ▼              ▼
┌─────────┐    ┌─────────┐    ┌─────────┐    ┌────────────────┐
│ Carregar│    │   Voa   │    │Entregar │    │  Path Planner  │
│  Item   │    │         │    │  Item   │    │  (Dijkstra +   │
│ Action  │    │ Action  │    │ Action  │    │  Visualizer)   │
└─────────┘    └────┬────┘    └─────────┘    └────────┬───────┘
                    │                                 │
                    └────────────────┬────────────────┘
                                     ▼
                      ┌──────────────────────────────┐
                      │   Path Visualizer Server     │
                      │    (Flask REST API)          │
                      └──────────────┬───────────────┘
                                     ▼
                      ┌──────────────────────────────┐
                      │   Web Interface (Canvas)     │
                      │   http://localhost:5007      │
                      └──────────────────────────────┘
```

### 🔑 Componentes Principais

- **Mission Controller**: Ponto de entrada que dispara o planejamento de missões
- **Action Planner**: Interface com solvers PDDL (OPTIC/TFD) para geração de planos
- **Action Planner Executor**: Orquestra execução de ações respeitando precondições
- **Path Planner**: Implementa Dijkstra para navegação com obstáculos
- **Action Nodes**: Executores específicos (carregar, voar, entregar)
- **Lifecycle Manager**: Gerencia transições de estado dos nós ROS2
- **Path Visualizer**: Módulo modular de visualização web (Flask + Canvas)

## 🎨 Visualizador Web

### Acesso
Após iniciar o sistema, abra no navegador:
```
http://localhost:5007
```

### Funcionalidades

#### 🗺️ Visualização do Grid
- **Grid Map**: Mapa 15x15 com células de 50x50 pixels
- **Obstáculos**: Blocos em cinza escuro representando áreas inacessíveis
- **Localizações**: Círculos laranjas marcando pontos de interesse
- **Coordenadas**: Labels pretos dentro de cada célula para referência
- **Legenda Visual**: Cores e símbolos explicados

#### 📍 Representação de Caminhos
- **Linha Verde**: Trajetória planejada entre origem e destino
- **Círculo Azul**: Ponto de origem da rota
- **Círculo Vermelho**: Ponto de destino da rota
- **Pontos Intermediários**: Waypoints ao longo do caminho

#### 📜 Histórico de Planejamentos
- **Timeline Reverso**: Execuções mais recentes no topo
- **Filtro por Status**: Sucesso (verde) ou Falha (vermelho)
- **Clique para Revisitar**: Selecione qualquer execução passada
- **Detalhes**:
  - Timestamp preciso
  - Rota (origem → destino)
  - Número de waypoints
  - Mensagem de status

#### ⚡ Atualizações em Tempo Real
- **Polling Automático**: Atualiza a cada 1 segundo
- **Thread-Safe**: Servidor Flask em thread daemon
- **API REST**: Endpoints para integração
  - `GET /` - Interface web
  - `GET /api/grid` - Dados do grid
  - `GET /api/history` - Histórico completo
  - `GET /api/latest` - Última execução
  - `GET /api/health` - Health check

### 💡 Valor Agregado

O visualizador web oferece:

- **Transparência Operacional**: Visibilidade completa do processo de pathfinding
- **Debug Facilitado**: Identificação rápida de problemas em rotas
- **Análise Histórica**: Comparação de execuções para otimização
- **Monitoramento Remoto**: Acesso via navegador de qualquer dispositivo na rede

### 🎨 Tecnologias

- **Backend**: Flask 3.0.0 + Flask-CORS 4.0.0
- **Frontend**: HTML5 Canvas + Vanilla JavaScript
- **Arquitetura**: REST API thread-safe

## 🛠️ Desenvolvimento

### Criar Novos Action Nodes

> **Nota**: Os diretórios `templates/` e `static/` contêm arquivos do visualizador web que são acessados em runtime pelo servidor Flask e não precisam ser instalados pelo CMakeLists.

1. **Criar script Python** em `action_planner/scripts/`:

```python
from action_executor_base import ActionExecutorBase

class MinhaAcao(ActionExecutorBase):
    def __init__(self):
        super().__init__("minha_acao")  # Nome da ação
    
    def new_goal(self, parameters):
        # Lógica de execução
        arg1 = parameters[0]
        arg2 = parameters[1]
        # ... implementação
        return True  # ou False
```

2. **Adicionar ao CMakeLists.txt**:

```cmake
install(PROGRAMS 
  scripts/minha_acao.py
  DESTINATION lib/${PROJECT_NAME})
```

> **Nota**: Scripts auxiliares como `path_visualizer.py`, `action_planner_executor.py`, `action_planner_memory.py` e `action_executor_base.py` são importados pelos nós principais e não precisam ser listados no `install(PROGRAMS ...)`.

3. **Registrar no Lifecycle Manager** (`scripts/lifecycle_manager.py`):

```python
nodes = [
    {
        'node_name': 'minha_acao',
        'depends_on': []  # ou lista de dependências
    },
    # ... outros nós
]
```

4. **Definir no PDDL** (`pddl/domain.pddl`):

```lisp
(:action minha_acao
    :parameters (?arg1 ?arg2)
    :precondition (and
        ; precondições
    )
    :effect (and
        ; efeitos
    )
)
```

### Modificar PDDL Problem

Edite `pddl/problem.pddl` para definir:
- Objetos (`objects`)
- Estado inicial (`init`)
- Objetivo da missão (`goal`)

**Exemplo**:
```lisp
(:objects 
    drone1 - drone
    item1 item2 - item
    base casaA casaB - location
)

(:init
    (at drone1 base)
    (at item1 base)
)

(:goal
    (and (at item1 casaA))
)
```

### Expandir Grid Map

Modifique `action_planner/scripts/path_planner.py`:

```python
# Tamanho do grid
self.grid_map = np.zeros((20, 20), dtype=int)  # Altere dimensões

# Adicionar obstáculos
self.grid_map[5:8, 10:15] = 1  # Bloco de obstáculos

# Adicionar localizações
self.location_coords = {
    'nova_casa': (18, 18),
    # ... outras localizações
}
```

## 📂 Estrutura do Projeto

```
action_planner/
├── action_planner/
│   ├── scripts/
│   │   ├── action_executor_base.py       # Classe base para actions
│   │   ├── action_planner_executor.py    # Executor de planos
│   │   ├── action_planner_memory.py      # Gerenciamento de estado
│   │   ├── action_planner.py             # Nó principal do planner
│   │   ├── carregaritem.py               # Action: carregar item
│   │   ├── entregaritem.py               # Action: entregar item
│   │   ├── voa.py                        # Action: voar entre locais
│   │   ├── path_planner.py               # Dijkstra pathfinding + visualizer
│   │   ├── path_visualizer.py            # Módulo de visualização Flask
│   │   ├── lifecycle_manager.py          # Gerenciador de lifecycle
│   │   └── mission_controller.py         # Controlador de missões
│   ├── src/harpia_msgs/                  # Mensagens e actions customizadas
│   ├── solver/                           # PDDL solvers (OPTIC, TFD)
│   ├── launch/
│   │   └── launch.py                     # Launch file ROS2
│   ├── templates/
│   │   └── path_visualizer.html          # Interface web
│   ├── static/
│   │   ├── style.css                     # Estilos
│   │   └── visualizer.js                 # Lógica frontend
│   ├── output/                           # Diretório para saídas do planner
│   ├── CMakeLists.txt
│   └── package.xml
├── pddl/
│   ├── domain.pddl                       # Domínio PDDL
│   └── problem.pddl                      # Problema PDDL
├── Dockerfile                             # Container definition
├── demo.gif                               # Demonstração visual
└── readme.md                              # Este arquivo
```

---

<div align="center">

### 🛠️ Tecnologias Principais

![ROS2](https://img.shields.io/badge/ROS2-Humble-22314E?style=for-the-badge&logo=ros&logoColor=white)
![Python](https://img.shields.io/badge/Python-3.10-3776AB?style=for-the-badge&logo=python&logoColor=white)
![Flask](https://img.shields.io/badge/Flask-3.0-000000?style=for-the-badge&logo=flask&logoColor=white)
![Docker](https://img.shields.io/badge/Docker-Ready-2496ED?style=for-the-badge&logo=docker&logoColor=white)

</div>
