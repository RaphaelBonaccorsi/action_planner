# 🚁 Action Planner - Autonomous Drone Mission System

[![ROS2](https://img.shields.io/badge/ROS2-Humble-blue.svg)](https://docs.ros.org/en/humble/)
[![Docker](https://img.shields.io/badge/Docker-Ready-2496ED.svg?logo=docker)](https://www.docker.com/)
[![Python](https://img.shields.io/badge/Python-3.10-3776AB.svg?logo=python)](https://www.python.org/)
[![License](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)

Sistema autônomo de planejamento e execução de missões para drones usando **ROS2**, **PDDL** (Planning Domain Definition Language) e **algoritmo de Dijkstra** para pathfinding com visualização web em tempo real.

![Demo](demo.gif)

## 📑 Índice

- [Características](#-características)
- [Demonstração](#-demonstração)
- [Arquitetura](#-arquitetura)
- [Pré-requisitos](#-pré-requisitos)
- [Instalação e Execução](#-instalação-e-execução)
- [Visualizador Web](#-visualizador-web)
- [Desenvolvimento](#-desenvolvimento)
- [Estrutura do Projeto](#-estrutura-do-projeto)
- [Contribuindo](#-contribuindo)
- [Licença](#-licença)

## ✨ Características

### 🎯 Planejamento Automático de Missões
- **PDDL Planning**: Geração automática de planos de ação baseados em objetivos
- **Execução Sequencial**: Coordenação inteligente de ações (carregar item, voar, entregar)
- **Lifecycle Management**: Gerenciamento robusto do ciclo de vida dos nós ROS2
- **Tratamento de Falhas**: Sistema resiliente que continua execução mesmo após falhas em ações individuais

### 🗺️ Pathfinding Inteligente
- **Algoritmo de Dijkstra**: Busca otimizada de caminhos com suporte a movimento diagonal
- **Prevenção de Colisões**: Detecção automática de obstáculos e corner-cutting prevention
- **Grid Configurável**: Mapa 15x15 customizável com múltiplas localizações
- **Custos Ponderados**: Movimento cardinal (1.0) e diagonal (1.414) para caminhos realistas

### 🎨 Visualização em Tempo Real
- **Interface Web Moderna**: Dashboard interativo com Canvas HTML5
- **Atualização Automática**: Polling a cada 1 segundo para feedback imediato
- **Histórico Completo**: Registro das últimas 100 execuções com timestamps
- **Análise Visual**: 
  - Grid com obstáculos e espaços livres
  - Caminhos planejados destacados em verde
  - Origem (azul) e destino (vermelho) claramente marcados
  - Localizações conhecidas com identificadores visuais
  - Coordenadas e labels otimizados para legibilidade

### 🔧 DevOps e Containerização
- **Docker-Ready**: Ambiente completamente containerizado
- **Múltiplos Modos de Execução**: Interativo, automático e daemon
- **Volume Mounting**: PDDL files facilmente editáveis sem rebuild
- **Port Mapping**: Acesso direto ao visualizador web

## 🎬 Demonstração

O sistema executa missões complexas de forma autônoma:

1. **Planejamento**: Gera sequência de ações baseada no problema PDDL
2. **Pathfinding**: Calcula rota otimizada entre localizações evitando obstáculos
3. **Execução**: Coordena ações de carregar, voar e entregar itens
4. **Visualização**: Exibe em tempo real todos os caminhos planejados
5. **Resiliência**: Continua missão mesmo se uma localização for inalcançável

**Exemplo de missão**:
- Carregar 3 itens na base
- Voar para casaA, entregar itens
- Retornar à base, carregar mais itens
- Tentar voar para casaD (isolada) - falha graciosamente
- Continuar para outras localizações com sucesso

## 🏗️ Arquitetura

```
┌─────────────────────────────────────────────────────────────┐
│                     Mission Controller                       │
│              (Coordena execução da missão)                   │
└────────────────────┬────────────────────────────────────────┘
                     │
                     ▼
┌─────────────────────────────────────────────────────────────┐
│                    Action Planner                            │
│   (Gera plano PDDL usando OPTIC/TFD solvers)                │
└────────────────────┬────────────────────────────────────────┘
                     │
                     ▼
┌─────────────────────────────────────────────────────────────┐
│               Action Planner Executor                        │
│        (Gerencia execução sequencial de ações)              │
└───┬──────────────┬──────────────┬──────────────┬────────────┘
    │              │              │              │
    ▼              ▼              ▼              ▼
┌─────────┐  ┌─────────┐  ┌─────────┐  ┌─────────────────┐
│ Carregar│  │   Voa   │  │Entregar │  │  Path Planner   │
│  Item   │  │         │  │  Item   │  │  (Dijkstra +    │
│ Action  │  │ Action  │  │ Action  │  │  Visualizer)    │
└─────────┘  └────┬────┘  └─────────┘  └────────┬────────┘
                  │                              │
                  └──────────────┬───────────────┘
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

## 📋 Pré-requisitos

- **Docker** (recomendado) OU
- **ROS2 Humble**
- **Python 3.10+**
- **Navegador Web moderno** (Chrome, Firefox, Safari, Edge)

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
git clone <repository-url>
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

O visualizador web adiciona:

1. **Transparência Operacional**: Visibilidade completa do processo de pathfinding
2. **Debug Facilitado**: Identificação rápida de problemas em rotas
3. **Análise Histórica**: Comparação de execuções para otimização
4. **Demonstração Visual**: Ferramenta educacional e de apresentação
5. **Monitoramento Remoto**: Acesso via navegador de qualquer dispositivo na rede
6. **Modularidade**: Implementação separada não impacta lógica de negócio

### 🎨 Tecnologias do Visualizador

- **Backend**: Flask 3.0.0 + Flask-CORS 4.0.0
- **Frontend**: HTML5 Canvas + Vanilla JavaScript
- **Estilo**: CSS3 com gradientes e animações
- **Arquitetura**: REST API thread-safe
- **Design**: Responsivo e acessível

## 🛠️ Desenvolvimento

### Criar Novos Action Nodes

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
│   │   ├── path_planner.py               # Dijkstra pathfinding
│   │   ├── path_visualizer.py            # Módulo de visualização
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
│   ├── CMakeLists.txt
│   └── package.xml
├── pddl/
│   ├── domain.pddl                       # Domínio PDDL
│   └── problem.pddl                      # Problema PDDL
├── Dockerfile                             # Container definition
├── demo.gif                               # Demonstração visual
└── readme.md                              # Este arquivo
```

## 🤝 Contribuindo

Contribuições são bem-vindas! Para contribuir:

1. Fork o projeto
2. Crie uma branch para sua feature (`git checkout -b feature/MinhaFeature`)
3. Commit suas mudanças (`git commit -m 'Adiciona MinhaFeature'`)
4. Push para a branch (`git push origin feature/MinhaFeature`)
5. Abra um Pull Request

### 📝 Guidelines

- Siga PEP 8 para código Python
- Adicione testes quando aplicável
- Atualize documentação relevante
- Use commits semânticos

## 📄 Licença

Este projeto está sob a licença MIT. Veja o arquivo [LICENSE](LICENSE) para mais detalhes.

---

<div align="center">

### 🛠️ Tecnologias Principais

![ROS2](https://img.shields.io/badge/ROS2-Humble-22314E?style=for-the-badge&logo=ros&logoColor=white)
![Python](https://img.shields.io/badge/Python-3.10-3776AB?style=for-the-badge&logo=python&logoColor=white)
![Flask](https://img.shields.io/badge/Flask-3.0-000000?style=for-the-badge&logo=flask&logoColor=white)
![Docker](https://img.shields.io/badge/Docker-Ready-2496ED?style=for-the-badge&logo=docker&logoColor=white)

**Dúvidas ou sugestões?** Abra uma [issue](../../issues) ou entre em contato!

</div>
