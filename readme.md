# Buildar container:
```
docker build -t action_planner .
```

# Rodar com o diretório pddl montado e porta da visualização:

## Opção 1: Modo Interativo (manual)
```bash
docker run -it -v ./pddl:/pddl -p 5007:5007 action_planner
```
Depois, dentro do container, execute:
```bash
ros2 launch launch/launch.py
```

## Opção 2: Inicialização Automática
```bash
docker run -it -v ./pddl:/pddl -p 5007:5007 action_planner /usr/local/bin/start-ros2.sh
```
Este comando já inicia o ROS2 automaticamente ao entrar no container.

## Opção 3: Modo Background (daemon)
```bash
docker run -d --name action_planner_running -v ./pddl:/pddl -p 5007:5007 action_planner /usr/local/bin/start-ros2.sh
```
Para ver os logs:
```bash
docker logs -f action_planner_running
```
Para acessar o container:
```bash
docker exec -it action_planner_running /bin/bash
```
Para parar:
```bash
docker stop action_planner_running
docker rm action_planner_running
```

# Acessar a visualização web:
Após iniciar o sistema, abra no navegador:
```
http://localhost:5007
```

A interface exibirá:
- Mapa com obstáculos
- Caminho planejado em tempo real
- Histórico de todas as execuções
- Pontos de origem e destino

# Passo a passo para criar novos actions nodes/mudar problem e domain:
- Criar os actions nodes dentro da pasta script, atualizar o nome da ação na linha:

``
def __init__(self):
        super().__init__("pegar")
``

Alterar o "pegar" para a ação desejada

- Na função new goal alterar os argumentos de acordo com os argumentos da ação no domain

- Adicionar no CMakeLists.txt o novo action node: 

```
install(PROGRAMS 
  -- nós existentes -- 
  scripts/pegar.py <-- mudar de acordo com o nome do seu arquivo .py
  DESTINATION lib/${PROJECT_NAME})
```

- Adicionar na variável nodes do lifecycle manager o nó criado, por exemplo:

``
nodes = [
    {
        'node_name': 'pegar',
        'depends_on': []
    },
    restante dos nós...
]
``
