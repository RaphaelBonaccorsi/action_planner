Buildar container:
docker build -t action_planner .

Rodar com o diretório pddl montado:
docker run -it -v ./pddl:/pddl action_planner

Rodar o sistema dentro do container:
ros2 launch launch/launch.py

Passo a passo para criar novos actions nodes/mudar problem e domain:
- Criar os actions nodes dentro da pasta script, atualizar o nome da ação na linha:

def __init__(self):
        super().__init__("pegar")
* Alterar o "pegar"

- Na função new goal alterar os argumentos de acordo com os argumentos da ação no domain

- Adicionar no CMakeLists.txt o novo action node: 

install(PROGRAMS
  -- nós existentes --
  scripts/pegar.py <-- mudar de acordo com o nome do seu arquivo .py
  DESTINATION lib/${PROJECT_NAME})

- Adicionar na variável nodes o nó criado, por exemplo:
nodes = [
    {
        'node_name': 'pegar',
        'depends_on': []
    },
    restante dos nós...
]