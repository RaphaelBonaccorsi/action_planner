Buildar container:
docker build -t action_planner .

Rodar com o diretório pddl montado:
docker run -it -v ./pddl:/pddl action_planner
