Buildar container:
docker build -t action_planner .

Rodar com o diretório pddl montado:
docker run -it -v /home/raphael/action_planner_container/pddl:/pddl action_planner