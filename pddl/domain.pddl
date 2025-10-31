(define (domain elevador-dominio)

(:requirements :typing)

(:types elevador passageiro andar -object)

(:predicates (passageiro-no-andar ?p -passageiro ?a -andar)
             (passageiro-no-elevador ?p -passageiro ?e -elevador)
             (elevador-no-andar ?e -elevador ?a -andar) (proximo ?n1 - andar ?a2 - andar)
)

(:action move_acima
  :parameters (?e - elevador ?atual ?prox - andar)
  :precondition (and(elevador-no-andar ?e ?atual) (proximo ?atual ?prox))
  :effect (and(not(elevador-no-andar ?e ?atual))(elevador-no-andar ?e ?prox))
)

(:action move_abaixo
  :parameters (?e -elevador ?atual ?prox - andar)
  :precondition (and(elevador-no-andar ?e ?atual)(proximo ?prox ?atual))
  :effect (and (not (elevador-no-andar ?e ?atual)) (elevador-no-andar ?e ?prox))
)

(:action entrar
  :parameters (?p -passageiro ?numAndar -andar ?e -elevador )
  :precondition ( and(elevador-no-andar ?e ?numAndar) (passageiro-no-andar ?p ?numAndar))
  :effect (and(not(passageiro-no-andar ?p ?numAndar)) (passageiro-no-elevador ?p ?e))
)
 
(:action sair
  :parameters (?p -passageiro ?numAndar -andar ?e -elevador )
  :precondition ( and(elevador-no-andar ?e ?numAndar)(passageiro-no-elevador ?p ?e))
  :effect (and (passageiro-no-andar ?p ?numAndar) (not(passageiro-no-elevador ?p ?e)))
)

)