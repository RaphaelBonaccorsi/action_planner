(define (domain droneDomainV3)

    (:requirements :strips :typing :equality :numeric-fluents :durative-actions)
    (:types
        local drone item - object
        base - local
    )
    
    (:predicates ;;seção que contém a lista de variáveis de estado do modelo. 
        (posicao ?dr - drone ?c - local)
    )
    
    (:functions
     (total-cost) - number
     (dist-per_drone ?dr - drone) - number
     (distancia ?orig - local ?dest - local) - number
     (capacidade ?dr - drone) - number;; capacidade máxima do drone
     (carga-total ?dr - drone) - number;; carga total atual do drone
     (carga ?dr - drone ?i - item) - number;; quantidade de cada item no drone
     (demandaItem ?i - item ?c - local) - number ;; demanda cliente item      
    )

    (:durative-action entregarItem
        :parameters (?dr - drone ?i - item ?l - local)
        :duration (= ?duration 1)
        :condition (and
            (at start (posicao ?dr ?l))
            (at start (>= (demandaItem ?i ?l) 1))
            (at start (>= (carga ?dr ?i) 1))
        )
        :effect (and
            (at end (decrease (demandaItem ?i ?l) 1))
            (at end (decrease (carga ?dr ?i) 1))
            (at end (decrease (carga-total ?dr) 1))
        )
    )

    (:durative-action carregarItem
        :parameters (?dr - drone ?i - item ?b - base)
        :duration (= ?duration 1)
        :condition (and
            (at start (posicao ?dr ?b))
            (at start (< (carga-total ?dr) (capacidade ?dr)))
        )
        :effect (and
            (at end (increase (carga ?dr ?i) 1))
            (at end (increase (carga-total ?dr) 1))
        )
    )

    (:durative-action voa
        :parameters (?dr - drone ?origem ?destino - local)
        :duration (= ?duration (distancia ?origem ?destino))
        :condition (and
            (at start (posicao ?dr ?origem))
        )
        :effect (and
            (at start (not (posicao ?dr ?origem)))
            (at end (posicao ?dr ?destino))
            (at end (increase (total-cost) (distancia ?origem ?destino)))
            (at end (increase (dist-per_drone ?dr) (distancia ?origem ?destino)))
        )
    )
)