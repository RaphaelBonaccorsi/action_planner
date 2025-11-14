(define (problem droneProblemV3 )
    (:domain droneDomainV3)
    (:objects 
        quad - drone
        casaA casaB casaC - local
        base1 - base
        item1 item2 - item
    )
    (:init ;;essa seção contém uma lista de fatos que são verdadeiros no estado inicial.
        (posicao quad base1)
        (= (total-cost) 0)
        (= (dist-per_drone quad) 0)
        
        ; Capacidade e carga do drone
        (= (capacidade quad) 2)
        (= (carga-total quad) 0)
        (= (carga quad item1) 0)
        (= (carga quad item2) 0)

        (= (demandaItem item1 base1) 0 ) (= (demandaItem item2 base1) 0)
        (= (demandaItem item1 casaA) 2) (= (demandaItem item2 casaA) 1)
        (= (demandaItem item1 casaB) 0) (= (demandaItem item2 casaB) 3)
        (= (demandaItem item1 casaC) 1) (= (demandaItem item2 casaC) 2)
        (= (distancia base1 casaA) 18) (= (distancia base1 casaB) 45) (= (distancia base1 casaC) 20)
        (= (distancia casaA base1) 18) (= (distancia casaA casaB) 12) (= (distancia casaA casaC) 35)
        (= (distancia casaB base1) 45) (= (distancia casaB casaA) 12) (= (distancia casaB casaC) 10)
        (= (distancia casaC base1) 20) (= (distancia casaC casaA) 35) (= (distancia casaC casaB) 10)
        ; distâncias reflexivas necessárias
        (= (distancia base1 base1) 0)
        (= (distancia casaA casaA) 0)
        (= (distancia casaB casaB) 0)
        (= (distancia casaC casaC) 0)

    )
    (:goal 
        (and 
            (= (demandaItem item1 casaA) 0)
            (= (demandaItem item2 casaA) 0)
            (= (demandaItem item1 casaB) 0)
            (= (demandaItem item2 casaB) 0)
            (= (demandaItem item1 casaC) 0)
            (= (demandaItem item2 casaC) 0)
        )
    )
    
    (:metric minimize (total-time))
    
)