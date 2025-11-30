(define (problem droneProblemV3 )
    (:domain droneDomainV3)
    (:objects 
        quad - drone
        casaA casaB casaC casaD - local
        base1 - base
        deposito - local
        item1 item2 - item
    )
    (:init ;;essa seção contém uma lista de fatos que são verdadeiros no estado inicial.
        (posicao quad base1)
        (= (total-cost) 0)
        (= (dist-per_drone quad) 0)
        
        ; Capacidade e carga do drone
        (= (capacidade quad) 2) ; bug comentado em sala que com capacidade 2 ele carrega até 3
        (= (carga-total quad) 0)
        (= (carga quad item1) 0)
        (= (carga quad item2) 0)

        (= (demandaItem item1 base1) 0 ) (= (demandaItem item2 base1) 0)
        (= (demandaItem item1 casaA) 2) (= (demandaItem item2 casaA) 1)
        (= (demandaItem item1 casaB) 0) (= (demandaItem item2 casaB) 3)
        (= (demandaItem item1 casaC) 1) (= (demandaItem item2 casaC) 2)
        (= (demandaItem item1 casaD) 1) (= (demandaItem item2 casaD) 1)
        (= (demandaItem item1 deposito) 0) (= (demandaItem item2 deposito) 0)
        
        ; Distâncias entre locais
        (= (distancia base1 casaA) 18) (= (distancia base1 casaB) 45) (= (distancia base1 casaC) 20) (= (distancia base1 casaD) 50) (= (distancia base1 deposito) 8)
        (= (distancia casaA base1) 18) (= (distancia casaA casaB) 12) (= (distancia casaA casaC) 35) (= (distancia casaA casaD) 40) (= (distancia casaA deposito) 15)
        (= (distancia casaB base1) 45) (= (distancia casaB casaA) 12) (= (distancia casaB casaC) 10) (= (distancia casaB casaD) 25) (= (distancia casaB deposito) 40)
        (= (distancia casaC base1) 20) (= (distancia casaC casaA) 35) (= (distancia casaC casaB) 10) (= (distancia casaC casaD) 30) (= (distancia casaC deposito) 18)
        (= (distancia casaD base1) 50) (= (distancia casaD casaA) 40) (= (distancia casaD casaB) 25) (= (distancia casaD casaC) 30) (= (distancia casaD deposito) 45)
        (= (distancia deposito base1) 8) (= (distancia deposito casaA) 15) (= (distancia deposito casaB) 40) (= (distancia deposito casaC) 18) (= (distancia deposito casaD) 45)
        
        ; distâncias reflexivas necessárias
        (= (distancia base1 base1) 0)
        (= (distancia casaA casaA) 0)
        (= (distancia casaB casaB) 0)
        (= (distancia casaC casaC) 0)
        (= (distancia casaD casaD) 0)
        (= (distancia deposito deposito) 0)

    )
    (:goal 
        (and 
            (= (demandaItem item1 casaA) 0)
            (= (demandaItem item2 casaA) 0)
            (= (demandaItem item1 casaB) 0)
            (= (demandaItem item2 casaB) 0)
            (= (demandaItem item1 casaC) 0)
            (= (demandaItem item2 casaC) 0)
            ; casaD isolada - tentará planejar rota mas falhará
            (= (demandaItem item1 casaD) 0)
            (= (demandaItem item2 casaD) 0)
        )
    )
    
    (:metric minimize (total-time))
    
)