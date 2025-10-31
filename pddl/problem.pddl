(define (problem elevador-problema)
  (:domain elevador-dominio)
  (:objects 
   a1 a2 a3 a4 a5 - andar
   p1 p2 p3 - passageiro
   e1 e2 - elevador
  )
  
  (:init
   (proximo a1 a2) (proximo a2 a3) (proximo a3 a4) (proximo a4 a5)
   (elevador-no-andar e1 a1) (elevador-no-andar e2 a5)
   (passageiro-no-andar p1 a2) (passageiro-no-andar p2 a2)
   (passageiro-no-andar p3 a4)
  )
  
  (:goal (and (passageiro-no-andar p1 a1)
              (passageiro-no-andar p2 a1) 
              (passageiro-no-andar p3 a1))
  )
)