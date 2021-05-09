(define (domain amazonwarehouse)

(:requirements :strips :typing :durative-actions)

(:types  
zone
palet robot - physical_object
) 

(:predicates
(free ?x - robot)
(at ?x - physical_object ?zone - zone)
(no-palet ?x - zone)
(no-robot ?x - zone)
(unloaded ?x - robot)
(holding ?robot - robot ?palet - palet)
)


(:durative-action move
    :parameters (
        ?robot - robot
        ?zone1 ?zone2 - zone
    )
    :duration ( = ?duration 5)
    :condition (and 
        (at start (at ?robot ?zone1))
        (at start (no-robot ?zone2))
        (at start (free ?robot))
        (at start (unloaded ?robot))
    )

    :effect (and 
        (at start (not (at ?robot ?zone1)))
        (at end (at ?robot ?zone2))
        (at start (not (free ?robot)))
        (at end (free ?robot))
        (at start (no-robot ?zone1))
        (at end (not (no-robot ?zone2)))
    )
)

(:durative-action transport
    :parameters (
        ?robot - robot
        ?palet - palet
        ?zone1 ?zone2 - zone
    )
    :duration ( = ?duration 5)
    :condition (and 
        (at start (at ?robot ?zone1))
        (at start (no-robot ?zone2))
        (at start (free ?robot))
        (at start (holding ?robot ?palet))
        (at start (at ?palet ?zone1))
        (at start (no-palet ?zone2))
    )
    :effect (and 
        (at start (not (at ?robot ?zone1)))
        (at end (at ?robot ?zone2))
        (at start (not (at ?palet ?zone1)))
        (at end (at ?palet ?zone2))
        (at start (not (free ?robot)))
        (at end (free ?robot))
        (at start (no-palet ?zone1))
        (at end (not (no-palet ?zone2)))
        (at start (no-robot ?zone1))
        (at end (not (no-robot ?zone2)))
    )
)

(:action lift
    :parameters (
        ?robot - robot
        ?palet - palet
        ?zone - zone
    )
    :precondition (and 
        (at ?robot ?zone)
        (at ?palet ?zone)
        (free ?robot)
    )
    :effect(and 
        (holding ?robot ?palet)
        (not (unloaded ?robot))
    )
)

(:action drop
    :parameters (
        ?robot - robot
        ?palet - palet
        ?zone - zone
    )
    :precondition(and 
        (at ?robot ?zone)
        (at ?palet ?zone)
        (free ?robot)
    )
    :effect(and
        (not (holding ?robot ?palet))
        (unloaded ?robot)
    )
)
)

    