(define (domain amazonwarehouse)

(:requirements :strips :typing :adl :fluents :durative-actions)

(:types  
zone
palet robot - physical_object
) 

(:predicates
(free ?x - robot)
(at ?x - physical_object ?zone - zone)
(has-palet ?x - zone)
(has-robot ?x - zone)
(loaded ?x - robot)
(holding ?x -robot ?y - palet)
)


(:durative-action move
    :parameters (
        ?robot - robot
        ?zone1 ?zone2 - zone
    )
    :duration ( = ?duration 5)
    :condition (and 
        (at start (at ?robot ?zone1))
        (at start (not (has-robot ?zone2)))
        (at start (free ?robot))
        (over all (not (loaded ?robot)))
    )

    :effect (and 
        (at start (not (at ?robot ?zone1)))
        (at end (at ?robot ?zone2))
        (at start (not (free ?robot)))
        (at end (free ?robot))
        (at start (not (has-robot ?zone1)))
        (at end (has-robot ?zone2))
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
        (at start (not (has-robot ?zone2)))
        (at start (free ?robot))
        (over all (holding ?robot ?palet))
        (at start (at ?palet ?zone1))
        (at start (not (has-palet ?zone2)))
        (over all (loaded ?robot))
    )
    :effect (and 
        (at start (not (at ?robot ?zone1)))
        (at end (at ?robot ?zone2))
        (at start (not (at ?palet ?zone1)))
        (at end (at ?palet ?zone2))
        (at start (not (free ?robot)))
        (at end (free ?robot))
        (at start (not (has-palet ?zone1)))
        (at end (has-palet ?zone2))
        (at start (not (has-robot ?zone1)))
        (at end (has-robot ?zone2))
    )
)

(:action lift
    :parameters (
        ?robot - robot
        ?zone - zone
        ?palet - palet
    )
    :precondition (and 
        (at ?robot ?zone)
        (at ?palet ?zone)
        (not (loaded ?robot))
        (not (holding ?robot ?palet))
        (free ?robot)
    )
    :effect(and 
        (holding ?robot ?palet)
        (loaded ?robot)
    )
)

(:action drop
    :parameters (
        ?robot - robot
        ?zone - zone
        ?palet - palet
    )
    :precondition(and 
        (at ?robot ?zone)
        (at ?palet ?zone)
        (holding ?robot ?palet)
        (loaded ?robot)
        (free ?robot)
    )
    :effect(and
        (not (holding ?robot ?palet))
        (not (loaded ?robot))
    )
)
)

    