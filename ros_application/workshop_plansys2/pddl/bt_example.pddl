(define (domain simple)
(:requirements :strips :typing :adl :fluents :durative-actions)

;; Types ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(:types
robot
piece
zone
);; end Types ;;;;;;;;;;;;;;;;;;;;;;;;;

;; Predicates ;;;;;;;;;;;;;;;;;;;;;;;;;
(:predicates

(robot_available ?r - robot)
(battery_full ?r - robot)
(robot_at ?r - robot ?z - zone)
(piece_at ?p - piece ?z - zone)
(is_recharge_zone ?z - zone)
(is_tool_zone ?z - zone)

);; end Predicates ;;;;;;;;;;;;;;;;;;;;
;; Functions ;;;;;;;;;;;;;;;;;;;;;;;;;
(:functions

);; end Functions ;;;;;;;;;;;;;;;;;;;;
;; Actions ;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(:durative-action move
    :parameters (?r - robot ?z1 ?z2 - zone)
    :duration ( = ?duration 5)
    :condition (and
        (at start(robot_at ?r ?z1))
        (at start(robot_available ?r))
        )
    :effect (and
        (at start(not(robot_at ?r ?z1)))
        (at end(robot_at ?r ?z2))
        (at start(not(robot_available ?r)))
        (at end(robot_available ?r))
    )
) 	
(:durative-action transport
    :parameters (?r - robot ?p - piece ?z1 ?z2 - zone)
    :duration ( = ?duration 5)
    :condition (and
        (over all(battery_full ?r))
        (over all(is_tool_zone ?z2))
        (at start(robot_at ?r ?z1))
        (at start(piece_at ?p ?z1))
        (at start(robot_available ?r))
    )
    :effect (and
        (at start(not(robot_at ?r ?z1)))
        (at end(robot_at ?r ?z2))
        (at start(not(piece_at ?p ?z1)))
        (at end(piece_at ?p ?z2))
        (at start(not(robot_available ?r)))
        (at end(robot_available ?r))
    )
)

(:durative-action recharge
    :parameters (?r - robot ?z - zone)
    :duration ( = ?duration 5)
    :condition (and
        (at start(is_recharge_zone ?z))
        (over all(robot_at ?r ?z))
        (at start(robot_available ?r))
      )
    :effect (and
        (at end(battery_full ?r))
        (at start(not(robot_available ?r)))
        (at end(robot_available ?r))
    )
)
);; end Domain ;;;;;;;;;;;;;;;;;;;;;;;;
