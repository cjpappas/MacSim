
type __ = Obj.t

type nat =
| O
| S of nat

type 'a list =
| Nil
| Cons of 'a * 'a list

module Finite =
 struct
  type sort = __
 end

type 'rT finfun_on =
| Finfun_nil
| Finfun_cons of Finite.sort * Finite.sort list * 'rT * 'rT finfun_on

type 'rT finfun_of =
  'rT finfun_on
  (* singleton inductive, whose constructor was FinfunOf *)

type 'r matrix =
  'r finfun_of
  (* singleton inductive, whose constructor was Matrix *)

(** val compute_thrust : nat matrix -> nat **)

let compute_thrust _ =
  S O
