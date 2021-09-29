{-# OPTIONS_GHC -cpp -XMagicHash #-}
{- For Hugs, use the option -F"cpp -P -traditional" -}

module ThrusterAllocation where

import qualified Prelude

#ifdef __GLASGOW_HASKELL__
import qualified GHC.Base
#else
-- HUGS
import qualified IOExts
#endif

#ifdef __GLASGOW_HASKELL__
type Any = GHC.Base.Any
#else
-- HUGS
type Any = ()
#endif

data Nat =
   O
 | S Nat

data List a =
   Nil
 | Cons a (List a)

type Sort = Any

data Finfun_on rT =
   Finfun_nil
 | Finfun_cons Sort (List Sort) rT (Finfun_on rT)

type Finfun_of rT =
  Finfun_on rT
  -- singleton inductive, whose constructor was FinfunOf
  
type Matrix r = Finfun_of r
  -- singleton inductive, whose constructor was Matrix
  
compute_thrust :: (Matrix Nat) -> Nat
compute_thrust _ =
  S O

