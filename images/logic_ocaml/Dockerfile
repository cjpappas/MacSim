FROM ocaml/opam:alpine-ocaml-4.14

# Use bash
SHELL ["/bin/bash", "-c"]

# Install deps
RUN opam init
Run opam switch create 4.12.0
Run opam install dune