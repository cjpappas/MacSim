FROM haskell:stretch

# Use bash
SHELL ["/bin/bash", "-c"]

# Install deps
RUN cabal update
RUN cabal install --lib socket websockets
