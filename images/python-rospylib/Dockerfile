FROM python

# Use bash
SHELL ["/bin/bash", "-c"]

# Install deps
RUN pip install roslibpy

# Copy talker script across
COPY talker.py /home/
