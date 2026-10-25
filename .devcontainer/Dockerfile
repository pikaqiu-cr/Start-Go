FROM python:3.14-bookworm

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        git \
        curl \
        tree \
        less \
        nano \
        man-db \
        manpages \
        lowdown \
    && rm -rf /var/lib/apt/lists/*

RUN python -m pip install --no-cache-dir \
    pytest \
    pycodestyle \
    numpy \
    matplotlib \
    ruff \
    pdoc \
    rich

RUN useradd -m -s /bin/bash aap

USER aap