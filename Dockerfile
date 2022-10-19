FROM debian:11

RUN apt update && apt install --no-install-recommends -y \
python3 \
python3-matplotlib \
python3-numpy \
python3-pandas \
python3-scipy \
vim \
&& rm -rf /var/lib/apt/lists/*

COPY ./**.py /app/
COPY ./instances /app/instances
RUN mkdir -p /app/simulation_results/
WORKDIR /app/

