# Use the official Ubuntu 22.04 base image
FROM ubuntu:jammy-20240227

# Set the environment variables to avoid interaction while installing packages
# Set the environment variables to avoid interaction while installing packages
ENV DEBIAN_FRONTEND=noninteractive
ENV LANG pt_BR.UTF-8
ENV LANGUAGE pt_BR:pt
ENV LC_ALL pt_BR.UTF-8
ENV TZ=America/Sao_Paulo

RUN apt-get update && apt-get install -y \
    sudo \
    locales \
    curl \
    wget \
    nano \
    vim \
    git \
    libspdlog1 \
    bzip2 \
    tzdata \
    python3 \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Set up locale
RUN apt-get update && apt-get install -y locales \
    && sed -i -e 's/# pt_BR.UTF-8 UTF-8/pt_BR.UTF-8 UTF-8/' /etc/locale.gen \
    && locale-gen

# Set the timezone0,000 GBP
RUN ln -fs /usr/share/zoneinfo/America/Sao_Paulo /etc/localtime \
    && dpkg-reconfigure --frontend noninteractive tzdata

# Set up a non-root user for better security practices
RUN useradd -m username && \
    echo "username:dockerpassword" | chpasswd && \
    adduser username sudo && \
    echo "username ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers.d/username

# Switch to the non-root user
USER username

# Set the working directory
WORKDIR /home/username

# Create default folder
RUN mkdir -p /home/username/Downloads
RUN mkdir -p /home/username/Scripts

# Copies the scripts files
COPY . /home/username/

# Update packages
RUN sudo apt update && sudo apt upgrade -y

EXPOSE 1135 

RUN echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc

CMD ["/bin/bash"]