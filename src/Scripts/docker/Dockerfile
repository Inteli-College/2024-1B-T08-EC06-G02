# Use the official Ubuntu 22.04 base image
FROM ubuntu:22.04

# Set the environment variables to avoid interaction while installing packages
ENV DEBIAN_FRONTEND=noninteractive
ENV LANG pt_BR.UTF-8
ENV LANGUAGE pt_BR:pt
ENV LC_ALL pt_BR.UTF-8
ENV TZ=America/Sao_Paulo

# Update the package lists and install some useful tools and locales
RUN apt-get update && apt-get install -y \
    sudo \
    locales \
    curl \
    wget \
    nano \
    vim \
    x11-apps \
    && rm -rf /var/lib/apt/lists/*

# Set up locale
RUN sed -i -e 's/# pt_BR.UTF-8 UTF-8/pt_BR.UTF-8 UTF-8/' /etc/locale.gen && \
    locale-gen

# Install required packages for GUI support
RUN apt-get update && apt-get install -y \
    x11-apps \
    && rm -rf /var/lib/apt/lists/*

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
COPY prepare_ros.sh /home/username/Scripts

# Default command to run bash shell
CMD ["/bin/bash"]
