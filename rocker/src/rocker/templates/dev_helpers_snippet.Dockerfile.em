# workspace development helpers
RUN apt-get update \
 && apt-get install -y \
    byobu \
    apt-utils \
    wget \
    gpg \
    apt-transport-https \
 && apt-get clean

# Install VS Code
RUN wget -qO- https://packages.microsoft.com/keys/microsoft.asc | gpg --dearmor > packages.microsoft.gpg \
 && install -D -o root -g root -m 644 packages.microsoft.gpg /etc/apt/keyrings/packages.microsoft.gpg \
 && echo "deb [arch=amd64,arm64,armhf signed-by=/etc/apt/keyrings/packages.microsoft.gpg] https://packages.microsoft.com/repos/code stable main" > /etc/apt/sources.list.d/vscode.list \
 && rm -f packages.microsoft.gpg \
 && apt-get update \
 && apt-get install -y code \
 && apt-get clean