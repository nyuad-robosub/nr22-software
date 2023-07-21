#!/bin/bash
# Recommend Miniconda install as will be useful in many cases
read -p "Install Miniconda? (allows multiple Python environments) [N/y]"
if [[ $REPLY =~ ^[Yy]$ ]]
then
    # If curl is not installed yet
    sudo apt-get install -y curl
    # Install silent mode 
    # https://docs.anaconda.com/free/anaconda/install/silent-mode/
    wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-$(uname -m).sh -LO $HOME/miniconda3.sh
    bash $HOME/miniconda3.sh -b -p $HOME/miniconda3
    rm $HOME/miniconda3.sh
    eval "$($HOME/miniconda3/bin/conda shell.bash hook)"
    conda init

    # Check if user wants to activate conda on startup
    read -p "Automatically activate conda (base) on startup? [N/y]"
    if [[ $REPLY =~ ^[Yy]$ ]]
    then
        echo "Done!"
    else
        conda config --set auto_activate_base false
    fi
fi

# Check if user wants to create a Python 2 environment
read -p "Create a conda environment with Python 2.7? (recommended for isolation from system Python, skip this step if you don't have conda installed) [N/y]"
if [[ $REPLY =~ ^[Yy]$ ]]
then
    read -p "Give your environment a name (no whitespaces): "
    ENV_NAME=$REPLY
    conda create -n $ENV_NAME python=2.7
    
    # Check if user wants to activate new environment on startup
    SHELL_INIT=".bashrc"
    exportline="conda activate $ENV_NAME";
    eval $exportline
    grep -Fxq "$exportline" ~/$SHELL_INIT 2> /dev/null || {
        read -p "Automatically activate ($ENV_NAME) on startup? (will replace the default conda environment (base) if set earlier) [N/y]"
        if [[ $REPLY =~ ^[Yy]$ ]]; then
            conda config --set auto_activate_base false
            echo $exportline >> ~/$SHELL_INIT
            echo "Done!"
        fi
    }
fi