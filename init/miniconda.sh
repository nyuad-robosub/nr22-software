#!/bin/bash
# Script description
# | This script prompts you to install Miniconda and set up a Python 2
# | environment for later uses. Also prompts you if you want the
# | Python 2 environment to be default.

NR22_DIRECTORY=$PWD

# Recommend Miniconda install as will be useful in many cases
if utils_prompt_user "Install Miniconda? (allows multiple Python environments)" y; then
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
    if utils_prompt_user "Automatically activate conda (base) on startup?" n; then
        echo "Done!"
    else
        conda config --set auto_activate_base false
    fi
fi

# Check if user wants to create a Python 2 environment
if utils_prompt_user "Create a conda environment with Python 2.7? (recommended for isolation from system Python, skip this step if you don't have conda installed)" y; then
    read -p "Give your environment a name (no whitespaces): "
    ENV_NAME=$REPLY
    conda create -n $ENV_NAME python=2.7
    
    # Check if user wants to activate new environment on startup
    SHELL_INIT=".bashrc"
    exportline="conda activate $ENV_NAME";
    eval $exportline
    grep -Fxq "$exportline" ~/$SHELL_INIT 2> /dev/null || {
        if utils_prompt_user "Automatically activate ($ENV_NAME) on startup? (will replace the default conda environment (base) if set earlier)" y; then
            conda config --set auto_activate_base false
            echo $exportline >> ~/$SHELL_INIT
            echo "Done!"
        fi
    }
fi

python --version
cd $NR22_DIRECTORY