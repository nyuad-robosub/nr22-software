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

# Check if user wants to create conda environments
if utils_prompt_user "Create conda environments? (recommended for isolation from system Python, skip this step if you don't have conda installed)" y; then
    # Check if user wants to create environments in custom location
    CUSTOM_ENVS=false
    if utils_prompt_user "Install environments in custom location? (useful if you run out of storage in /home)"; then
        CUSTOM_ENVS=true
        read -p "Enter environment path (no whitespaces): "
        CUSTOM_ENVS_DIR=$REPLY
        conda config --append envs_dirs $CUSTOM_ENVS_DIR
    fi

    # Check if user wants to create a Python 2 environment
    if utils_prompt_user "Create Python 2.7 environment? (main environment for catkin & ardupilot)" y; then
        read -p "Give your Python 2.7 environment a name (no whitespaces): "
        ENV_NAME_PY27=$REPLY

        if $CUSTOM_ENVS; then
            conda create --prefix=$CUSTOM_ENVS_DIR/$ENV_NAME_PY27 python=2.7 -y
        else
            conda create -n $ENV_NAME_PY27 python=2.7 -y
        fi
        
        # Check if user wants to activate new environment on startup
        SHELL_INIT=".bashrc"
        exportline="conda activate $ENV_NAME_PY27";
        eval $exportline
        grep -Fxq "$exportline" ~/$SHELL_INIT 2> /dev/null || {
            if utils_prompt_user "Automatically activate ($ENV_NAME_PY27) on startup? (will replace the default conda environment (base) if set earlier)" y; then
                conda config --set auto_activate_base false
                echo $exportline >> ~/$SHELL_INIT
                echo "Done!"
            fi
        }
    fi

    # Check if user wants to create a Python 3 environment
    if utils_prompt_user "Create Python 3.7 environment? (environment for PyTorch & YOLO)" y; then
        read -p "Give your Python 3.7 environment a name (no whitespaces): "
        ENV_NAME_PY37=$REPLY

        if $CUSTOM_ENVS; then
            conda create --prefix=$CUSTOM_ENVS_DIR/$ENV_NAME_PY37 python=3.7 -y
        else
            conda create -n $ENV_NAME_PY37 python=3.7 -y
        fi

        # Install PyTorch & dependencies
        if utils_prompt_user "Install PyTorch for YOLO? (might not work with incompatible CUDA version)" y; then
            conda activate $ENV_NAME_PY37
            echo "To use YOLO in simulation, change $(tput bold)model_type$(tput sgr0) and $(tput bold)subprocess_python_file$(tput sgr0) in faux_detection.launch in the faux_detection package."
            read -p "Enter your CUDA version with no dot (e.g. 116 for CUDA 11.6): "
            pip3 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu$REPLY
            pip3 install --upgrade opencv-python pandas psutil pyyaml tqdm ultralytics gitpython>=3.1.30
            conda activate $ENV_NAME_PY27
        fi
    fi
fi

python --version
cd $NR22_DIRECTORY