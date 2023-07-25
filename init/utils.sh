#!/bin/bash
# Get default argument (-d)
DEFAULT=false
if [[ $1 == "-d" ]]; then
    DEFAULT=true
fi

# Function to prompt user
# Params: $1: question string
#         $2: default choice (either y/Y or n/N)
# Returns: 0 means success
function utils_prompt_user() {
    # Default with no options: No
    DEF_STRING="[y/N]"
    DEF_ANSWER=$2
    # Return if default is set
    if [[ $DEF_ANSWER =~ ^[Yy]$ ]]; then
        if $DEFAULT; then
            return 0
        fi
        DEF_STRING="[Y/n]"
    elif [[ $DEF_ANSWER =~ ^[Nn]$ ]]; then
        if $DEFAULT; then
            return 1
        fi
        DEF_STRING="[y/N]"
    fi
    
    # Ask question
    read -p "$1 $DEF_STRING"
    # "Yes" answered
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        return 0
    # "No" answered
    elif [[ $REPLY =~ ^[Nn]$ ]]; then
        return 1
    # Default answer
    else
        echo "Default answer '$(echo $DEF_STRING | grep -o '[A-Z]')' selected"
        if [[ $DEF_ANSWER =~ ^[Yy]$ ]]; then
            return 0
        else
            return 1
        fi
    fi
}

# if utils_prompt_user "yes default q?" y; then
#     echo "Yes selected"
# else
#     echo "No selected"
# fi

# if utils_prompt_user "no default q?" n; then
#     echo "Yes selected"
# else
#     echo "No selected"
# fi

# if utils_prompt_user "yes default q?" Y; then
#     echo "Yes selected"
# else
#     echo "No selected"
# fi

# if utils_prompt_user "no default q?" N; then
#     echo "Yes selected"
# else
#     echo "No selected"
# fi

# if utils_prompt_user "non-default q?"; then
#     echo "Yes selected"
# else
#     echo "No selected"
# fi
