#!/bin/bash

while true; do
    # Create a menu with whiptail
    CHOICE=$(whiptail --title "Tmux sessions" --menu "Choose the session to run:" 15 50 5 \
        "1" "Timer example" \
        "2" "Publisher-Subscriber example" \
        "3" "Exit" 3>&1 1>&2 2>&3)

    # Handle user choices
    case "$CHOICE" in
        1)
            ./start.sh timer_session
            exit 0
            ;;
        2)
            ./start.sh publisher_session
            exit 0
            ;;
        3)
            exit 0
            ;;
        *)
            whiptail --title "Invalid Option" --msgbox "Please select a valid option!" 10 50
            ;;
    esac
done
