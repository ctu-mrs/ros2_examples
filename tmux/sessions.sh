#!/bin/bash

# Create a menu with whiptail
CHOICE=$(whiptail --title "Tmux sessions" --menu "Choose the session to run:" 15 50 5 \
    "1" "Timer example" \
    "2" "Publisher-Subscriber example" \
    "3" "Service-Client example" 3>&1 1>&2 2>&3)

# Handle user choices
case "$CHOICE" in
    1)
        ./start.sh timer_session
        exit 0
        ;;
    2)
        ./start.sh pubsub_session
        exit 0
        ;;
    3)
        ./start.sh service_session
        exit 0
        ;;
esac
exit 0