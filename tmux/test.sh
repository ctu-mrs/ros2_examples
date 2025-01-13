#!/bin/bash

while true; do
    # Create a menu with whiptail
    CHOICE=$(whiptail --title "Shell Script Menu" --menu "Choose an option:" 15 50 5 \
        "1" "Display Date and Time" \
        "2" "List Files" \
        "3" "Show Disk Usage" \
        "4" "Display Logged-in Users" \
        "5" "Exit" 3>&1 1>&2 2>&3)

    # Handle user choices
    case $CHOICE in
        1)
            whiptail --title "Date and Time" --msgbox "Current Date and Time: $(date)" 10 50
            ;;
        2)
            FILES=$(ls -lh)
            whiptail --title "List of Files" --msgbox "Files in Current Directory:\n$FILES" 20 60
            ;;
        3)
            DISK_USAGE=$(df -h)
            whiptail --title "Disk Usage" --msgbox "Current Disk Usage:\n$DISK_USAGE" 20 60
            ;;
        4)
            USERS=$(who)
            whiptail --title "Logged-in Users" --msgbox "Currently Logged-in Users:\n$USERS" 20 60
            ;;
        5)
            whiptail --title "Exit" --msgbox "Goodbye!" 10 50
            exit 0
            ;;
        *)
            whiptail --title "Invalid Option" --msgbox "Please select a valid option!" 10 50
            ;;
    esac
done
