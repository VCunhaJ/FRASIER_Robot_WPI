start /d "C:\PARbotbatch" plink.exe parbot@192.168.1.100 -pw parbot /home/parbot/ros_workspace/src/parbot_ui/bash_scripts/Shutdown.sh
shutdown -s
