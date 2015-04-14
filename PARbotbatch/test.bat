REM @echo off
REM Next command opens Microsoft Word
REM start /d "C:\Program Files\Microsoft Word" WINWORD.EXE
plink.exe parbot@192.168.1.100 -pw parbot /home/parbot/ros_workspace/src/parbot_ui/bash_scripts/ESTOP.sh
REM PAUSE 192.168.1.140
