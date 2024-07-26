import sys 
import subprocess
import os
import time

def restart_program():
    """Restart the current program"""
    print('[INFO] Restart')
    time.sleep(3)

    if os.name == 'nt':  # Windows
        # Running the bat file
        subprocess.Popen([fr"{os.getcwd()}\\restart.bat"])
    else:  # Linux or other UNIX-like systems
        script_path = os.path.abspath(os.path.join(os.getcwd(), 'restart.sh'))
        subprocess.Popen([script_path])
        