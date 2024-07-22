import sys 
import subprocess
import os
import time

def restart_program():
    """Restart the current program"""
    print('[INFO] Restart')
    time.sleep(3)

    # Running the bat file
    subprocess.Popen([fr"{os.getcwd()}\\restart.bat"])
