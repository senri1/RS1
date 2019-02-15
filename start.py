import subprocess
import shlex
import sys

final_data = []

g_string = ['Path travel cost','Total angle turned in degrees','Distance Travelled','Theta Turned','Time =']
def run_command2(command):
    process = subprocess.Popen(command, shell = True, stdout=subprocess.PIPE)
    while True:
        output = process.stdout.readline()
        if output == "" and process.poll() is not None:
            break
        for i in range(len(g_string)):
            if g_string[i] in output:
                final_data.append(output)
                sys.stdout.write(output)
                sys.stdout.flush()
    rc = process.poll()
    return rc

cmd1 = 'roslaunch comp0037_cw1 factory_scenario.launch'
cmd2 = 'roslaunch comp0037_cw1 custom_scenario.launch'
cmd3 = 'roslaunch comp0037_cw1 improved_scenario.launch'
cmd4 = 'roslaunch comp0037_cw1 improved2_scenario.launch'
# run_command2(cmd1)
# run_command2(cmd2)
run_command2(cmd3)
# run_command2(cmd4)

