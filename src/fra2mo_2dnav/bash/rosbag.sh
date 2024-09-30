
#!/bin/bash

# Specifica il nome del file finale
FINAL_BAGFILE="/home/lucabor/Scrivania/RoboticsLab/src/fra2mo_2dnav/rosbag/Path.bag"

# Avvia la registrazione
rosbag record -O "$FINAL_BAGFILE" /fra2mo/pose