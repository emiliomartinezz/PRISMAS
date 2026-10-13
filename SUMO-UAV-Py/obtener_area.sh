python3 -c "
import sumolib
net = sumolib.net.readNet('MadridScenario/madrid.net.xml')
print(net.getBoundary())
"
