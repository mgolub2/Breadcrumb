from synapse.switchboard import *

@setHook(HOOK_STARTUP)
def startupEvent():
    initUart(1, 1) # <= put your desired baud rate here!
    flowControl(1, False) # <= set flow control to True or False as needed
    #crossConnect(DS_UART1, DS_STDIO) # Connect UART to stdio
    
    crossConnect(DS_UART1, DS_TRANSPARENT) # Connect 
    
#When data is received over uart or network
#@setHook(HOOK_STDIN)
#def stdinRec(data):
#    print(data)