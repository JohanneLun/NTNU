# TFY4115 Øving 8, oppgave 1d.
# Verlet-integrasjon av svingeligningen med dempning.

# Definerer parametrene (kan endres før hver kjøring av programmet):
#-------------------------------------------------------------------
import numpy as np
import matplotlib.pyplot as plt
import math
import time
# Pakkene numpy og matplotlib må installeres manuelt i f.eks IDLE eller pyCharm før scriptet kan kjøre.

omega0 = 1       
gamma = 0.05   # Typen dempning avgjøres om gamma < = > omega0
dt=0.1        # Tidsinkrementet
t = np.arange(0,10,dt)  # Tidsintervallet vi integrerer over t=0:dt:10.0   
x=0*t         # Initialiserer posisjonsvektoren til dimensjon lik antall t.
x[0]=1.0      # Kulas startposisjon.
x[1]=1.0      # Neste posisjon, hvis ulik x(0) er starthastigheten ulik 0                 

# Numerisk løsning:
#------------------

aa=(2-omega0**2*dt**2)/(1+gamma*dt) # Definerer forenklende parametre.
bb=(1-gamma*dt)/(1+gamma*dt)

for n in range(1,len(t)-1):    #for n=2:(length(t)-1)
    x[n+1]=aa*x[n]-bb*x[n-1] # Verlet-integrasjonen

plt.plot(t,x,'b', label='numerisk') # Numerisk løsning blå strek 
plt.title(r'Dempa pendel')
plt.ylabel(r'$x$')
plt.xlabel(r'$t/$s')
plt.legend(loc='best')

# Vi sammenligner nå med de analytiske løsninger
# OBS: Uttrykkene er riktige bare når starthastighet = 0 , dvs. x(0)=x(1)
#------------------
if (gamma < omega0): # Underkritisk demping  
    omegad=math.sqrt(omega0**2-gamma**2)
    phi=math.atan(-gamma/omegad) # Fasevinkel, antar starthastighet = 0
    aa=x[1]/math.cos(phi)  # Amplitude A
    for n in range(0,len(t)-1):    
      x[n]=aa*math.exp(-gamma*t[n])*math.cos(omegad*t[n]+phi)

if (gamma > omega0): # Overkritisk demping gamma > omega0 
    omegad=math.sqrt(gamma**2-omega0**2)
    aa=x[1]*0.5*(1+gamma/omegad) # Amplitudeverdi A, antar starthastighet = 0
    bb=x[1]*0.5*(1-gamma/omegad)# Amplitudeverdi B, antar starthastighet = 0
    for n in range(0,len(t)-1):
      x[n]=math.exp(-gamma*t[n])*(aa*math.exp(omegad*t[n])+bb*math.exp(-omegad*t[n]))
if (gamma == omega0): # Kritisk demping
    aa=x[1] # Amplitudeverdi A
    bb=x[1]*gamma # Amplitudeverdi B, antar starthastighet = 0
    for n in range(0,len(t)-1):
      x[n]=(aa + bb*t[n]) * math.exp(-gamma*t[n])
    
plt.plot(t,x,'r', label='analytisk') # Analytisk løsning rød strek
plt.legend(loc='best')
plt.show()   


