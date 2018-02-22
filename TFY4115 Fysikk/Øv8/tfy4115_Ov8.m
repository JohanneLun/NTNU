% TFY4115 Øving 8, oppgave 1d.
% Verlet-integrasjon av svingeligningen med dempning.

% Definerer parametrene (kan endres før hver kjøring av programmet):
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
omega0 = 1;       
gamma = 0.1;   % Typen dempning avgjøres om gamma < = > omega0
dt=0.1;        % Tidsinkrementet
t=0:dt:10;     % Tidsintervallet vi integrerer over
x=0*t;         % Initialiserer posisjonsvektoren til dimensjon lik antall t.
x(1)=1.0;      % Kulas startposisjon.
x(2)=1.0;      % Neste posisjon, hvis ulik x(1) er starthastigheten ulik 0                 

% Numerisk løsning:
%%%%%%%%%%%%%%%%%%%%%

aa=(2-omega0^2*dt^2)/(1+gamma*dt); % Definerer forenklende parametre.
bb=(1-gamma*dt)/(1+gamma*dt);

for n=2:(length(t)-1)
    x(n+1)=aa*x(n)-bb*x(n-1); % Verlet-integrasjonen
end

plot(t,x); % Plotter numerisk løsning blå strek
hold on    % Følgende plott i samme plot som tidligere

% Vi sammenligner nå med de analytiske løsninger
% OBS: Uttrykkene er riktige bare når starthastighet = 0 , dvs. x(1)=x(2)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if gamma < omega0 % Underkritisk demping  
    omegad=sqrt(omega0^2-gamma^2);
    phi=atan(-gamma/omegad); % Fasevinkel, antar starthastighet = 0
    aa=x(1)/cos(phi);  % Amplitude A
    for n=1:(length(t)-1)
      x(n)=aa*exp(-gamma*t(n))*cos(omegad*t(n)+phi);
      end

elseif gamma > omega0 % Overkritisk demping gamma > omega0 
    omegad=sqrt(gamma^2-omega0^2);
    aa=x(1)*0.5*(1+gamma/omegad); % Amplitudeverdi A, antar starthastighet = 0
    bb=x(1)*0.5*(1-gamma/omegad);% Amplitudeverdi B, antar starthastighet = 0
    for n=1:(length(t)-1)
      x(n)=exp(-gamma*t(n))*(aa*exp(omegad*t(n))+bb*exp(-omegad*t(n)));
      end

else % Kritisk demping
    aa=x(1); % Amplitudeverdi A
    bb=x(1)*gamma; % Amplitudeverdi B, antar starthastighet = 0
    for n=1:(length(t)-1)
      x(n)=(aa + bb*t(n)) * exp(-gamma*t(n));
      end
end 
    
plot(t,x,'r');  % Plotter analytisk løsning rød strek
hold off