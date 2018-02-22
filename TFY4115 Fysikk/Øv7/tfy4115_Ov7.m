% TFY4115 �ving 7, Oppgave 4
% Verlet-integrasjon av tyngdependel uten demping.
% Vi sammenligner ikke-line�re l�sning med den lineariserte.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear;            % Nullstille alle variabler og deres dimensjoner
% Definerer parametrene, startvinkelen theta0grad endres f�r hver kj�ring:
omega0 = 1.0;   
dt=0.001;         % Tidsinkrementet
t=0:dt:20.0;      % Tidsintervallet vi integrerer over
theta=0*t;        % Initialiserer vinkel theta for ikke-line�re ligning.
thetalin=0*t;     % Initialiserer vinkel theta for line�re ligning.
theta0grad = 20;  % Startvinkel (startamplitude) for pendelen i GRADER
theta0 = theta0grad*pi/180; % Starvinkel i RADIANER

% Startposisjon, antar null initiell hastighet dvs. theta(1) = theta(2):
theta(1)=theta0; % Ikke-line�re
theta(2)=theta0;
thetalin(1)=theta0; % Line�re
thetalin(2)=theta0;

% Definerer forenklede parametre:
aa = 2-omega0^2*dt^2;
dd = omega0^2*dt^2;
% Hjelpeparametre for � finne nullgjennomgang:
zero1=0;
zero2=0;
zerolin1=0;
zerolin2=0;

% Verlet-integrasjonen og samtidig finne nullpunktene:
for n=2:(length(t)-1)
   theta(n+1)=2*theta(n)-theta(n-1)-dd*sin(theta(n)); % Ikke-line�re
      % Sjekker f�rste og andre nullgjennomgang pos -> neg:
      if (theta(n)>0) && (theta(n+1)<0) && (zero1>0) && (zero2==0) zero2=n; end;
      if (theta(n)>0) && (theta(n+1)<0) && (zero1==0) zero1=n; end;
   thetalin(n+1)=aa*thetalin(n)-thetalin(n-1);  % Line�re
      % Sjekker f�rste og andre nullgjennomgang pos -> neg:
      if (thetalin(n)>0) && (thetalin(n+1)<0) && (zerolin1>0) && (zerolin2==0) zerolin2=n; end;
      if (thetalin(n)>0) && (thetalin(n+1)<0) && (zerolin1==0) zerolin1=n; end;
end

plot(t,theta); % Bl� strek ikke-line�r l�sning
hold on
plot(t,thetalin,'r'); % R�d strek line�r l�sning
hold off

% Beregning av periode fra nullgjennomgang for theta og thetalin:
periode = (zero2-zero1)*dt;
periodelin = (zerolin2-zerolin1)*dt;
% Rapporter til skjermen:
fprintf(1,' ampl =%4.0f gr  ',theta0grad);
fprintf(1,'  T =%7.4f',periode);
fprintf(1,'  Tlin =%7.4f',periodelin);
fprintf(1,'  T/Tlin =%9.6f\n',periode/periodelin);

% Slutt program