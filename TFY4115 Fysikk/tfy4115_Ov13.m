% TFY4115 Øving 13
% Veggisolasjon

e=1;                    % Emissiviteten [1]
TI=290;                 % Innetemperaturen [K]
TY=265;                 % Utetemperaturen [K]
nplatesmin=2;           % Minimum antall isolerende plater
nplatesmax=50;         % Maksimum antall isolerende plater

sigma=5.6703e-8;        % Stefan-Boltzmann konstanten [J s^-1 m^-2 K^-4]
j=zeros(1,nplatesmax);  % Allokerer vektor som skal holde 
                        % varmestrømtetthetene for forskjellige antall 
                        % plater

% Finn varmestrømmen for hvert antall av isolerende plater
for nplates=nplatesmin:nplatesmax
    % Setter opp den tridiagonale matrisa S
    D=sparse(1:nplates,1:nplates,-2*ones(1,nplates),nplates,nplates);
    E=sparse(2:nplates,1:nplates-1,ones(1,nplates-1),nplates,nplates);
    S=E+D+transpose(E);
    % Setter opp konstantvektoren b
    b=zeros(nplates,1);
    b(1)=-TI.^4;
    b(nplates)=-TY.^4;
    % Løser ligningssystemet Sx=b
    x=S\b;
    % Regner ut varmestrømtettheten for dette antall isolerende plater
    j(nplates)=e.*sigma.*(x(nplates)-TY.^4);
end

% Plott varmestrømtettheten mot antall plater
figure(1); % Figurnummer
subplot(2,1,1); % 2 subplot over hverandre
plot(j(nplatesmin:nplatesmax));
xlabel('Antall isolerende plater n');
ylabel('Varmestrømtetthet j(n) [W/m^2]');

% Plott log(j(n)) mot log(n)
subplot(2,1,2);
plot(log(nplatesmin:nplatesmax),log(j(nplatesmin:nplatesmax)));
xlabel('log(n)');
ylabel('log(j(n))');
grid on