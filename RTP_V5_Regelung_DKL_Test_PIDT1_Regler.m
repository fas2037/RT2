% Praktikum Regelungstechnik 1
% Regelung der Drosselklappe
% Versuch 4
% R. Gregor. R. Hermann, C. Birkner, 2019

close all;
clear all;
clc;

%%
 % ***************************************************
% GM-Parameter - NICHT ÄNDERN
% 
Ra     = 3.5;    % Ankerwiderstand kl. Maschine
La     = 1.6e-3; % Ankerinduktivität
cphi   = 0.03;   % Drehmomentkonstante 
J      = 7.5e-6; % Massenträgheitsmoment
i_Getr = 1/20;   % Getriebeübersetzung
U_mot  = 12;     % Spannung Bordnetz KFZ
sat    = 1;      % Pegel des Saettigungsblocks (PWM-Modul, +/-100%)

% ***************************************************
%%
% ***** Streckenmodell der Drosselklappe *****

% Übertragungsfunktion G_A: Ankerstromkreis R, L => I 
z1  = 1/Ra;               % Zählerpolynom          
n1  = [La/Ra, 1];            % Nennerpolynom
G_I = tf(z1, n1);           % Übertragungsfunktion

% Übertragungsfunktion G_M: I auf Moment (konstanter Faktor)
G_M = cphi;         

% Übertragungsfunktion G_om: Moment auf Winkelgeschwindigkeit
z3  = 1;                  
n3  = [J, 0];
G_Om = tf(z3, n3);

% rückgekoppelte Ankerquellspannung Uq
G_Rueck = cphi;

% Übertragungsfunktion mit rückgekoppelte Ankerquellspannung Uq
G_i = feedback(G_I*G_M*G_Om, G_Rueck);

% Üfkt. Omega Motor -> Omega DKL in [°/s]
z4 = i_Getr*180;       
n4 = pi;
G_OmDKL = tf(z4, n4);

% Üfkt. Omega DKL in [°/s] -> Drosselklappenwinkel in [°]
z5 = 1;       
n5 = [1,0];
G_posDKL = tf(z5, n5);


% Üfkt der gesamten Strecke; Faktor U_mot für PWM -> Spannung
G_s = G_i * G_OmDKL * G_posDKL * U_mot;

% *********Ende Streckenmodell ******************************************

%% Auslegung P-Regler

% Hinweis: Alle Parameter fuer den PIDT1-Regler anlegen, da diese im
% Modell eingetragen sind.
% Nur P-Anteil mit Strecke verbinden.

K  = 1;
Tv = 1;
Tn = 1;
T  = 1;

% Üfkt offener Kreis
Gr = K;

% ***** Ende P-Regler *****

% Weiter mit Abschnitt "WOK zeichnen und Reglerverstärkung K bestimmen"

%% Auslegung PIDT1-Regler
%  Zur Aktivierung des Blocks alle Zeilen des Blocks markieren und "Uncomment" drücken
% ***** PIDT1-Regler, Kr = 1, Tn = 0.1323, Tv = 0.0242, T = 0.004 => Nullstellen bei -10 und -30

T = 0.004;   %Parasitaere Zeitkonstante, nicht aendern!

% Vorgabe der Nullstellen bei -10 und -30
% Uebertragungsfunktion des PIDT1-Reglers
% Zaehler faktorisiert mit Tz1, Tz2        und  ausmultipliziert mit T, Tv und Tn
%         Kr(Tz1*s + 1)*(Tz2*s + 1)             Kr((T*Tn+Tn*Tv)*s^2 + (T+Tn)*s + 1)
%Gr(s) = ----------------------------   =   ---------------------------------------
%          Tn*s*(T*s + 1)                         Tn*s*(T*s + 1)

Tz1=1/10;   % Soll dominierende Nullstelle festlegen -> wichtig für Vorfilter
Tz2=1/30;

Tn = Tz1 + Tz2 - T;   % Berechnungsformel eingeben, nicht Zahlenwert!
Tv = Tz1*Tz2 / Tn - T;   % Berechnungsformel eingeben, nicht Zahlenwert!

% Kontrolle der Parameter
%  Tn = 0.1293, Tv = 0.0218

% Parametrierung der Regler-Uefkt fuer Simulation der Sprungantwort
zr = [T*Tn+Tn*Tv T+Tn 1]; 
nr = [T*Tn Tn 0];
% % ***********************

Gr = tf(zr, nr);           % Übertragungsfunktion des Reglers

% ***** Ende PIDT1-Regler *****

%%  WOK zeichnen und Reglerverstärkung K bestimmen
zf = 1;
nf = [Tz1 1];

Gfilt = tf(zf, nf);

Go = G_s * Gr;
%minreal(Go)             % bessere Darstellung der Übertragungsfunktion
zpk(Go)                  % Darstellung in faktorisierter WO-NF

%g = [0:1e-5:0.06 , 0.06:1e-3:50]; % Schrittweite zur besseren 
                                    % Darstellung Berechungen der WOK
%   rlocus(Go,g)          % WOK des offenen Kreises mit Schrittweite

figure ('Name', 'WOK fuer Regelkreis Drosselklappe');
rlocus(Go)          % WOK des offenen Kreises
% hold on;
grid on;

% Ablesen der gewünschten Reglerverstärkung aus der WOK
% für PIDT1 gilt: Pole am Verzweigungspunkt zwischen den Nullstellen 
K = 0.0508;                         % Reglerverstärkung aus WOK, dominierende Pole auf Verzweigungspunkt
Gw = feedback(Go*K, 1);       % Übertragungsfunktion des geschlossenen Regelkreises

% Erweiterung des Regelkreises (Aufgabe 2.4) hier vornehmen
% Übertragungsfunktion des geschlossenen Kreises für Sprungantwort erweitern 

% Parametrierung des Simulink-PIDT1-Reglers. Parameter nicht ändern.
P=K;
D=Tv*K;
I=K/Tn;
N=1/T;

% Kontrolle der Kreisschließung
pole(Gw)                     % Pole des geschlossenen RKs an richtiger Stelle?

figure('Name', 'Singularitaeten des geschlossenen Kreises'); 
pzmap(Gw);                   % PN_Plan des geschlossenen RK

figure('Name', 'Sprungantwort geschlossener Kreis');
step(Gw);             % Sprungantwort des geschlossenen Kreises ohne Nichtlinearitäten. Vorgaben erfüllt?
hold on;
grid on;

