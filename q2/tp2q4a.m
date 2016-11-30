% Systeme representant le chariot pour le TP2, Automne 2016
% (c) Philippe Giguere, 2016 Version 1.11
clear all
clearvars
clf
myFontSize = 14; % Taille de la police de caractere pour les graphes

% Parametre du systeme
%C=5;
% C=10;
C = 50;
Pwifi   = 10*10^(-3); % Puissance d'émission du routeur WiFi
d_init  = 6;     % Point de depart du robot
SRecepteur = 0.4*10^(-3);  % Bruit sur la puissance du récepteur radio, en mW.
SV      = 0.25;      % Bruit sur le voltage du moteur. NE JAMAIS CHANGER!
nStep   = 400;      % Nombres de mesures/pas
dT      = 0.1;     % Intervalle de temps entre les mesures/pas

% Valeur des bruits utilises dans le filtre a particules
SVEstime = 10*SV;   % Valeur du bruit sur la commande de deplacement des particules
SRecepteurEstime = 10*SRecepteur;% Valeur du bruit sur le recepteur des particules

% Important! Il faut initialiser la matrice X de facon a avoir
% une dimension (2,1) et non (1,2). Sinon, le filtre EKF ne marchera pas.
xVrai = [d_init 0]'; % Etat reel, inconnu du filtre et du robot.

nParticules = C;
X(1,:) = xVrai(1)*ones(1,nParticules); % position en x
X(2,:) = xVrai(2)*ones(1,nParticules); % position en y
w = ones(1,nParticules)/nParticules; % Poids egaux pour les particules


for iStep = 1:nStep
    % Simulation du systeme a chaque etape
    time = iStep*dT;
    
    % Commande de voltage envoye vers le systeme.
    U =  3*cos(0.2*pi*time);
    
    % ============== Debut de la simulation du deplacement reel ===========
    % Le deplacement veritable du chariot selon les equations.
    % Je vous donne les equations, vous n'avez rien a changer ici.
    
    xVrai(1) = xVrai(1) + (xVrai(2))*dT; % Calcul du deplacement
    xVrai(2) = 1.5*atan((U+SV*randn)/2);   % Calcul de la vitesse
    
    % Je simule pour vous la réponse du récepteur radio du chariot.
    % J'ajoute une valeur absolue car la puissance ne peut pas etre
    % negative. Cela n'affectera pas beaucoup le systeme.
    z = Pwifi*exp(-xVrai(1)/10) + SRecepteur*randn;
    % =============== Fin de la simulation de deplacement reel ============
    
    % ================ Debut de votre filtre EKF ou particule ==================
    % ATTENTION ATTENTION ATTENTION ATTENTION
    % Vous n'avez pas le droit d'utiliser xVrai dans votre filtre
    % car c'est la position et la vitesse reele du systeme, et
    % elles vous sont inconnues.
    
    for iParticule = 1:nParticules
        X(1,iParticule) = X(1,iParticule) + X(2,iParticule)*dT;
        X(2,iParticule) = 1.5*atan((U+SVEstime*randn)/2);
        
        % Mise-a-jour des poids
        wnew = gauss(Pwifi*exp(-X(1,iParticule)/10) -z,SRecepteurEstime);
        w(iParticule) = w(iParticule).*wnew;
        % k = waitforbuttonpress;
    end
    [X, w] = ParticuleResampling(X,w,0.5);
    
    Xmoyen = sum(X(1,:).*w);
    % ========= Fin des equations du filtre EKF ou particule =============
    
    % Cueillette des donnees pour les graphiques/statistiques
    AxVrai1(iStep)  = xVrai(1);
    AxVrai2(iStep)  = xVrai(2);
    AX1(iStep)      = Xmoyen;
    AX2(iStep)      = X(2);
    AU(iStep)       = U;
    AZ(iStep)       = z;
    ATime(iStep) = time;
    
    % Pour voir votre filtre evoluer dans le temps
    clf
    h(1) = plot(ATime,AX1,'go');
    hold on;
    h(2) = plot(ATime,AxVrai1,'k-','LineWidth',2);
    h(3) = plot(ATime,-10*log(AZ/Pwifi),'r*'); % Ici on peut inverser le capteur, pour trouver la position correspondant a z.
    xlabel('Temps (s)');
    ylabel('Estime de position (m)');
    legend(h,{'EKF','Position Exacte','Mesure h_z^{-1}'});
    message = sprintf('Le filtre à particule avec la position initial connue et C = %d',C);
    title(message);
    ylim([0 20]);
    drawnow();
end
