% Systeme representant le chariot pour le TP2, Automne 2016
% (c) Philippe Giguere, 2016 Version 1.11
clearvars
clf
myFontSize = 14; % Taille de la police de caractere pour les graphes

% Parametre du systeme
Pwifi   = 10;      % Puissance d'émission du routeur WiFi
d_init  = 6;     % Point de depart du robot
SRecepteur = 0.4*10^(-3);  % Bruit sur la puissance du récepteur radio, en mW.
SV      = 0.25;      % Bruit sur le voltage du moteur. NE JAMAIS CHANGER!
nStep   = 400;      % Nombres de mesures/pas
dT      = 0.1;     % Intervalle de temps entre les mesures/pas

% Important! Il faut initialiser la matrice X de facon a avoir
% une dimension (2,1) et non (1,2). Sinon, le filtre EKF ne marchera pas.
xVrai = [d_init 0]'; % Etat reel, inconnu du filtre et du robot.

% Specifier les valeurs initiales des matrices.
% Ne pas oublier qu'ici, ce sont des covariances, pas des ecarts-types.
X = [d_init 0]'; 
P = 

% Matrices de bruits
Cv = [SV^2];           % C'est la valeur à changer dans l'énoncé
Cw = [SRecepteur^2];   % Matrice de covariance sur les mesures

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

    Votre code ici!
    
    % ========= Debut des equations du filtre EKF =============
    % Ces equations sont standard. Vous n'avez donc pas a
    % les modifier. Les retirer simplement pour un filtre à particules.
    P = F*P*F' + Cv;  % Propagation covariance

    % ======== Mise-a-jour ========
    K = P*H'/(H*P*H'+Cw);   % matrice [3x1]
    r = (z-zhat);           % matrice [1]
    X = X + K*r;
    P = (eye(size(P))-K*H)*P;
    % ========= Fin des equations du filtre EKF ou particule =============
    
    % Cueillette des donnees pour les graphiques/statistiques
    AxVrai1(iStep)  = xVrai(1);
    AxVrai2(iStep)  = xVrai(2);
    AX1(iStep)      = X(1);
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
    ylim([0 20]);
    drawnow();
end
