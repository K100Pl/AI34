# SY28/AI34 - Systèmes Cyber-Physiques Multi-Robots

Simulateur Robotarium pour le projet SY28/AI34 : Navigation coopérative, estimation d'état et robustesse des communications.

## Structure du Projet

```
AI34/
├── Obstacle.m                    # Script principal (Q1-Q4)
├── init.m                        # Initialisation des paths
├── utilities/
│   ├── controllers/
│   │   ├── create_smooth_waypoint_controller.m   # Contrôleur Bézier (Q1)
│   │   └── ...
│   ├── barrier_certificates/     # Évitement de collisions
│   ├── graph/                    # Topologies de graphes
│   └── transformations/          # SI <-> Unicycle
├── Response_II_i/                # Odométrie parfaite
│   ├── Obstacle_PerfectOdometry.m
│   └── reponse_II_i.txt
├── Response_II_ii/               # Odométrie bruitée
│   ├── Obstacle_NoisyOdometry.m
│   └── reponse_II_ii.txt
├── Response_II_iii/              # Correction par landmarks
│   ├── Obstacle_Landmarks.m
│   └── reponse_II_iii.txt
├── Response_Q2_iii/              # Comparaison rigide vs non-rigide
│   ├── Obstacle_NonRigid.m
│   └── comparison_rigid_vs_nonrigid.txt
├── Response_Q5/                  # Bruit localisation/communication
│   ├── Obstacle_NoisyLocalization.m
│   ├── Obstacle_NoisyCommunication.m
│   └── reponse_Q5.txt
├── Response_I_6_i/               # Formation triangulaire
│   └── reponse_I_6_i.txt
└── examples/                     # Exemples Robotarium
```

## Questions Implémentées

### Partie I : Navigation et Formation

| Question | Description | Fichier |
|----------|-------------|---------|
| Q1 | Navigation fluide (courbes de Bézier) | `utilities/controllers/create_smooth_waypoint_controller.m` |
| Q2 | Formation diamant rigide (2N-3 edges) | `Obstacle.m` |
| Q2.iii | Comparaison rigide vs non-rigide | `Response_Q2_iii/Obstacle_NonRigid.m` |
| Q3 | Évitement d'obstacles | `Obstacle.m` |
| Q4 | Reconfiguration diamant/platoon | `Obstacle.m` |
| Q5 | Impact bruit localisation/communication | `Response_Q5/` |
| Q6.i | Formation triangulaire (N=3) | `Response_I_6_i/reponse_I_6_i.txt` |

### Partie II : Localisation et Estimation d'État

| Question | Description | Fichier | Résultats |
|----------|-------------|---------|-----------|
| II.i | Odométrie parfaite (baseline) | `Response_II_i/Obstacle_PerfectOdometry.m` | Erreur: 0.0001 m |
| II.ii | Odométrie bruitée (drift) | `Response_II_ii/Obstacle_NoisyOdometry.m` | Erreur: 0.175 m |
| II.iii | Correction par landmarks | `Response_II_iii/Obstacle_Landmarks.m` | Erreur: 0.086 m (-51%) |

## Paramètres Clés

### Formation Diamant (N=5)
- Topologie : 7 edges (minimalement rigide)
- Leader : Robot 1
- Followers : Robots 2-5 avec offsets rotés selon θ_leader

### Odométrie (Partie II)
- σ_v = 0.01 m/s (bruit vitesse linéaire)
- σ_ω = 0.02 rad/s (bruit vitesse angulaire)
- dt = 0.033 s (Robotarium ~30Hz)

### Landmarks (II.iii)
- 2 landmarks : (-0.8, 0.4) et (0.8, -0.4)
- Rayon d'observation : 0.25 m
- Méthode : Reset simple (pas Kalman)

## Utilisation

```matlab
% 1. Initialiser les paths
run('init.m')

% 2. Exécuter le script principal
run('Obstacle.m')

% 3. Ou exécuter une variante spécifique
run('Response_II_ii/Obstacle_NoisyOdometry.m')
```

## Prérequis

- MATLAB R2014b ou supérieur
- Optimization Toolbox (pour `quadprog`)

## Références

- Vilca et al. (2015) - Courbes de Bézier pour navigation fluide
- Adouane - Formation control et graphes rigides
