# CLAUDE.md — lorandi_assignament_2

Guide de contexte pour Claude Code. Lu automatiquement à chaque session.

## Le projet

Package ROS 2 (`lorandi_assignament_2`) — Assignment 2 du cours *Intelligent Robotics 2025/2026* (Université de Padoue).  
Un bras UR5 avec pince (Gazebo + MoveIt!) détecte deux cubes via des AprilTags, effectue un **swap** (échange de position) des deux cubes, puis retourne en position initiale.  
**Deadline : 18 janvier 2026, 23h59 CET.**

## ⚠️ Contexte critique : version de ROS 2

- **Le projet cible ROS 2 Humble (Ubuntu 22.04)** — version attendue par le correcteur.
- **La machine de dev actuelle tourne sous ROS 2 Jazzy (Ubuntu 24.04, WSL2).**
- Ne PAS corriger les références à Humble pour les passer à Jazzy sans validation explicite.
- Si un test local échoue, signaler l'écart de version avant de modifier le code.

## ⚠️ Orthographe du nom de package

Même convention que l'assignment 1 : **`lorandi_assignament_2`** (avec un « a » : assign**a**ment).  
Le dépôt GitHub utilisera `lorandi_assignement_2` (avec un « e ») — incohérence connue, ne pas renommer le package.

## ⚠️ Ne pas modifier `ir_2526`

Les packages `ir_launch`, `ir_movit_config`, `ir_desription`, `ir_base`, `apriltag_ros` sont fournis par le cours.  
**Ne jamais éditer ces fichiers.** Toute modification entraîne l'échec à l'examen.

## Paramètres clés de la simulation

| Élément | Valeur |
|---|---|
| AprilTag rouge | ID 1, taille 0.050×0.050 m |
| AprilTag bleu | ID 10, taille 0.050×0.050 m |
| Taille des cubes | 0.06×0.06×0.10 m (L×l×H) |
| Caméra externe | Topics : `/rgb_camera/image`, `/rgb_camera/camera_info` |
| Position caméra | Déjà déclarée dans l'arbre TF |
| Robot | UR5 avec pince Robotiq |

## Analyse du code du prof (à ne pas modifier !)

### Ce que `ir_2526` fournit

| Composant | Détail |
|---|---|
| `AprilTagNode` (C++) | Souscrit `image_rect` + `camera_info` (QoS sensor_data). Publie `detections` (AprilTagDetectionArray). Broadcast TF de chaque tag dans le frame caméra. |
| `short_joint_state_publisher` (C++) | Publie l'état initial des joints 5 s au démarrage, puis se termine. Normal. |
| `ir_base.launch.py` | Gazebo + UR5 + controllers + bridge caméra |
| `ir_movit.launch.py` | MoveIt! move_group + RViz (config `ir_movit_config`) |
| `assignment_2.launch.py` | Appelle ir_base + ir_movit. **N'inclut PAS le node AprilTag.** |

### Planning groups MoveIt! (SRDF `ur_gripper.srdf`)

- `ir_arm` : chaîne `base_link → tool0` (6 DOF)
- `ir_gripper` : Robotiq 85 — action `GripperCommand` sur `gripper_controller`
  - `open` : `robotiq_85_left_knuckle_joint = 0`
  - `close` : `robotiq_85_left_knuckle_joint = 0.8`
- Named state `home` : `shoulder_lift=-1.57`, `wrist_1=-1.57`, reste à 0

### Pièges critiques

1. **AprilTag non lancé par le prof** → on doit le lancer nous-mêmes
2. **Remapping obligatoire** : AprilTagNode souscrit `image_rect` mais le bridge expose `rgb_camera/image`
   → remap `image_rect → rgb_camera/image` et `camera_info → rgb_camera/camera_info`
3. **Config tags à surcharger** : le yaml par défaut a `size=0.068m` et `ids=[0,1,2]`
   → forcer `size=0.050m`, `ids=[1, 10]`
4. **TF des tags** : AprilTagNode broadcast les poses en frame caméra (`rgb_camera_link` ou similaire)
   → notre `tag_detector_node` doit utiliser TF2 pour convertir en frame `world`

## Architecture — Nodes

| Node | Rôle | Entrées | Sorties |
|---|---|---|---|
| `tag_detector_node.py` | Détecte les AprilTags, convertit les poses caméra→world via TF2, publie les positions des cubes | `/apriltag/detections`, TF tree | `/cube_poses` (PoseArray ou custom) |
| `cube_swapper_node.py` | Orchestrateur principal : machine à états qui pilote MoveIt! pour pick→place→swap→home | `/cube_poses`, MoveIt! | Actions MoveIt!, logs terminal |
| `color_detector_node.py` *(extra +3)* | Détecte la couleur de chaque cube via la caméra RGB | `/rgb_camera/image` | Log terminal en fin de simulation |

**Pipeline séquentiel :**
1. Détection des deux tags (ID 1 et ID 10) → poses monde via TF2
2. Calcul des positions de pick/place (dessus du cube = pose tag + offset Z hauteur cube/2)
3. Move to pre-grasp → descente → close gripper → lift
4. Swap : A→zone intermédiaire, B→pos(A), A→pos(B)
5. Retour en home position (`ir_arm` group, named state `home`)

**Notre launch file orchestre :**
```
lorandi_assignment_2.launch.py
├─ IncludeLaunch: ir_launch/assignment_2.launch.py   ← prof
├─ Node: apriltag (remap image_rect→rgb_camera/image)
├─ Node: tag_detector_node.py
├─ Node: cube_swapper_node.py
└─ Node: color_detector_node.py  (extra)
```

## Build & lancement

```bash
# Build (depuis la racine du workspace)
cd ~/ws_assignments
colcon build --packages-select lorandi_assignament_2
source install/setup.bash

# Lancement : simulation complète (ir_2526) + nos nodes
ros2 launch lorandi_assignament_2 lorandi_assignment_2.launch.py

# Lancement de la simulation seule (package du cours)
ros2 launch ir_launch assignment_2.launch.py
```

## Structure du package

```
lorandi_assignament_2/
├── scripts/
│   ├── tag_detector_node.py       # Détection AprilTag + TF2
│   ├── cube_swapper_node.py       # Orchestrateur MoveIt! (machine à états)
│   └── color_detector_node.py     # (extra) Détection couleur RGB
├── launch/
│   └── lorandi_assignment_2.launch.py  # Lance ir_launch + nos nodes
├── config/
│   └── apriltag_params.yaml       # Config détecteur (tag IDs, taille 0.050m)
├── package.xml
├── CMakeLists.txt
└── CLAUDE.md
```

## Dépendances ROS 2

- `rclpy`, `std_msgs`, `geometry_msgs`, `sensor_msgs`
- `tf2`, `tf2_ros`, `tf2_geometry_msgs`
- `apriltag_msgs` (messages de détection)
- `apriltag_ros` (node de détection, fourni par ir_2526)
- `moveit_msgs`, `moveit_commander` (ou `moveit_py`) — interface MoveIt!
- `cv_bridge`, `opencv-python` (pour détection couleur, extra)

## Conventions

- **Messages de commit** : format Conventional Commits en français (`feat:`, `fix:`, `refactor:`, `docs:`, `chore:`).
- **Branches** : `main` = version stable/rendue ; `dev` = branche de travail. Merger via PR.
- **Langue** : code et commentaires en anglais ; communications (CLAUDE.md, README) en français.

## À ne pas faire

- Ne pas committer `build/`, `install/`, `log/`.
- Ne pas committer les PDFs ni `.claude/`.
- Ne pas pousser directement sur `main`.
- Ne pas modifier les packages `ir_2526`.
- Ne pas committer tout le code en une seule fois : tracker le travail par commits atomiques.

## État d'avancement

- [x] Package créé (`ros2 pkg create`)
- [x] Structure de dossiers (scripts/, launch/, config/)
- [x] CLAUDE.md initial
- [ ] `package.xml` mis à jour avec les dépendances
- [ ] `CMakeLists.txt` configuré pour installer scripts et launch
- [ ] `apriltag_params.yaml`
- [ ] `tag_detector_node.py`
- [ ] `cube_swapper_node.py`
- [ ] `lorandi_assignment_2.launch.py`
- [ ] `color_detector_node.py` (extra +3)
- [ ] Tests et validation en simulation
- [ ] Dépôt GitHub privé + accès tuteurs
