# OctoWorld_ws

## Table of Contents

- [English Version](#english-version)
  - [Pre-requires](#pre-requires)
  - [How to use](#how-to-use)
  - [Tips](#tips)
- [Versão em Português](#versão-em-português)
  - [Pré Requisitos](#pré-requisitos)
  - [Como utilizar](#como-utilizar)
  - [Dicas](#dicas)
- [References](#references)
- [Referências](#referências)

---

<p align="center"><strong> ▼ English version below ▼ </strong></p>

## English Version

### OctoWorld_ws – Generation and Saving of OctoMaps using RTAB-Map SLAM

This repository was created with the goal of generating and saving an **OctoMap** and its corresponding '.bt' (binary tree) file, based on the topics published by RTAB-Map in **ROS2 Humble**.  
In this particular project, an **Intel RealSense D455** was used to capture environmental data and apply the **RTAB-Map SLAM** method to incrementally build a 3D map, which is then processed and exported as an OctoMap.

---

### Pre-requires:

 -ROS2 Humble
 -Intel RealSense SDK 2.0
 -realsense2_camera package installed
 -rtabmap_ros installed and working (with octomap_plugins)
 -octomap_msgs and octomap installed (sudo apt install ros-humble-octomap*)

### How to use:

**Step 1** (launch the camera node)

    ros2 launch realsense2_camera rs_launch.py 

**Step 2** (launch RTAB-Map SLAM)

    ros2 launch rtabmap_launch rtabmap.launch.py \
    rtabmap_args:="--delete_db_on_start --database_path /path/to/the/map/map.db" \
    octomap_full:=true \
    octomap_binary:=true \
    rgb_topic:=/camera/camera/color/image_raw \
    depth_topic:=/camera/camera/depth/image_rect_raw \
    camera_info_topic:=/camera/camera/color/camera_info \
    frame_id:=camera_link \
    approx_sync:=true \
    launch_rviz:=true


### Notes:

This launch method was adapted from the original (designed for a ZED camera).  
RTAB-Map generates a .db file, so choose the most convenient path for you.

**Step 3** (compile the octomap_tools package and run save_octomap_bt to save the .bt map file)

    cd ~/world_ws
    colcon build --packages-select octomap_tools
    source install/setup.bash

Then, run the node responsible for saving the map:

    ros2 run octomap_tools save_octomap_bt --ros-args -p output_path:=/path/to/map/map.bt

**Step 4 (optional)** (visualize the saved OctoMap to verify it)

    octovis /path/to/map/map.bt


### Tips:

Before saving the map, make sure RTAB-Map has already built the complete layout of the environment.  
You can visualize this using rtabmap_viz (via the point cloud) or in Rviz2, where RTAB-Map topics are displayed *(e.g rtabmap/octomap_full and rtabmap/octomap_binary)*.

Note that RTAB-Map doesn't publish traditional OcTree structures, but rather ColorOcTree.  
Therefore, you can't use *octomap_server* neither *octomap_saver* directly to export the .bt file, which is why this custom node was created.


---

<p align="center"><strong> ▼ Abaixo está a versão em Português ▼ </strong></p>

## Versão em Português

### OctoWorld_ws – Geração e Gravação de OctoMaps utilizando SLAM RTAB-Map

Este repositório foi criado com o objetivo de gerar e guardar um **OctoMap** e o respetivo ficheiro '.bt' (binary tree), a partir dos tópicos publicados pelo RTAB-Map em **ROS2 Humble**.  
No caso particular deste projeto, foi utilizada uma **Intel RealSense D455** para captar a informação do ambiente e aplicar o método **SLAM RTAB-Map** para construir, incrementalmente, um mapa em 3D, que possa ser processado e exportado como OctoMap.

---

### Pré Requisitos:

-ROS2 Humble
-Intel RealSense SDK 2.0
-Pacote `realsense2_camera` instalado
-rtabmap_ros instalado e funcional (com octomap_plugins)
-`octomap_msgs` e `octomap` instalados (`sudo apt install ros-humble-octomap*`)



### Como utilizar:

**1º passo** (lançar nó da câmera)

    ros2 launch realsense2_camera rs_launch.py 

**2º passo** (lançar o Slam Rtab-Map)

    ros2 launch rtabmap_launch rtabmap.launch.py \
    rtabmap_args:="--delete_db_on_start --database_path /caminho/para/o/mapa/mapa.db" \
    octomap_full:=true \
    octomap_binary:=true \
    rgb_topic:=/camera/camera/color/image_raw \
    depth_topic:=/camera/camera/depth/image_rect_raw \
    camera_info_topic:=/camera/camera/color/camera_info \
    frame_id:=camera_link \
    approx_sync:=true \
    launch_rviz:=true



### Notas:

Este modo de lançamento foi adaptado do original (criado para uma câmera ZED).  
O RTAB-Map gera um ficheiro .db, por isso escolhe o caminho mais conveniente para ti.

**3º passo** (compilar o pacote octomap_tools e lançar save_octomap_bt para guardar o mapa.bt)

    cd ~/world_ws
    colcon build --packages-select octomap_tools
    source install/setup.bash

Depois, executar o nó responsável por guardar o mapa:

    ros2 run octomap_tools save_octomap_bt --ros-args -p output_path:=/caminho/para/o/mapa/mapa.bt

**4º passo -- opcional** (visualizar o octomap guardado para ver se está tudo bem)

    octovis /caminho/para/o/mapa/mapa.bt


### Dicas:

Antes de se gravar o mapa, é importante verificar se o RTAB-Map já construiu a planta completa do espaço.  
Podendo ser visualizado tanto no Rtabmap_viz (através da nuvem de pontos), como no Rviz2 onde se pode ver os tópicos do *rtabmap/octomap_full* e *rtabmap/octomap_binary*.  

O RTAB-Map não publica estruturas OcTree tradicionais, mas sim ColorOcTree.  
Por isso, não é possível usar diretamente o *octomap_server* e *octomap_saver* para exportar o .bt, o que levou à criação deste nó personalizado.
 

---

## References

- [octomap_msgs/conversions.h](https://github.com/OctoMap/octomap_msgs/blob/melodic-devel/include/octomap_msgs/conversions.h) – Useful C++ utility for converting ROS messages into `ColorOcTree` format.
- [rtabmap_ros issue #876](https://github.com/introlab/rtabmap_ros/issues/876#issuecomment-1407534728) – Clarifies why `octomap_server` cannot save `ColorOcTree` directly from RTAB-Map topics.
- [octomap_mapping repository](https://github.com/OctoMap/octomap_mapping) – Collection of tools including `octomap_server` e `octomap_saver`.
- [rtabmap_ros repository](https://github.com/introlab/rtabmap_ros) – SLAM implementation used to generate maps and publish octomap topics.
- [Intel RealSense ROS wrapper](https://github.com/IntelRealSense/realsense-ros) – Driver integration for RealSense cameras in ROS2.

## Referências

- [octomap_msgs/conversions.h](https://github.com/OctoMap/octomap_msgs/blob/melodic-devel/include/octomap_msgs/conversions.h) – Conversor essencial para manipular `ColorOcTree` no ROS2.
- [Issue #876 em rtabmap_ros](https://github.com/introlab/rtabmap_ros/issues/876#issuecomment-1407534728) – Explica porque o `octomap_server` não consegue salvar `ColorOcTree`, diretamente dos tópicos publicados pelo Rtab-Map.
- [Repositório octomap_mapping](https://github.com/OctoMap/octomap_mapping) – Inclui ferramentas úteis como `octomap_server` e `octomap_saver`.
- [Repositório rtabmap_ros](https://github.com/introlab/rtabmap_ros) – Técnica SLAM usada neste projeto.
- [RealSense ROS Wrapper](https://github.com/IntelRealSense/realsense-ros) – Driver da câmera RealSense D455 usado com ROS2.
