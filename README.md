# Control Robusto del xArm Lite6 con Perturbaciones (ROS2 + MoveIt Servo)

Este proyecto implementa un sistema de **seguimiento de trayectorias para el robot xArm Lite6** utilizando **ROS2**, **MoveIt2** y **MoveIt Servo**.

El sistema evalúa el desempeño de dos controladores bajo perturbaciones artificiales:

- Control **PD Cartesiano**
- Control **Computed Torque Control (CTC)**

Se inyectan perturbaciones para evaluar la **robustez y estabilidad del seguimiento de trayectoria**.

---

# Arquitectura del Sistema

El sistema está compuesto por tres nodos ROS2 y un archivo de trayectorias.

```
waypoints.csv
      │
      ▼
Lite6IKNode
(genera IK y publica posición deseada)
      │
      ▼
CircleServoXArmLite6
(controlador PD / CTC)
      │
      ▼
MoveIt Servo
(capa de ejecución del robot)
      ▲
      │
PerturbationGenerator
(inserta perturbaciones)
```

---

# Estructura del Proyecto

```
xarm_perturbations/

waypoint_generator.py
controller_node.py
perturbation_injector.py

waypoints.csv

tracking_data_lite6.csv
```

---

# Waypoints

La trayectoria del robot está definida en:

```
waypoints.csv
```

Formato:

```
x,y,z
```

Ejemplo:

```
0.15,0.2,0.3
0.15,0.2,0.2
0.15,0.2,0.3
0.15,0,0.3
0.15,0,0.2
0.15,0,0.3
0.35,0,0.3
0.35,0,0.2
0.35,0,0.3
0.35,0.2,0.3
0.35,0.2,0.2
0.35,0.2,0.3
0.15,0.2,0.3
```

Esta trayectoria genera un **movimiento rectangular en el plano XY con movimiento vertical en Z**.

Área de trabajo:

```
      y
      ↑
0.2 ──●────────●
     │        │
     │        │
0.0 ──●────────●
      0.15    0.35 → x
```

En cada vértice el robot realiza un movimiento vertical:

```
z = 0.30
↓
z = 0.20
↑
z = 0.30
```

Cada waypoint se ejecuta cada **10 segundos**.

---

# Nodo 1 — Generador de Waypoints + Cinemática Inversa

Nodo:

```
Lite6IKNode
```

Responsabilidades:

- Leer los waypoints
- Calcular **cinemática inversa utilizando MoveIt**
- Publicar la posición cartesiana deseada y la configuración articular

Este nodo requiere que la **configuración de MoveIt esté ejecutándose**, ya que la IK se calcula usando `pymoveit2`.

Topics publicados:

```
/posicion_deseada  → geometry_msgs/Point
/q_deseada         → sensor_msgs/JointState
```

---

# Nodo 2 — Controlador

Nodo:

```
CircleServoXArmLite6
```

Implementa dos estrategias de control.

---

## Control PD Cartesiano

Ley de control:

```
v = Kp * e + Kd * de/dt
```

donde

```
e = x_deseado − x_actual
```

Los comandos se envían usando MoveIt Servo:

```
/servo_server/delta_twist_cmds
```

Tipo de mensaje:

```
geometry_msgs/TwistStamped
```

---

## Computed Torque Control (CTC)

Control no lineal en espacio articular:

```
τ = M(q)v + C(q,q̇) + G(q)
```

donde

```
v = q̈d + Kd(q̇d − q̇) + Kp(qd − q)
```

Las dinámicas del robot se aproximan mediante:

```
inertia_matrix(q)
coriolis_torque(q, qd)
gravity_torque(q)
```

Los comandos se envían mediante:

```
/servo_server/delta_joint_cmds
```

Tipo de mensaje:

```
control_msgs/JointJog
```

---

# Nodo 3 — Generador de Perturbaciones

Nodo:

```
PerturbationGenerator
```

Este nodo inyecta perturbaciones en los comandos de control.

Topic publicado:

```
/servo_server/delta_joint_cmds
```

Tipo de mensaje:

```
control_msgs/JointJog
```

---

# Modos de Perturbación

## Sin perturbaciones

```
mode = off
```

---

## Perturbación senoidal

Simula vibraciones periódicas:

```
v = A sin(2πft)
```

Parámetros:

```
sine_freq_hz
sine_amp_joint
sine_axis
```

Ejemplo:

```
ros2 run xarm_perturbations perturbation_injector \
--ros-args \
-p mode:=sine \
-p sine_freq_hz:=8 \
-p sine_amp_joint:=0.1 \
-p sine_axis:=2
```

---

## Ruido gaussiano

Perturbación aleatoria:

```
v ~ N(0, σ)
```

Ejemplo:

```
ros2 run xarm_perturbations perturbation_injector \
--ros-args \
-p mode:=gaussian \
-p noise_std_joint:=0.05
```

---

# Datos Registrados

Los datos de seguimiento se guardan en:

```
tracking_data_lite6.csv
```

Variables almacenadas:

```
time
x_d, y_d, z_d
x, y, z
vx_cmd, vy_cmd, vz_cmd
q1..q6
qd1..qd6
qref1..qref6
cmd1..cmd6
mode
perturbation
```

Estos datos se utilizan para:

- análisis de seguimiento de trayectoria
- cálculo de RMSE
- comparación entre controladores

---

# Ejecución del Sistema

## 1. Iniciar MoveIt + Servo

Primero debe ejecutarse la configuración del robot.

Ejemplo:

```
ros2 launch xarm_moveit_config lite6_moveit.launch.py
```

Esto inicia:

- modelo del robot
- árbol TF
- MoveIt Servo
- solucionador de cinemática inversa

---

## 2. Ejecutar el generador de waypoints

```
ros2 run xarm_perturbations waypoint_generator
```

Publica:

```
/posicion_deseada
/q_deseada
```

---

## 3. Ejecutar el controlador

```
ros2 run xarm_perturbations controller_node
```

El controlador:

- lee los estados articulares
- calcula el control
- envía comandos al robot usando MoveIt Servo

---

## 4. Ejecutar perturbaciones (opcional)

Ejemplo:

```
ros2 run xarm_perturbations perturbation_injector \
--ros-args \
-p mode:=gaussian \
-p noise_std_joint:=0.05
```

---

# Métrica de Evaluación

El desempeño se evalúa usando **Root Mean Square Error (RMSE)**.

```
RMSE = sqrt( (1/N) Σ (x_d − x)^2 )
```

Ejemplo de salida:

```
Mode: CTC | RMSE: 0.094 m
```

---

# Tecnologías Utilizadas

ROS2  
MoveIt2  
MoveIt Servo  
Python  
NumPy  
pymoveit2  

---

# Objetivo del Proyecto

Evaluar la **robustez de controladores robóticos bajo perturbaciones** comparando:

- Control PD Cartesiano
- Computed Torque Control (CTC)

en tareas de seguimiento de trayectoria con el **xArm Lite6**.

---

# Autor

Experimento de Control Robótico  
xArm Lite6 — Seguimiento de Trayectoria bajo Perturbaciones
