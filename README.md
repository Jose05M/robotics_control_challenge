# Control del xArm Lite6 con Perturbaciones (ROS2)

Este proyecto implementa un sistema de seguimiento de trayectorias para el robot xArm Lite6 utilizando ROS2 y la cinematica inversa implementada con MoveIt2.

El sistema evalúa el desempeño de dos controladores bajo perturbaciones artificiales:

- Control **PD Cartesiano**
- Control **Computed Torque Control (CTC)**

Se inyectan perturbaciones para evaluar la robustez y estabilidad del seguimiento de trayectoria.

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

# Waypoints

La trayectoria del robot está definida en:

```
waypoints.csv
```

Formato:

```
x,y,z
```

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

Esta trayectoria genera un movimiento cuadrangular en el plano XY con movimiento vertical en Z, lo cual simula un taladro que tiene que hacer 4 agujeros, formando un cuadrado de 0.2 m por 0.2m.

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

Cada waypoint es procesado (publica posiciones cartesianas y articulares) cada 10 segundos.

---

# Nodo 1 — Generador de Waypoints + Cinemática Inversa

Nodo:

```
Lite6IKNode
```

Responsabilidades:

- Leer los waypoints
- Calcular cinemática inversa utilizando MoveIt
- Publicar la posición cartesiana deseada y la configuración articular

Este nodo requiere que la configuración de MoveIt esté ejecutándose, ya que la IK se calcula usando `pymoveit2`.

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
v = Kp * e + Kd * de
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

Ejemplo de ejecución:

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

Ejemplo de perturbación:

```
ros2 run xarm_perturbations perturbation_injector \
--ros-args \
-p mode:=gaussian \
-p noise_std_joint:=0.0001
```

---

# Datos Registrados

Los datos de seguimiento se guardan en un archivo csv.

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
Primero debemos estar conectados con el Xarm Lite6 fisico.
Despues, ejecutar la configuración del robot en dos diferentes terminales, con lo siguiente:
```
ros2 launch xarm_moveit_config lite6_moveit_realmove.launch.py robot_ip:=192.168.1.179
ros2 launch xarm_moveit_servo lite6_moveit_servo_realmove.launch.py robot_ip:=192.168.1.179
```

Esto inicia:

- modelo del robot
- árbol TF
- MoveIt Servo
- solucionador de cinemática inversa
---
Despues, mover el robot a una posicion predefinida, para garantizar que todas las pruebas tengan las mismas condiciones iniciales y evitar errores en la solucion de la IK de MoveIt.

## 2. Ejecutar el generador de waypoints
En otra terminal, ejecutar el generador de waypoints con lo siguiente:
```
ros2 run lite6_demo_moveit lite6_demo 
```

Publica:

```
/posicion_deseada
/q_deseada
```

---

## 3. Ejecutar el controlador
En otra terminal, ejecutar el nodo controlador, con lo siguiente:
```
ros2 run xarm_perturbations circle_maker 
```

El controlador:

- lee las posiciones cartesianas deseadas
- lee los estados articulares deseadas
- calcula el control (PD/CTC)
- envía comandos al robot usando MoveIt Servo

---

## 4. Ejecutar perturbaciones
Para comprobar el comportamiento de los controladores bajo perturbaciones, mientras se esta ejecutando la trajectoria, en otra terminal, ejecutar el nodo de perturbaciones con lo siguiente:

```
ros2 run xarm_perturbations perturbation_injector \
--ros-args \
-p mode:=gaussian \
-p noise_std_joint:=0.0001
```

---

# Métrica de Evaluación

El desempeño se evalúa usando **Root Mean Square Error (RMSE)**.

```
RMSE = sqrt( (1/N) Σ (x_d − x)^2 )
```

# Tecnologías Utilizadas

ROS2  
MoveIt2  
MoveIt Servo  
Python  
NumPy  
pymoveit2  

---

# Objetivo del Proyecto

Evaluar la robustez de controladores robóticos bajo perturbaciones comparando:

- Control PD Cartesiano
- Computed Torque Control (CTC)

en tareas de seguimiento de trayectoria con el xArm Lite6.

---

# Autores
Jose Eduardo Sanchez Martinez		      IRS | A01738476
Josue Ureña Valencia				IRS | A01738940
César Arellano Arellano			      IRS | A00839373
Rafael André Gamiz Salazar			IRS | A00838280
