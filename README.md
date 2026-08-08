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
MoveitPositionNode
(genera IK y publica posición deseada)
      │
      ▼
PdCtcController
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
MoveitPositionNode
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
PdCtcController
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
ros2 run xarm_perturbations_ctc perturbation_injector \
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
ros2 run xarm_perturbations_ctc perturbation_injector \
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

# Requisitos

- Ubuntu 22.04
- ROS 2 Humble
- [`xarm_ros2`](https://github.com/xArm-Developer/xarm_ros2) (UFACTORY), con soporte para xArm Lite 6 y los paquetes `xarm_moveit_config` / `xarm_moveit_servo`
- `pymoveit2` (lo usa `moveit_position` para resolver la IK)
- Dependencias de Python: `numpy`, `pynput` (`sudo apt install python3-pynput` o vía `rosdep`)

**Importante — control por teclado:** `pd_ctc_controller` usa `pynput` para capturar `p` (pausa/reanuda) y `c` (cambia entre CTC y PD) desde la terminal. `pynput` necesita una sesión **X11** para capturar el teclado de forma global; en **Wayland** las teclas no se interceptan y simplemente se escriben como texto plano en la terminal, sin ningún efecto sobre el nodo. Verifica tu sesión con:
```
echo $XDG_SESSION_TYPE
```
Si dice `wayland`, cierra sesión y en la pantalla de login selecciona **"Ubuntu on Xorg"** (en vez de la sesión normal) antes de volver a iniciar sesión.

---

# Ejecución del Sistema

## 1. Iniciar MoveIt + Servo

**Opción A — Robot físico:**

Debemos estar conectados con el xArm Lite6 físico. Ejecutar la configuración del robot en dos terminales distintas:
```
ros2 launch xarm_moveit_config lite6_moveit_realmove.launch.py robot_ip:=192.168.1.179
ros2 launch xarm_moveit_servo lite6_moveit_servo_realmove.launch.py robot_ip:=192.168.1.179
```
Ajusta `robot_ip` a la IP real del controlador del brazo, y mantén el botón de paro de emergencia al alcance.

**Opción B — Simulación (sin robot físico):**

Si no tienes el brazo disponible, usa el equivalente en hardware simulado (`fake`), que no necesita `robot_ip`:
```
ros2 launch xarm_moveit_config lite6_moveit_fake.launch.py
ros2 launch xarm_moveit_servo lite6_moveit_servo_fake.launch.py
```
Esto abre RViz con el modelo del robot en su pose home, listo para recibir comandos de MoveIt Servo. Es el modo recomendado para validar que el pipeline de nodos funciona antes de probar con el robot real (ver la nota sobre el CTC en simulación, más abajo).

Cualquiera de las dos opciones deja corriendo:

- modelo del robot
- árbol TF
- MoveIt Servo
- solucionador de cinemática inversa

Después hay que mover el robot a una posición predefinida, para garantizar que todas las pruebas tengan las mismas condiciones iniciales y evitar errores en la solución de la IK de MoveIt.

| Joint | Valor (°) |
|---|---|
| joint1 | 44 |
| joint2 | 24 |
| joint3 | 78 |
| joint4 | -2 |
| joint5 | 55 |
| joint6 | 45 |

Para llevar el robot físico ahí antes de correr las pruebas: en RViz, panel **MotionPlanning** → pestaña **Joints**, ahi podras mover los joints para colocarlos en la posicion correcta.

---

## 2. Ejecutar el controlador
En otra terminal, ejecutar el nodo controlador, con lo siguiente:
```
ros2 run xarm_perturbations_ctc pd_ctc_controller 
```

El controlador:

- lee las posiciones cartesianas deseadas
- lee los estados articulares deseadas
- calcula el control (PD/CTC)
- envía comandos al robot usando MoveIt Servo

> ⚠️ **Nota sobre CTC en simulación:** el modelo de dinámica que usa el control CTC (`inertia_matrix`, `coriolis_torque`, `gravity_torque` en `pd_ctc_controller.py`) es una aproximación ajustada empíricamente observando el comportamiento del **robot real**, no un modelo físico exacto. El hardware simulado (`fake`) no tiene inercia ni fricción real: aplica los comandos de velocidad/torque de forma prácticamente instantánea, sin el amortiguamiento que sí aporta el robot real. Por eso en simulación el CTC puede verse más brusco (saltos rápidos al inicio) o no terminar de converger a los waypoints si el error inicial satura `tau_limit`.

> **Cambiar de PD a CTC** para realizar este cambio, basta con teclear `c` para cambiar de un controlador a otro, y con la tecla `p` se pone en pausa.
---

## 3. Ejecutar el generador de waypoints
En otra terminal, ejecutar el generador de waypoints con lo siguiente:
```
ros2 run xarm_perturbations_ctc moveit_position
```

Publica:

```
/posicion_deseada
/q_deseada
```

---

## 4. Ejecutar perturbaciones
Para comprobar el comportamiento de los controladores bajo perturbaciones, mientras se esta ejecutando la trajectoria, en otra terminal, ejecutar el nodo de perturbaciones con lo siguiente:

```
ros2 run xarm_perturbations_ctc perturbation_injector \
--ros-args \
-p mode:=gaussian \
-p noise_std_joint:=0.0001
```

---

# Autores
Jose Eduardo Sanchez Martinez		      IRS | A01738476;
Josue Ureña Valencia				IRS | A01738940;
César Arellano Arellano			      IRS | A00839373;
Rafael André Gamiz Salazar			IRS | A00838280;
