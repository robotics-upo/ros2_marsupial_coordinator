<div align="center">
<h1>Coordinador de trayectoria para un sistema robótico  marsupial UAV-UGV con cable de alimentación de longitud variable</h1>
</div>

# Índice de contenido

1. [Introducción](#introduccion)
2. [Requisitos previos](#requisitos-previos)
3. [Experimentos realizados](#experimentos-realizados)
4. [Validación de experimentos](#validacion-de-experimentos)

---

# Introducción

El objetivo de este trabajo es implementar un esquema de control modular que permita a cada miembro del sistema robótico marsupial seguir su trayectoria correspondiente. Posteriormente, se diseña un algoritmo de coordinación durante el seguimiento de trayectorias de forma que ambos vehículos: terrestre y aéreo, sigan el mismo punto correspondiente a su trayectoria, evaluando el progreso de cada vehículo dentro de la misma mediante "índices de waypoints".

Para ello, se hace necesaria la implementación de los siguientes módulos dentro del ecosistema de ROS2:

- **Visualizador en RViz2**: representa todos los waypoints junto con las catenarias que forma el cable en cada punto de la trayectoria. Además, resalta en color verde los objetivos perseguidos por cada elemento del sistema en cada instante de tiempo.
- **Generador de referencias**: se implementa un Pure Pursuit para cada vehículo dentro del sistema marsupial. Mediante la publicación en tópicos de ROS, se informa al resto de nodos acerca del "índice de waypoint" al que se dirige cada miembro dentro del sistema y de la lookahead distance establecida.
- **Controladores individuales**: calculan la dirección a la que deben dirigirse UAV y UGV, estableciendo la velocidad calculada por el módulo Coordinador del sistema. Para el caso del cable (tether) se utiliza un control PID (Proporcional, Integral, Derivativo) que garantiza que la longitud de cable siga en todo momento la longitud de referencia establecida por el Coordinador y verificada por el controlador individual.


# Requisitos previos
Aquí se describen los requisitos necesarios.

# Experimentos realizados
Descripción de los experimentos.

# Validación de experimentos
Explicación y análisis de resultados.
