# AgriSwarm Planner: Coverage Path Planning (CPP)

## 1. Abstract
Implementación de un sistema avanzado de Planificación de Rutas de Cobertura (CPP) para UAVs de fumigación agrícola, diseñado para operar en polígonos arbitrarios (convexos y cóncavos). Basado en la metodología de **Li et al. (2023)**, este sistema integra algoritmos de reducción de márgenes, detección topológica de concavidades y optimización de dirección de vuelo mediante Algoritmos Genéticos (GA).

El objetivo principal es minimizar una función de costo híbrida que pondera la distancia total de vuelo y la tasa de sobre-cobertura (desperdicio de pesticida).

---

## 2. Modelado Matemático Completo

La implementación sigue estrictamente las 4 fases matemáticas propuestas en el paper de referencia.

### 2.1. Fase 1: Reducción de Márgenes (Margin Reduction)
Para evitar la aspersión fuera de los límites, el polígono original se contrae una distancia $h$.
Cada vértice $D_i$ se desplaza hacia el interior a una nueva posición $D'_i$.

* [cite_start]**Vector de Dirección de Contracción ($\vec{C}$):** Suma normalizada de los vectores de los lados adyacentes[cite: 585].
    $$
    \vec{C} = \frac{\vec{D_i D_{i-1}}}{|\vec{D_i D_{i-1}}|} + \frac{\vec{D_i D_{i+1}}}{|\vec{D_i D_{i+1}}|} \tag{1}
    $$

* [cite_start]**Ángulo Interior ($\theta$):** Cálculo del ángulo del vértice[cite: 616].
    $$
    \theta = \arccos \left( \frac{\vec{D_i D_{i-1}}}{|\vec{D_i D_{i-1}}|} \cdot \frac{\vec{D_i D_{i+1}}}{|\vec{D_i D_{i+1}}|} \right) \tag{2}
    $$

* [cite_start]**Magnitud del Desplazamiento:** Distancia euclidiana del nuevo vértice al original para garantizar un margen $h$ perpendicular [cite: 618-620].
    $$
    |\vec{D_i D'_i}| = \frac{h}{\sin(\theta / 2)} \tag{3}
    $$

### 2.2. Fase 2: Geometría de Vuelo en Polígonos Convexos
Para generar la ruta en zig-zag (Boustrophedon) con un ángulo de cabecera $\psi$:

* [cite_start]**Ecuación de Frontera:** Define los límites del área operativa reducida[cite: 636].
    $$
    (y - y_{D'_i})(x_{D'_i} - x_{D'_{i-1}}) = (x - x_{D'_i})(y_{D'_i} - y_{D'_{i-1}}) \tag{4}
    $$

* [cite_start]**Generación de Waypoints:** Intersección de las líneas de ruta $r_i$ (separadas por ancho $d$) con la frontera[cite: 641].
    $$
    \begin{cases} 
    y = y_{min} + r_i \cdot d/2 \\ 
    (y - y_{D'_i})(x_{D'_i} - x_{D'_{i-1}}) = (x - x_{D'_i})(y_{D'_i} - y_{D'_{i-1}}) 
    \end{cases} \tag{5}
    $$

* [cite_start]**Centro de Gravedad ($x_t, y_t$):** Punto pivote para la rotación del mapa, calculado ponderando los centroides de sub-triángulos[cite: 647].
    $$
    x_t = \frac{\sum_{i=1}^{j} C_{ix} A_i}{\sum_{i=1}^{j} A_i}, \quad y_t = \frac{\sum_{i=1}^{j} C_{iy} A_i}{\sum_{i=1}^{j} A_i} \tag{6}
    $$

* [cite_start]**Rotación de Coordenadas:** Transformación de vértices según el ángulo $\psi$[cite: 650].
    $$
    \begin{cases} 
    x' = (x_{D'_i} - x_t)\sin\psi + (y_{D'_i} - y_t)\cos\psi + x_t \\ 
    y' = (x_{D'_i} - x_t)\sin\psi + (y_{D'_i} - y_t)\cos\psi + y_t 
    \end{cases} \tag{7}
    $$

### 2.3. Fase 3: Detección de Concavidad (Mapeo Topológico)
Para identificar vértices que obstruyen la ruta ("Tipo 2") y requieren descomposición del área.

* [cite_start]**Líneas Proyectivas:** Se definen dos líneas auxiliares $L_1, L_2$ alrededor del vértice $D_i$[cite: 713].
    $$
    \begin{cases} \gamma_{L1} = y_i + k \\ \gamma_{L2} = y_i - k \end{cases} \tag{8}
    $$

* [cite_start]**Puntos de Proyección ($XX$):** Proyección de los vértices adyacentes sobre las líneas auxiliares[cite: 718].
    $$
    XX_{i \pm 1} = \frac{(x_{i \pm 1} - x_i)(y - y_i)}{(y_{i \pm 1} - y_i)} + x_i \tag{9}
    $$

* [cite_start]**Criterio de Decisión ($\delta$):** Diferencia posicional para clasificar la concavidad[cite: 721].
    $$
    \delta = XX_{i-1} - XX_{i+1} \tag{10}
    $$

### 2.4. Fase 4: Optimización por Algoritmo Genético (GA)
El ángulo óptimo $\psi_{opt}$ se encuentra maximizando una función de aptitud basada en eficiencia.

* [cite_start]**Distancia de Vuelo Total ($l$):** Suma de trayectos entre waypoints $p$[cite: 788].
    $$
    l = \sum_{i=1}^{n} (|\vec{p_{2i} p_{2i-1}}|) \tag{11}
    $$

* [cite_start]**Tasa de Cobertura Extra ($\eta$):** Porcentaje de área fumigada fuera del polígono útil[cite: 790].
    $$
    \eta = \frac{|\sum_{i=1}^{n}(|\vec{p_{2i} p_{2i-1}}| d) - S|}{S} \times 100\% \tag{12}
    $$
    [cite_start]Donde $S'$ (Área cubierta estimada) es[cite: 793]:
    $$
    S' = \sum_{i=1}^{n} (|\vec{p_{2i} p_{2i-1}}| d) \tag{13}
    $$

* **Función de Fitness ($f_{fitness}$):** Combina la distancia normalizada $I_{norm}$ y el error de cobertura. [cite_start]Se busca maximizar este valor[cite: 800, 801].
    $$
    I_{norm} = l_i / \sqrt{\sum_{i=1}^{\alpha} l_i^2} \tag{14}
    $$
    $$
    f_{fitness} = \left( I_{norm} + \left| \frac{S' - S}{S} \right| \right)^{-1} \tag{15}
    $$

* [cite_start]**Selección (Ruleta):** Probabilidad de que un individuo (ángulo) pase a la siguiente generación[cite: 811].
    $$
    P_{select}(a_i) = \frac{f_{fitness}(a_i)}{\sum_{j=1}^{n} f(a_j)} \tag{16}
    $$
    $$
    Q_{select}(a_i) = \sum_{j=1}^{i} P(a_j) \tag{17}
    $$

---

## 3. Arquitectura del Software
(Mantén aquí tu estructura de carpetas src/ data/ examples/)

## 4. Referencias
1. **Li, J., Sheng, H., Zhang, J., & Zhang, H. (2023).** *"Coverage Path Planning Method for Agricultural Spraying UAV in Arbitrary Polygon Area"*. Aerospace, 10(9), 755. MDPI. [https://doi.org/10.3390/aerospace10090755]