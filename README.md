# Optimización de Packing 3D en Contenedores

## Descripción General

Este proyecto aborda el problema de optimización de carga tridimensional (3D Container Packing), cuyo objetivo es determinar cómo ubicar un conjunto de cajas dentro de un contenedor maximizando el aprovechamiento del espacio y respetando restricciones geométricas.

Se partió de un modelo exacto de Programación Lineal Entera previamente formulado, el cual presentaba limitaciones significativas en términos de memoria y tiempo de cómputo al escalar el tamaño de las instancias.

El objetivo principal del proyecto fue analizar dichas limitaciones y diseñar estrategias heurísticas que permitieran obtener soluciones factibles en tiempos considerablemente menores, manteniendo buena calidad de solución.

---

## Problema

El problema consiste en decidir la ubicación de cajas con dimensiones definidas dentro de un contenedor, asegurando:

- No superposición entre cajas.
- Respeto de los límites del contenedor.
- Cumplimiento de restricciones geométricas.

El modelo exacto fue implementado en Java utilizando IBM CPLEX como solver.

---

## Estructura del Repositorio

- `master`  
  Contiene la implementación del modelo exacto provisto como punto de partida.

- `por_capas`  
  Heurística basada en una estrategia constructiva por capas.

- `por_capas_filling`  
  Variante de la heurística por capas incorporando lógica adicional de filling.

- `relajacion_lineal`  
  Implementación basada en la relajación del modelo exacto.

- `agrupar_cajas`  
  Heurística basada en la agrupación previa de cajas antes de su asignación.

Cada branch contiene una implementación completa y funcional de la estrategia correspondiente.

---

## Metodología

El desarrollo se estructuró en tres etapas:

1. Análisis del modelo exacto y estudio de sus limitaciones computacionales.
2. Diseño conceptual de distintas heurísticas para reducir tiempos de resolución.
3. Implementación y evaluación comparativa de las estrategias desarrolladas.

Se compararon:

- Tiempo de cómputo.
- Calidad de la solución.
- Escalabilidad ante instancias de mayor tamaño.

---

## Resultados

Las heurísticas desarrolladas permitieron:

- Reducir significativamente los tiempos de cómputo respecto del modelo exacto.
- Obtener soluciones factibles en instancias donde el modelo exacto no escalaba.
- Mantener niveles de calidad de solución competitivos frente al enfoque exacto en instancias medianas.
Este proyecto pone en evidencia el trade-off entre optimalidad y eficiencia computacional

---

## Documentación

El informe técnico completo se encuentra en la carpeta `/docs`.

Incluye:

- Formulación matemática detallada.
- Justificación de restricciones.
- Diseño de las heurísticas.
- Resultados computacionales.
- Análisis comparativo.
- Conclusiones sobre eficiencia y desempeño.

---

## Autoría

Desarrollado por:

- Catalina Dolhare 
- Joaquín Schanz  

Proyecto académico de optimización enfocado en el análisis y mejora de desempeño computacional mediante diseño de heurísticas.
