import matplotlib.pyplot as plt
from shapely.geometry import Polygon
from src.decomposition import ConcaveDecomposer

# 1. Crear un polígono en forma de "C" (Cóncavo)
coords_c = [
    (0,0), (100,0), (100,100), (0,100), 
    (0,70), (70,70), (70,30), (0,30) # La "mordida" de la C
]
poly_c = Polygon(coords_c)

# 2. Probar descomposición con vuelo a 0 grados (Horizontal)
# Debería cortarlo en 3 pedazos (o 2 dependiendo de dónde empiece) porque la C bloquea el paso horizontal.
parts_0deg = ConcaveDecomposer.decompose(poly_c, heading_angle_deg=0)

# 3. Probar con vuelo a 90 grados (Vertical)
# NO debería cortarlo, porque verticalmente se puede barrer la C sin problemas.
parts_90deg = ConcaveDecomposer.decompose(poly_c, heading_angle_deg=90)

print(f"Ángulo 0° generó {len(parts_0deg)} sub-áreas.")
print(f"Ángulo 90° generó {len(parts_90deg)} sub-áreas.")

# 4. Visualizar el corte a 0°
fig, ax = plt.subplots()
colors = ['red', 'green', 'blue', 'orange']
for i, p in enumerate(parts_0deg):
    x, y = p.exterior.xy
    ax.fill(x, y, alpha=0.5, fc=colors[i%len(colors)], ec='black', label=f'Parte {i+1}')

plt.title("Descomposición de Concavidad (Vuelo 0°)")
plt.legend()
plt.axis('equal')
plt.show()