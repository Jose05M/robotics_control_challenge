import csv

# Los datos exactos de tu código original
data = [
    [0.15, -0.06, 0.05], # b
    [0.15, 0.06, 0.05],  # A
    [0.19, 0.06, 0.05],  # C
    [0.19, 0.02, 0.05],  # D
    [0.15, 0.02, 0.05],  # E
    [0.19, -0.06, 0.05], # F
    [0.19, -0.06, 0.10], # 
    [0.26, 0.06, 0.10],  # 
    [0.26, 0.06, 0.05],  # Vertical 1
    [0.26, -0.03, 0.05], # Vertical 2
    [0.23, -0.06, 0.05], # Curva inf 1
    [0.21, -0.04, 0.05], # Curva inf 2
    [0.21, -0.04, 0.10], # 
    [0.35, 0.05, 0.10],  # 
    [0.35, 0.05, 0.05],  # Parte sup 1
    [0.32, 0.06, 0.05],  # Parte sup 2
    [0.29, 0.04, 0.05],  # Parte sup 3
    [0.28, 0.00, 0.05],  # Lado izq
    [0.29, -0.04, 0.05], # Lado izq 2
    [0.32, -0.06, 0.05], # Parte inf 1
    [0.35, -0.05, 0.05]  # Parte inf 2
]

file_name = "trayectoria_lite6.csv"

with open(file_name, "w", newline="") as f:
    writer = csv.writer(f)
    writer.writerows(data)

print(f"Archivo '{file_name}' generado con éxito.")
