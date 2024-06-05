import pandas as pd
import matplotlib.pyplot as plt

# Carregar os dados do arquivo CSV
file_path = 'grid/ssl-el.csv'  # Substitua pelo caminho do seu arquivo
data = pd.read_csv(file_path, header=None)

# Converter os dados para uma matriz numpy
image_data = data.to_numpy()

# Contar quantidade de pixels igual a 1
print("Quantidade de pixels igual a 1:", (image_data == 1).sum())
print("Quantidade de pixels igual a 0:", (image_data == 0).sum())

# Configurar o gráfico
plt.figure(figsize=(10, 10))  # Tamanho ajustado para grande imagem
plt.imshow(image_data, cmap='gray', interpolation='nearest', vmin=0, vmax=1)
# Remover os eixos para uma visualização mais limpa
plt.axis('off')

# Exibir a imagem
plt.show()
