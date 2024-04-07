import matplotlib.pyplot as plt
import matplotlib.colors as mcolors
import numpy as np

# Define your custom colors
custom_colors = ['#FF0000', '#00FF00', '#0000FF', '#FFFF00', '#FF00FF', '#00FFFF', '#FFA500', '#800080', '#008080', '#800000']

# Create a custom colormap
custom_cmap = mcolors.ListedColormap(custom_colors)

# Generate a random image for demonstration
image = np.random.randint(0, 10, (100, 100))

# Display the image using the custom colormap
plt.imshow(image, cmap=custom_cmap)

# Show the colorbar
plt.colorbar()

# Show the plot
plt.show()