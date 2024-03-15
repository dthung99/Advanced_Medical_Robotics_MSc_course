import numpy as np
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors
from matplotlib.widgets import Button, Slider
import pickle
import time
from tkinter import messagebox


class BuildEnvironment:
    def __init__(self, dimension: tuple = (600, 300, 10)):
        '''Get a tuple of length 3, create a zero np.array of that size'''
        self.dimension = dimension
        self.env = np.zeros(self.dimension)
        self.x_upper_limit = dimension[0] - 1
        self.y_upper_limit = dimension[1] - 1
        self.environment_obstacle_code = {"Circle Spring": 1,
                                          }
        self.environment_store_medthod = {"Circle Spring": ("Pure float force in vector index 1 and 2]")
                                          }
        

    def addCircleSpring(self, center: np.ndarray((2,)), k_matrix: np.ndarray((2,2)) = np.array([[1, 0],[0,1]]), radius = 10):
        center = np.array(center)
        lower_limit_x = max(0, round(center[0]-radius))
        upper_limit_x = min(self.x_upper_limit, round(center[0]+radius))
        lower_limit_y = max(0, round(center[1]-radius))
        upper_limit_y = min(self.y_upper_limit, round(center[1]+radius))
        for i in range(lower_limit_x, upper_limit_x + 1):
            for j in range(lower_limit_y, upper_limit_y + 1):
                current_point = np.array([i, j])
                if np.linalg.norm(current_point-center) < radius:
                    self.env[i, j, 0] = self.environment_obstacle_code["Circle Spring"]
                    directionvector = current_point - center
                    vectorlength = np.linalg.norm(directionvector)
                    if vectorlength == 0:
                        continue
                    self.env[i, j, 1:3] = self.env[i, j, 1:3] + np.matmul(k_matrix, directionvector * (radius/vectorlength - 1))/1000 #Environment is render in mm

    def draw_on_image(self):
        # Create fig, axe
        self.fig, self.ax = plt.subplots()
        self.ax.set_position([0.3, 0.05, 0.65, 0.9])

        # Declare custom variables
        # Define your custom colors
        custom_colors = ['#FF0000', '#00FF00', '#0000FF', '#FFFF00', '#FF00FF', '#00FFFF', '#FFA500', '#800080', '#008080', '#800000']
        self.custom_cmap = mcolors.ListedColormap(custom_colors)
        # self.drawing = False  # Flag to indicate if the mouse button is being held
      
        # Add a button to the figure
        # Add circle spring
        self.UIaddCircleSpringMode = False
        self.button_ax_add_circle_spring = plt.axes([0.05, 0.80, 0.2, 0.04])  # [left, bottom, width, height]
        self.button_add_circle_spring = Button(self.button_ax_add_circle_spring, 'Add Circle Spring')    
        self.button_add_circle_spring.on_clicked(self.UIaddCircleSpring)
        # Add export button
        self.button_ax_export = plt.axes([0.05, 0.90, 0.2, 0.04])  # [left, bottom, width, height]
        self.button_export = Button(self.button_ax_export, 'Export')    
        self.button_export.on_clicked(self.ExportEnvironment)
        # Add reset button
        self.button_ax_reset = plt.axes([0.05, 0.85, 0.2, 0.04])  # [left, bottom, width, height]
        self.button_reset = Button(self.button_ax_reset, 'Reset')    
        self.button_reset.on_clicked(self.ResetEnvironment)

        # self.fig.canvas.mpl_connect('button_press_event', self.on_mouse_press)
        # self.fig.canvas.mpl_connect('button_release_event', self.on_mouse_release)
        # self.fig.canvas.mpl_connect('motion_notify_event', self.on_mouse_move)

        # Plot image
        self.image_array = self.env[:,:,0]
        self.ax.imshow(self.image_array.astype(np.uint8), cmap=self.custom_cmap)
        self.ax.plot([0, self.y_upper_limit], [self.x_upper_limit/2, self.x_upper_limit/2], "k--", linewidth=0.5)
        self.ax.plot([0, 0], [0, self.x_upper_limit], "k--", linewidth=1)

        angles = np.linspace(-np.pi/2, np.pi/2, 50)
        # Compute x and y coordinates of the circle
        x = self.y_upper_limit * np.cos(angles)
        y = self.y_upper_limit * np.sin(angles) + self.x_upper_limit/2
        self.ax.plot(x, y, "k--", linewidth=0.5)

        self.ax.axis('off')
        plt.show()


    def UIaddCircleSpring(self, event):
        # Congigurate the button
        self.UIaddCircleSpringMode = True
        self.button_add_circle_spring.color = 'red'
        self.button_add_circle_spring.label.set_color('white')
        # Add widgets
        self.slide_ax_1 = plt.axes([0.10, 0.05, 0.15, 0.03])  # [left, bottom, width, height]
        self.slide_ax_2 = plt.axes([0.10, 0.10, 0.15, 0.03])  # [left, bottom, width, height]
        self.slide_ax_3 = plt.axes([0.10, 0.15, 0.15, 0.03])  # [left, bottom, width, height]

        self.slider_1 = Slider(self.slide_ax_1, 'K1', 0, 500, valinit=250)
        self.slider_2 = Slider(self.slide_ax_2, 'K2', 0, 500, valinit=250)
        self.slider_3 = Slider(self.slide_ax_3, 'Radius', 0, 100, valinit=10)
        # Connect to main code
        self.fig.canvas.mpl_connect('button_press_event', self.on_mouse_press_UIaddCircleSpring)
    def on_mouse_press_UIaddCircleSpring(self, event):
        if event.inaxes == self.ax:
            position = [int(event.ydata), int(event.xdata)]
            self.addCircleSpring(position, k_matrix=np.array([[self.slider_1.val, 0],[0,self.slider_2.val]]), radius = self.slider_3.val)
            self.updateFigure()
    def UIaddCircleSpringClear(self):
        self.UIaddCircleSpringMode = False
        self.button_add_circle_spring.color = "0.85"
        self.button_add_circle_spring.label.set_color('black')
        self.slide_ax_1.remove()
        self.slide_ax_2.remove()
        self.slide_ax_3.remove()

    def clear_all_UI_subWidgets(self):
        if self.UIaddCircleSpringMode:
            self.UIaddCircleSpringClear()


    def updateFigure(self):
        # Update the figure
        self.image_array = self.env[:,:,0]
        self.ax.imshow(self.image_array.astype(np.uint8), cmap=self.custom_cmap)
        self.fig.canvas.draw()

    def ResetEnvironment(self, event):
        self.env = np.zeros(self.dimension)
        self.clear_all_UI_subWidgets()
        self.updateFigure()

    def ExportEnvironment(self, event):
        export_array = np.copy(self.env)
        # Transform to robot space
        support_variable = np.copy(export_array[:,:,1])
        export_array[:,:,1] = np.copy(export_array[:,:,2])
        export_array[:,:,2] = -support_variable
        # Save file
        export_array = export_array.tolist()
        with open('Environment.pk1', 'wb') as file:
            pickle.dump(export_array, file)
        print("Successful!!")
        # Create a pop-up message box
        messagebox.showinfo("Congratulation", "Export finished!!")

        


    # def on_mouse_press(self, event):
    #     if event.inaxes == self.ax:
    #         self.drawing = True
    # def on_mouse_release(self, event):
    #     self.drawing = False
    # def on_mouse_move(self, event):
    #     if event.inaxes == self.ax and self.drawing:
    #         x = int(event.xdata)
    #         y = int(event.ydata)
    #         self.image_array[y, x] = 255  # Set the pixel to white while moving
    #         self.ax.imshow(self.image_array.astype(np.uint8), cmap='gray')
    #         self.fig.canvas.draw()

# Create a random 2D NumPy array as an example image



image_array = BuildEnvironment()
image_array.draw_on_image()



