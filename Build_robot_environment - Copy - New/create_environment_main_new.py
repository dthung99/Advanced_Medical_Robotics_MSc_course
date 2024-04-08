import numpy as np
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors
from matplotlib.widgets import Button, Slider
import pickle
import math
from tkinter import messagebox


class BuildEnvironment:
    def __init__(self, dimension: tuple = (600, 300, 10)):
        '''Get a tuple of length 3, create a zero np.array of that size'''
        self.dimension = dimension
        self.env = np.zeros(self.dimension)
        self.x_upper_limit = dimension[0] - 1
        self.y_upper_limit = dimension[1] - 1
        self.environment_obstacle_code = {"Circle Spring": 1,
                                          "Add Damping Area": 2,
                                          "Add Rigid Wall": 3,
                                          "Add Facilitate Area": 4,
                                          "Add Rough Wall": 3,
                                          }
        self.environment_store_medthod = {"Circle Spring": ("Pure float force in vector index 1 and 2]"),
                                          "Add Damping Area": ("Gain of velocity in vector index 3"),
                                          "Add Rigid Wall": ("Combined of spring and damping -> store in 1 2 3"),
                                          "Add Facilitate Area": ("A vector and a co-effeciency -> store in [4 5] and 6"),
                                          "Add Rough Wall": ("Rough wall at the surface, combined of spring and damping -> store in 1 2 3"),
                                          }
###########################################
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

    def addDampingArea(self, center: np.ndarray((2,)), damping_factor = -1, radius = 10):
        center = np.array(center)
        lower_limit_x = max(0, round(center[0]-radius))
        upper_limit_x = min(self.x_upper_limit, round(center[0]+radius))
        lower_limit_y = max(0, round(center[1]-radius))
        upper_limit_y = min(self.y_upper_limit, round(center[1]+radius))
        for i in range(lower_limit_x, upper_limit_x + 1):
            for j in range(lower_limit_y, upper_limit_y + 1):
                current_point = np.array([i, j])
                if np.linalg.norm(current_point-center) < radius:
                    self.env[i, j, 0] = self.environment_obstacle_code["Add Damping Area"]
                    self.env[i, j, 3] = damping_factor

    def addRigidWall(self, pointlist: np.ndarray, wall_depth = 10, wall_strength = 2, damping_factor = -15):
        # y = ax + b
        point_1 = pointlist[0]
        point_2 = pointlist[1]
        point_3 = pointlist[2]
        direction_unit_vector = point_2 - point_1
        segment_length = np.linalg.norm(direction_unit_vector)
        direction_unit_vector = direction_unit_vector/segment_length

        orthogonal_unit_vector = np.r_[-direction_unit_vector[1], direction_unit_vector[0]]
        orthogonal_unit_vector = np.sign(np.dot(orthogonal_unit_vector, point_3 - point_2)) * orthogonal_unit_vector

        current_point = point_1
        while np.linalg.norm(current_point - point_1) < segment_length:
            x_current = current_point[0]
            y_current = current_point[1]
            self.env[math.floor(x_current), math.floor(y_current), 0] = self.environment_obstacle_code["Add Rigid Wall"]
            self.env[math.ceil(x_current), math.ceil(y_current), 0] = self.environment_obstacle_code["Add Rigid Wall"]
            self.env[math.floor(x_current), math.ceil(y_current), 0] = self.environment_obstacle_code["Add Rigid Wall"]
            self.env[math.ceil(x_current), math.floor(y_current), 0] = self.environment_obstacle_code["Add Rigid Wall"]
            current_orthogonal_swipe_point = current_point
            while np.linalg.norm(current_orthogonal_swipe_point - current_point) < wall_depth:
                x_current = current_orthogonal_swipe_point[0]
                y_current = current_orthogonal_swipe_point[1]
                self.env[math.floor(x_current), math.floor(y_current), 0] = self.environment_obstacle_code["Add Rigid Wall"]
                self.env[math.ceil(x_current), math.ceil(y_current), 0] = self.environment_obstacle_code["Add Rigid Wall"]
                self.env[math.floor(x_current), math.ceil(y_current), 0] = self.environment_obstacle_code["Add Rigid Wall"]
                self.env[math.ceil(x_current), math.floor(y_current), 0] = self.environment_obstacle_code["Add Rigid Wall"]
                self.env[math.floor(x_current), math.floor(y_current), 1:3] = orthogonal_unit_vector*wall_strength
                self.env[math.ceil(x_current), math.ceil(y_current), 1:3] = orthogonal_unit_vector*wall_strength
                self.env[math.floor(x_current), math.ceil(y_current), 1:3] = orthogonal_unit_vector*wall_strength
                self.env[math.ceil(x_current), math.floor(y_current), 1:3] = orthogonal_unit_vector*wall_strength

                self.env[math.floor(x_current), math.floor(y_current), 3] = damping_factor
                self.env[math.ceil(x_current), math.ceil(y_current), 3] = damping_factor
                self.env[math.floor(x_current), math.ceil(y_current), 3] = damping_factor
                self.env[math.ceil(x_current), math.floor(y_current), 3] = damping_factor
                
                current_orthogonal_swipe_point = current_orthogonal_swipe_point + orthogonal_unit_vector
            current_point = current_point + direction_unit_vector

        # coef_a = (point_2[1] - point_1[1])/(point_2[0] - point_1[0])
        # coef_b = point_2[1] - point_2[0]*coef_a
        # print(coef_a)
        # print(coef_b)
        # print(coef_a*point_1[0] + coef_b)
        # print(coef_a*point_2[0] + coef_b)
        # lower_x_segment = math.floor(min(point_1[0], point_2[0]))
        # upper_x_segment = math.ceil(max(point_1[0], point_2[0]))
        # for i in range(lower_x_segment, upper_x_segment+1):
        #     current_y = coef_a*i + coef_b
        #     self.env[i, math.floor(current_y), 0] = self.environment_obstacle_code["Add Rigid Wall"]
        #     self.env[i, math.ceil(current_y), 0] = self.environment_obstacle_code["Add Rigid Wall"]
        #     print("Doing")

        
        # center = np.array(center)
        # lower_limit_x = max(0, round(center[0]-radius))
        # upper_limit_x = min(self.x_upper_limit, round(center[0]+radius))
        # lower_limit_y = max(0, round(center[1]-radius))
        # upper_limit_y = min(self.y_upper_limit, round(center[1]+radius))
        # for i in range(lower_limit_x, upper_limit_x + 1):
        #     for j in range(lower_limit_y, upper_limit_y + 1):
        #         current_point = np.array([i, j])
        #         if np.linalg.norm(current_point-center) < radius:
        #             self.env[i, j, 0] = self.environment_obstacle_code["Add Damping Area"]
        #             self.env[i, j, 3] = damping_factor

    def addFacilitateArea(self, pointlist: np.ndarray, facilitate_coefficient = 15):
        # y = ax + b
        point_1 = pointlist[0]
        point_2 = pointlist[1]
        point_3 = pointlist[2]
        direction_unit_vector = point_2 - point_1
        segment_length = np.linalg.norm(direction_unit_vector)
        direction_unit_vector = direction_unit_vector/segment_length

        edge_vector = point_3 - point_2
        orthogonal_unit_vector = np.r_[-direction_unit_vector[1], direction_unit_vector[0]]
        orthogonal_unit_vector = np.sign(np.dot(orthogonal_unit_vector, edge_vector)) * orthogonal_unit_vector

        facilitate_area_length = np.linalg.norm(np.cross(edge_vector, direction_unit_vector))

        current_point = point_1
        while np.linalg.norm(current_point - point_1) < segment_length:
            x_current = current_point[0]
            y_current = current_point[1]
            self.env[math.floor(x_current), math.floor(y_current), 0] = self.environment_obstacle_code["Add Facilitate Area"]
            self.env[math.ceil(x_current), math.ceil(y_current), 0] = self.environment_obstacle_code["Add Facilitate Area"]
            self.env[math.floor(x_current), math.ceil(y_current), 0] = self.environment_obstacle_code["Add Facilitate Area"]
            self.env[math.ceil(x_current), math.floor(y_current), 0] = self.environment_obstacle_code["Add Facilitate Area"]
            current_orthogonal_swipe_point = current_point
            while np.linalg.norm(current_orthogonal_swipe_point - current_point) < facilitate_area_length:
                x_current = current_orthogonal_swipe_point[0]
                y_current = current_orthogonal_swipe_point[1]
                self.env[math.floor(x_current), math.floor(y_current), 0] = self.environment_obstacle_code["Add Facilitate Area"]
                self.env[math.ceil(x_current), math.ceil(y_current), 0] = self.environment_obstacle_code["Add Facilitate Area"]
                self.env[math.floor(x_current), math.ceil(y_current), 0] = self.environment_obstacle_code["Add Facilitate Area"]
                self.env[math.ceil(x_current), math.floor(y_current), 0] = self.environment_obstacle_code["Add Facilitate Area"]
                self.env[math.floor(x_current), math.floor(y_current), 4:6] = orthogonal_unit_vector*facilitate_coefficient
                self.env[math.ceil(x_current), math.ceil(y_current), 4:6] = orthogonal_unit_vector*facilitate_coefficient
                self.env[math.floor(x_current), math.ceil(y_current), 4:6] = orthogonal_unit_vector*facilitate_coefficient
                self.env[math.ceil(x_current), math.floor(y_current), 4:6] = orthogonal_unit_vector*facilitate_coefficient

                # self.env[math.floor(x_current), math.floor(y_current), 3] = damping_factor
                # self.env[math.ceil(x_current), math.ceil(y_current), 3] = damping_factor
                # self.env[math.floor(x_current), math.ceil(y_current), 3] = damping_factor
                # self.env[math.ceil(x_current), math.floor(y_current), 3] = damping_factor
                
                current_orthogonal_swipe_point = current_orthogonal_swipe_point + orthogonal_unit_vector
            current_point = current_point + direction_unit_vector

        # coef_a = (point_2[1] - point_1[1])/(point_2[0] - point_1[0])
        # coef_b = point_2[1] - point_2[0]*coef_a
        # print(coef_a)
        # print(coef_b)
        # print(coef_a*point_1[0] + coef_b)
        # print(coef_a*point_2[0] + coef_b)
        # lower_x_segment = math.floor(min(point_1[0], point_2[0]))
        # upper_x_segment = math.ceil(max(point_1[0], point_2[0]))
        # for i in range(lower_x_segment, upper_x_segment+1):
        #     current_y = coef_a*i + coef_b
        #     self.env[i, math.floor(current_y), 0] = self.environment_obstacle_code["Add Rigid Wall"]
        #     self.env[i, math.ceil(current_y), 0] = self.environment_obstacle_code["Add Rigid Wall"]
        #     print("Doing")

        
        # center = np.array(center)
        # lower_limit_x = max(0, round(center[0]-radius))
        # upper_limit_x = min(self.x_upper_limit, round(center[0]+radius))
        # lower_limit_y = max(0, round(center[1]-radius))
        # upper_limit_y = min(self.y_upper_limit, round(center[1]+radius))
        # for i in range(lower_limit_x, upper_limit_x + 1):
        #     for j in range(lower_limit_y, upper_limit_y + 1):
        #         current_point = np.array([i, j])
        #         if np.linalg.norm(current_point-center) < radius:
        #             self.env[i, j, 0] = self.environment_obstacle_code["Add Damping Area"]
        #             self.env[i, j, 3] = damping_factor

    def addRoughWall(self, pointlist: np.ndarray, wall_depth = 10, wall_strength = 2, roughness = 0.2, damping_factor = -15):
        # y = ax + b
        point_1 = pointlist[0]
        point_2 = pointlist[1]
        point_3 = pointlist[2]
        direction_unit_vector = point_2 - point_1
        segment_length = np.linalg.norm(direction_unit_vector)
        direction_unit_vector = direction_unit_vector/segment_length

        orthogonal_unit_vector = np.r_[-direction_unit_vector[1], direction_unit_vector[0]]
        orthogonal_unit_vector = np.sign(np.dot(orthogonal_unit_vector, point_3 - point_2)) * orthogonal_unit_vector

        current_point = point_1
        while np.linalg.norm(current_point - point_1) < segment_length:
            x_current = current_point[0]
            y_current = current_point[1]
            self.env[math.floor(x_current), math.floor(y_current), 0] = self.environment_obstacle_code["Add Rigid Wall"]
            self.env[math.ceil(x_current), math.ceil(y_current), 0] = self.environment_obstacle_code["Add Rigid Wall"]
            self.env[math.floor(x_current), math.ceil(y_current), 0] = self.environment_obstacle_code["Add Rigid Wall"]
            self.env[math.ceil(x_current), math.floor(y_current), 0] = self.environment_obstacle_code["Add Rigid Wall"]
            current_orthogonal_swipe_point = current_point
            random_roughness = np.random.rand()*roughness*wall_depth
            while np.linalg.norm(current_orthogonal_swipe_point - current_point) < wall_depth + random_roughness:
                x_current = current_orthogonal_swipe_point[0]
                y_current = current_orthogonal_swipe_point[1]
                self.env[math.floor(x_current), math.floor(y_current), 0] = self.environment_obstacle_code["Add Rigid Wall"]
                self.env[math.ceil(x_current), math.ceil(y_current), 0] = self.environment_obstacle_code["Add Rigid Wall"]
                self.env[math.floor(x_current), math.ceil(y_current), 0] = self.environment_obstacle_code["Add Rigid Wall"]
                self.env[math.ceil(x_current), math.floor(y_current), 0] = self.environment_obstacle_code["Add Rigid Wall"]
                self.env[math.floor(x_current), math.floor(y_current), 1:3] = orthogonal_unit_vector*wall_strength
                self.env[math.ceil(x_current), math.ceil(y_current), 1:3] = orthogonal_unit_vector*wall_strength
                self.env[math.floor(x_current), math.ceil(y_current), 1:3] = orthogonal_unit_vector*wall_strength
                self.env[math.ceil(x_current), math.floor(y_current), 1:3] = orthogonal_unit_vector*wall_strength

                self.env[math.floor(x_current), math.floor(y_current), 3] = damping_factor
                self.env[math.ceil(x_current), math.ceil(y_current), 3] = damping_factor
                self.env[math.floor(x_current), math.ceil(y_current), 3] = damping_factor
                self.env[math.ceil(x_current), math.floor(y_current), 3] = damping_factor
                
                current_orthogonal_swipe_point = current_orthogonal_swipe_point + orthogonal_unit_vector
            current_point = current_point + direction_unit_vector

        # coef_a = (point_2[1] - point_1[1])/(point_2[0] - point_1[0])
        # coef_b = point_2[1] - point_2[0]*coef_a
        # print(coef_a)
        # print(coef_b)
        # print(coef_a*point_1[0] + coef_b)
        # print(coef_a*point_2[0] + coef_b)
        # lower_x_segment = math.floor(min(point_1[0], point_2[0]))
        # upper_x_segment = math.ceil(max(point_1[0], point_2[0]))
        # for i in range(lower_x_segment, upper_x_segment+1):
        #     current_y = coef_a*i + coef_b
        #     self.env[i, math.floor(current_y), 0] = self.environment_obstacle_code["Add Rigid Wall"]
        #     self.env[i, math.ceil(current_y), 0] = self.environment_obstacle_code["Add Rigid Wall"]
        #     print("Doing")

        
        # center = np.array(center)
        # lower_limit_x = max(0, round(center[0]-radius))
        # upper_limit_x = min(self.x_upper_limit, round(center[0]+radius))
        # lower_limit_y = max(0, round(center[1]-radius))
        # upper_limit_y = min(self.y_upper_limit, round(center[1]+radius))
        # for i in range(lower_limit_x, upper_limit_x + 1):
        #     for j in range(lower_limit_y, upper_limit_y + 1):
        #         current_point = np.array([i, j])
        #         if np.linalg.norm(current_point-center) < radius:
        #             self.env[i, j, 0] = self.environment_obstacle_code["Add Damping Area"]
        #             self.env[i, j, 3] = damping_factor


###########################################
    def draw_on_image(self):
        # Create fig, axe
        self.fig, self.ax = plt.subplots()
        self.ax.set_position([0.6, 0.05, 0.35, 0.9])

        # Declare custom variables
        self.button_manager = MyButton()
        # Define your custom colors
        custom_colors = ['#FF0000', '#00FF00', '#0000FF', '#FFFF00', '#FF00FF', '#00FFFF', '#FFA500', '#800080', '#008080', '#800000']
        self.custom_cmap = mcolors.ListedColormap(custom_colors)
        # self.drawing = False  # Flag to indicate if the mouse button is being held
      
        # Add a button to the figure
        # Add validate button
        self.button_manager.create_button(axes=[0.30, 0.85, 0.2, 0.04], text="Validate", connection=self.ValidateEnvironment, clearfunction = self.ValidateEnvironmentClear)
        # Add circle spring
        self.button_manager.create_button(axes=[0.05, 0.80, 0.2, 0.04], text="Add Circle Spring", connection=self.UIaddCircleSpring, clearfunction = self.UIaddCircleSpringClear)
        # Add damping areas
        self.button_manager.create_button(axes=[0.05, 0.75, 0.2, 0.04], text="Add Damping Area", connection=self.UIaddDampingArea, clearfunction = self.UIaddDampingAreaClear)
        # Add rigid wall
        self.button_manager.create_button(axes=[0.05, 0.70, 0.2, 0.04], text="Add Rigid Wall", connection=self.UIaddRigidWall, clearfunction = self.UIaddRigidWallClear)
        # Add facilitate Area
        self.button_manager.create_button(axes=[0.05, 0.65, 0.2, 0.04], text="Add Facilitate Area", connection=self.UIaddFacilitateArea, clearfunction = self.UIaddFacilitateAreaClear)
        # Add rough wall
        self.button_manager.create_button(axes=[0.05, 0.60, 0.2, 0.04], text="Add Rough Wall", connection=self.UIaddRoughWall, clearfunction = self.UIaddRoughWallClear)

        # Add export button
        self.button_manager.create_button(axes=[0.05, 0.90, 0.2, 0.04], text="Export", connection=self.ExportEnvironment)
        # Add reset button
        self.button_manager.create_button(axes=[0.05, 0.85, 0.2, 0.04], text="Reset", connection=self.ResetEnvironment)
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

###########################################
    def ValidateEnvironment(self, event):
        self.clear_all_UI_subWidgets()
        # Congigurate the button
        self.button_manager.turn_on_button(text="Validate")
        # Connect to main code
        self.ValidateEnvironmentButtonConnection = self.fig.canvas.mpl_connect('button_press_event', self.on_mouse_press_ValidateEnvironment)
    def on_mouse_press_ValidateEnvironment(self, event):
        if event.inaxes == self.ax:
            position = [int(event.ydata), int(event.xdata)]
            print(self.env[position[0],position[1]])
    def ValidateEnvironmentClear(self):
        # Anything that is still not clean
        self.fig.canvas.mpl_disconnect(self.ValidateEnvironmentButtonConnection)
        pass

###########################################
    def UIaddCircleSpring(self, event):
        self.clear_all_UI_subWidgets()
        # Congigurate the button
        self.button_manager.turn_on_button(text="Add Circle Spring")
        # Add widgets
        self.slide_ax_1 = plt.axes([0.30, 0.05, 0.15, 0.03])  # [left, bottom, width, height]
        self.slide_ax_2 = plt.axes([0.30, 0.10, 0.15, 0.03])  # [left, bottom, width, height]
        self.slide_ax_3 = plt.axes([0.30, 0.15, 0.15, 0.03])  # [left, bottom, width, height]

        self.slider_1 = Slider(self.slide_ax_1, 'Ky ', 0, 500, valinit=250)
        self.slider_2 = Slider(self.slide_ax_2, 'Kx ', 0, 500, valinit=250)
        self.slider_3 = Slider(self.slide_ax_3, 'Radius ', 0, 100, valinit=10)
        # Store widget
        self.button_manager.add_dependent_widgets_axes("Add Circle Spring", self.slide_ax_1, self.slide_ax_2, self.slide_ax_3)
        self.button_manager.add_dependent_widgets_objects("Add Circle Spring", self.slider_1, self.slider_2, self.slider_3)

        # Connect to action
        self.UIaddCircleSpringButtonConnection = self.fig.canvas.mpl_connect('button_press_event', self.on_mouse_press_UIaddCircleSpring)
    def on_mouse_press_UIaddCircleSpring(self, event):
        if event.inaxes == self.ax:
            position = [int(event.ydata), int(event.xdata)]
            self.addCircleSpring(center = position, k_matrix=np.array([[self.slider_1.val, 0],[0,self.slider_2.val]]), radius = self.slider_3.val)
            self.updateFigure()
    def UIaddCircleSpringClear(self):
        # Anything that is still not clean
        self.fig.canvas.mpl_disconnect(self.UIaddCircleSpringButtonConnection)
        pass

###########################################
    def UIaddDampingArea(self, event):
        self.clear_all_UI_subWidgets()
        # Congigurate the button
        self.button_manager.turn_on_button(text="Add Damping Area")
        # Add widgets
        self.slide_ax_1 = plt.axes([0.30, 0.05, 0.15, 0.03])  # [left, bottom, width, height]
        self.slide_ax_2 = plt.axes([0.30, 0.10, 0.15, 0.03])  # [left, bottom, width, height]
        # self.slide_ax_3 = plt.axes([0.10, 0.15, 0.15, 0.03])  # [left, bottom, width, height]

        self.slider_1 = Slider(self.slide_ax_1, 'Damping coefficient ', -30, 30, valinit=-20)
        self.slider_2 = Slider(self.slide_ax_2, 'Radius ', 0, 100, valinit=10)
        # self.slider_3 = Slider(self.slide_ax_3, 'Radius', 0, 100, valinit=10)
        # Store widget
        self.button_manager.add_dependent_widgets_axes("Add Damping Area", self.slide_ax_1, self.slide_ax_2)
        self.button_manager.add_dependent_widgets_objects("Add Damping Area", self.slider_1, self.slider_2)

        # Connect to action
        self.UIaddDampingAreaButtonConnection = self.fig.canvas.mpl_connect('button_press_event', self.on_mouse_press_UIaddDampingArea)
    def on_mouse_press_UIaddDampingArea(self, event):
        if event.inaxes == self.ax:
            position = [int(event.ydata), int(event.xdata)]
            self.addDampingArea(center=position, damping_factor = self.slider_1.val, radius = self.slider_2.val)
            # Store the point for further processing
            self.updateFigure()
    def UIaddDampingAreaClear(self):
        # Anything that is still not clean
        self.fig.canvas.mpl_disconnect(self.UIaddDampingAreaButtonConnection)
        pass

###########################################
    def UIaddRigidWall(self, event):
        self.clear_all_UI_subWidgets()
        # Congigurate the button
        self.button_manager.turn_on_button(text="Add Rigid Wall")
        # Add widgets
        self.slide_ax_1 = plt.axes([0.30, 0.05, 0.15, 0.03])  # [left, bottom, width, height]
        self.slide_ax_2 = plt.axes([0.30, 0.10, 0.15, 0.03])  # [left, bottom, width, height]
        # self.slide_ax_3 = plt.axes([0.10, 0.15, 0.15, 0.03])  # [left, bottom, width, height]

        self.slider_1 = Slider(self.slide_ax_1, 'Wall Depth', 10, 50, valinit=20)
        self.slider_2 = Slider(self.slide_ax_2, 'Wall Strength ', 0, 2, valinit=2)
        self.UIaddRigidWallClickPointStorage = np.empty((0,2))
        self.UIaddRigidWallClickPointPlot, = self.ax.plot([], [], 'ko', markersize = 1)
        # self.slider_3 = Slider(self.slide_ax_3, 'Radius', 0, 100, valinit=10)
        # Store widget
        self.button_manager.add_dependent_widgets_axes("Add Rigid Wall", self.slide_ax_1, self.slide_ax_2)
        self.button_manager.add_dependent_widgets_objects("Add Rigid Wall", self.slider_1, self.slider_2)

        # Connect to action
        self.UIaddRigidWallButtonConnection = self.fig.canvas.mpl_connect('button_press_event', self.on_mouse_press_UIaddRigidWall)
    def on_mouse_press_UIaddRigidWall(self, event):
        if event.inaxes == self.ax:
            position = np.array([[int(event.ydata), int(event.xdata)]])
            # position_list_type = [int(event.ydata), int(event.xdata)]
            # self.addDampingArea(center=position_list_type, damping_factor = self.slider_1.val, radius = self.slider_2.val)
            # Store the point for further processing
            self.UIaddRigidWallClickPointStorage = np.append(self.UIaddRigidWallClickPointStorage, position, axis=0)
            self.UIaddRigidWallClickPointPlot.set_data(self.UIaddRigidWallClickPointStorage.T[1], self.UIaddRigidWallClickPointStorage.T[0])
            if len(self.UIaddRigidWallClickPointStorage)==3:
                self.addRigidWall(pointlist=self.UIaddRigidWallClickPointStorage, wall_depth=self.slider_1.val, wall_strength=self.slider_2.val)
                self.UIaddRigidWallClickPointStorage = np.empty((0,2))
            self.updateFigure()
    def UIaddRigidWallClear(self):
        # Anything that is still not clean
        self.fig.canvas.mpl_disconnect(self.UIaddRigidWallButtonConnection)
        self.UIaddRigidWallClickPointPlot.remove()
        pass

###########################################
    def UIaddFacilitateArea(self, event):
        self.clear_all_UI_subWidgets()
        # Congigurate the button
        self.button_manager.turn_on_button(text="Add Facilitate Area")
        # Add widgets
        self.slide_ax_1 = plt.axes([0.30, 0.05, 0.15, 0.03])  # [left, bottom, width, height]
        # self.slide_ax_2 = plt.axes([0.30, 0.10, 0.15, 0.03])  # [left, bottom, width, height]
        # self.slide_ax_3 = plt.axes([0.10, 0.15, 0.15, 0.03])  # [left, bottom, width, height]

        self.slider_1 = Slider(self.slide_ax_1, 'Facilitate Coefficient ', -30, 30, valinit=20)
        # self.slider_2 = Slider(self.slide_ax_2, 'Radius ', 0, 100, valinit=10)
        self.UIaddFacilitateAreaClickPointStorage = np.empty((0,2))
        self.UIaddFacilitateAreaClickPointPlot, = self.ax.plot([], [], 'ko', markersize = 1)
        # self.slider_3 = Slider(self.slide_ax_3, 'Radius', 0, 100, valinit=10)
        # Store widget
        self.button_manager.add_dependent_widgets_axes("Add Facilitate Area", self.slide_ax_1)
        self.button_manager.add_dependent_widgets_objects("Add Facilitate Area", self.slider_1)

        # Connect to action
        self.UIaddFacilitateAreaButtonConnection = self.fig.canvas.mpl_connect('button_press_event', self.on_mouse_press_UIaddFacilitateArea)
    def on_mouse_press_UIaddFacilitateArea(self, event):
        if event.inaxes == self.ax:
            position = np.array([[int(event.ydata), int(event.xdata)]])
            # position_list_type = [int(event.ydata), int(event.xdata)]
            # self.addDampingArea(center=position_list_type, damping_factor = self.slider_1.val, radius = self.slider_2.val)
            # Store the point for further processing
            self.UIaddFacilitateAreaClickPointStorage = np.append(self.UIaddFacilitateAreaClickPointStorage, position, axis=0)
            self.UIaddFacilitateAreaClickPointPlot.set_data(self.UIaddFacilitateAreaClickPointStorage.T[1], self.UIaddFacilitateAreaClickPointStorage.T[0])
            if len(self.UIaddFacilitateAreaClickPointStorage)==3:
                self.addFacilitateArea(pointlist=self.UIaddFacilitateAreaClickPointStorage, facilitate_coefficient=self.slider_1.val)
                self.UIaddFacilitateAreaClickPointStorage = np.empty((0,2))
            self.updateFigure()
    def UIaddFacilitateAreaClear(self):
        # Anything that is still not clean
        self.fig.canvas.mpl_disconnect(self.UIaddFacilitateAreaButtonConnection)
        self.UIaddFacilitateAreaClickPointPlot.remove()
        pass

###########################################
    def UIaddRoughWall(self, event):
        self.clear_all_UI_subWidgets()
        # Congigurate the button
        self.button_manager.turn_on_button(text="Add Rough Wall")
        # Add widgets
        self.slide_ax_1 = plt.axes([0.30, 0.05, 0.15, 0.03])  # [left, bottom, width, height]
        self.slide_ax_2 = plt.axes([0.30, 0.10, 0.15, 0.03])  # [left, bottom, width, height]
        self.slide_ax_3 = plt.axes([0.30, 0.15, 0.15, 0.03])  # [left, bottom, width, height]

        self.slider_1 = Slider(self.slide_ax_1, 'Wall Depth', 10, 50, valinit=20)
        self.slider_2 = Slider(self.slide_ax_2, 'Wall Strength ', 0, 2, valinit=2)
        self.slider_3 = Slider(self.slide_ax_3, 'Roughness', 0, 0.5, valinit=0.3)
        self.UIaddRoughWallClickPointStorage = np.empty((0,2))
        self.UIaddRoughWallClickPointPlot, = self.ax.plot([], [], 'ko', markersize = 1)
        # self.slider_3 = Slider(self.slide_ax_3, 'Radius', 0, 100, valinit=10)
        # Store widget
        self.button_manager.add_dependent_widgets_axes("Add Rough Wall", self.slide_ax_1, self.slide_ax_2, self.slide_ax_3)
        self.button_manager.add_dependent_widgets_objects("Add Rough Wall", self.slider_1, self.slider_2, self.slider_3)

        # Connect to action
        self.UIaddRoughWallButtonConnection = self.fig.canvas.mpl_connect('button_press_event', self.on_mouse_press_UIaddRoughWall)
    def on_mouse_press_UIaddRoughWall(self, event):
        if event.inaxes == self.ax:
            position = np.array([[int(event.ydata), int(event.xdata)]])
            # position_list_type = [int(event.ydata), int(event.xdata)]
            # self.addDampingArea(center=position_list_type, damping_factor = self.slider_1.val, radius = self.slider_2.val)
            # Store the point for further processing
            self.UIaddRoughWallClickPointStorage = np.append(self.UIaddRoughWallClickPointStorage, position, axis=0)
            self.UIaddRoughWallClickPointPlot.set_data(self.UIaddRoughWallClickPointStorage.T[1], self.UIaddRoughWallClickPointStorage.T[0])
            if len(self.UIaddRoughWallClickPointStorage)==3:
                self.addRoughWall(pointlist=self.UIaddRoughWallClickPointStorage, wall_depth=self.slider_1.val, wall_strength=self.slider_2.val, roughness = self.slider_3.val)
                self.UIaddRoughWallClickPointStorage = np.empty((0,2))
            self.updateFigure()
    def UIaddRoughWallClear(self):
        # Anything that is still not clean
        self.fig.canvas.mpl_disconnect(self.UIaddRoughWallButtonConnection)
        self.UIaddRoughWallClickPointPlot.remove()
        pass

############### SOME STORE CODE

############### SOME STORE CODE
    # def UIaddDampingArea(self, event):
    #     self.clear_all_UI_subWidgets()
    #     # Congigurate the button
    #     self.button_manager.turn_on_button(text="Add Damping Area")
    #     # Add widgets
    #     self.slide_ax_1 = plt.axes([0.30, 0.05, 0.15, 0.03])  # [left, bottom, width, height]
    #     self.slide_ax_2 = plt.axes([0.30, 0.10, 0.15, 0.03])  # [left, bottom, width, height]
    #     # self.slide_ax_3 = plt.axes([0.10, 0.15, 0.15, 0.03])  # [left, bottom, width, height]

    #     self.slider_1 = Slider(self.slide_ax_1, 'Damping coefficient ', -15, 15, valinit=-1)
    #     self.slider_2 = Slider(self.slide_ax_2, 'Radius ', 0, 100, valinit=10)
    #     self.UIaddDampingAreaClickPointStorage = np.empty((0,2))
    #     self.UIaddDampingAreaClickPointPlot, = self.ax.plot([], [], 'ko', markersize = 1)
    #     # self.slider_3 = Slider(self.slide_ax_3, 'Radius', 0, 100, valinit=10)
    #     # Store widget
    #     self.button_manager.add_dependent_widgets_axes("Add Damping Area", self.slide_ax_1, self.slide_ax_2)
    #     self.button_manager.add_dependent_widgets_objects("Add Damping Area", self.slider_1, self.slider_2)

    #     # Connect to action
    #     self.UIaddDampingAreaButtonConnection = self.fig.canvas.mpl_connect('button_press_event', self.on_mouse_press_UIaddDampingArea)
    # def on_mouse_press_UIaddDampingArea(self, event):
    #     if event.inaxes == self.ax:
    #         position = np.array([[int(event.ydata), int(event.xdata)]])
    #         position_list_type = [int(event.ydata), int(event.xdata)]
    #         self.addDampingArea(center=position_list_type, damping_factor = self.slider_1.val, radius = self.slider_2.val)
    #         # Store the point for further processing
    #         self.UIaddDampingAreaClickPointStorage = np.append(self.UIaddDampingAreaClickPointStorage, position, axis=0)
    #         self.UIaddDampingAreaClickPointPlot.set_data(self.UIaddDampingAreaClickPointStorage.T[1], self.UIaddDampingAreaClickPointStorage.T[0])
    #         self.updateFigure()
    # def UIaddDampingAreaClear(self):
    #     # Anything that is still not clean
    #     self.fig.canvas.mpl_disconnect(self.UIaddDampingAreaButtonConnection)
    #     self.UIaddDampingAreaClickPointPlot.remove()
    #     pass

############### SOME STORE CODE


###########################################
    def clear_all_UI_subWidgets(self):
        # if self.UIaddCircleSpringMode:
        #     self.UIaddCircleSpringClear()
        self.button_manager.turn_off_all_button()
        self.updateFigure()
    def updateFigure(self):
        # Update the figure
        self.image_array = self.env[:,:,0]
        self.ax.imshow(self.image_array.astype(np.uint8), cmap=self.custom_cmap)
        self.fig.canvas.draw()
    def ResetEnvironment(self, event):
        self.env = np.zeros(self.dimension)
        self.clear_all_UI_subWidgets()
    def ExportEnvironment(self, event):
        export_array = np.copy(self.env)
        # Transform to robot space
        support_variable = np.copy(export_array[:,:,1])
        export_array[:,:,1] = np.copy(export_array[:,:,2])
        export_array[:,:,2] = -support_variable
        # Transform to robot space
        support_variable = np.copy(export_array[:,:,4])
        export_array[:,:,4] = np.copy(export_array[:,:,5])
        export_array[:,:,5] = -support_variable
        # Save file
        export_array = export_array.tolist()
        with open('Environment.pk1', 'wb') as file:
            pickle.dump(export_array, file)
        print("Successful!!")
        # Create a pop-up message box
        messagebox.showinfo("Congratulation", "Export finished!!")

        
class MyButton:
    def __init__(self):
        self.button_name_list = [] #Name - text
        self.button_mode_dict = {} #Mode of button on or off
        self.button_ax_dict = {} #Coordinate
        self.buttons_dict = {} #Button object
        self.buttons_connection_dict = {} #Button connection
        self.buttons_widgets_axes = {} #Button widget axes
        self.buttons_widgets_objects = {} #Button widget
        self.buttons_clear_function_dict = {} #Button clear function

    def create_button(self, axes = [0.0, 0.0, 0.5, 0.5], text = "Hello", connection = lambda event: print("Default Function"), clearfunction = lambda event: print("Default Function")):
        # Create button
        axes = plt.axes(axes)
        button = Button(axes, text)
        button.on_clicked(connection)
        # Store the button information
        self.button_name_list.append(text)
        self.button_mode_dict[text] = False
        self.button_ax_dict[text] = axes
        self.buttons_dict[text] = button
        self.buttons_connection_dict[text] = connection
        self.buttons_clear_function_dict[text] = clearfunction
        self.buttons_widgets_axes[text] = []
        self.buttons_widgets_objects[text] = [] #Button widget

    def turn_on_button(self, text):
        self.button_mode_dict[text] = True
        button = self.buttons_dict[text]
        button.color = 'red'
        button.label.set_color('white')

    def turn_off_button(self, text):
        self.button_mode_dict[text] = False
        button = self.buttons_dict[text]
        button.color = "0.85"
        button.label.set_color('black')

    def turn_off_all_button(self):
        for text in self.button_name_list:
            if self.button_mode_dict[text]:
                self.turn_off_button(text)
                self.remove_dependent_widgets_axes(text)
                self.remove_dependent_widgets_objects(text)
                self.buttons_clear_function_dict[text]()
    
    def add_dependent_widgets_axes(self, text, *widget_axes):
        self.buttons_widgets_axes[text] = []
        for widget_ax in widget_axes:
            self.buttons_widgets_axes[text].append(widget_ax)

    def add_dependent_widgets_objects(self, text, *widget_objects):
        self.buttons_widgets_objects[text] = []
        for widget_object in widget_objects:
            self.buttons_widgets_objects[text].append(widget_object)
    
    def remove_dependent_widgets_axes(self, text):
        for ax in self.buttons_widgets_axes[text]:
            ax.remove()

    def remove_dependent_widgets_objects(self, text):
        for object in self.buttons_widgets_objects[text]:
            object.active = False



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



