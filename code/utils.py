import cv2
import numpy as np
import pygame
from pygame.locals import *
from OpenGL.GL import *
from OpenGL.GLU import *

class AHRSdisplay:
    def __init__(self, width=800, height=600):
        self.width = width
        self.height = height
        pygame.init()
        self.screen = pygame.display.set_mode((self.width, self.height), DOUBLEBUF | OPENGL | HIDDEN)
        gluPerspective(45, (self.width / self.height), 0.1, 50.0)
        glTranslatef(0.0, 0.0, -5)
        self.font = pygame.font.SysFont('Arial', 36)  # Larger font size
        self.rot_matrix = np.identity(3)
        
        self.screen_image = None

    def euler_to_rot_matrix(self, yaw, pitch, roll):
        yaw = np.radians(-yaw + 50)
        pitch = np.radians(-pitch)
        roll = np.radians(-roll)

        R_yaw = np.array([
            [np.cos(yaw), -np.sin(yaw), 0],
            [np.sin(yaw), np.cos(yaw), 0],
            [0, 0, 1]
        ])

        R_pitch = np.array([
            [np.cos(pitch), 0, np.sin(pitch)],
            [0, 1, 0],
            [-np.sin(pitch), 0, np.cos(pitch)]
        ])

        R_roll = np.array([
            [1, 0, 0],
            [0, np.cos(roll), -np.sin(roll)],
            [0, np.sin(roll), np.cos(roll)]
        ])

        self.rot_matrix = R_yaw @ R_pitch @ R_roll

    def to_4x4_matrix(self):
        transform_matrix = np.identity(4)
        transform_matrix[:3, :3] = self.rot_matrix
        return transform_matrix

    def draw_prism(self):
        vertices = [
            [-1, -0.5, -0.25],
            [-1, -0.5, 0.25],
            [-1, 0.5, -0.25],
            [-1, 0.5, 0.25],
            [1, -0.5, -0.25],
            [1, -0.5, 0.25],
            [1, 0.5, -0.25],
            [1, 0.5, 0.25]
        ]

        edges = [
            (0, 1), (1, 3), (3, 2), (2, 0), 
            (4, 5), (5, 7), (7, 6), (6, 4),
            (0, 4), (1, 5), (2, 6), (3, 7)
        ]

        glBegin(GL_LINES)
        for edge in edges:
            for vertex in edge:
                glVertex3fv(vertices[vertex])
        glEnd()

    def draw_vector(self, vector, color):
        glColor3fv(color)
        glBegin(GL_LINES)
        glVertex3fv([0, 0, 0])
        glVertex3fv(vector)
        glEnd()
        glColor3fv([1, 1, 1])  # Reset to white

    def draw_text(self, text, position):
        text_surface = self.font.render(text, True, (255, 255, 255))  # White text
        text_data = pygame.image.tostring(text_surface, "RGBA", True)
        glWindowPos2d(position[0], position[1])
        glDrawPixels(text_surface.get_width(), text_surface.get_height(), GL_RGBA, GL_UNSIGNED_BYTE, text_data)

    def draw_cone(self):
        quad = gluNewQuadric()
        glColor3f(1.0, 0.0, 0.0)  # Red color for the cone
        glPushMatrix()
        glTranslatef(1.0, 0.0, 0.0)  # Move cone to the front of the model
        glRotatef(90, 0, 1, 0)  # Rotate to align with the X-axis
        gluCylinder(quad, 0.1, 0, 0.3, 32, 32)
        glPopMatrix()
        glColor3fv([1, 1, 1])  # Reset to white

    def update_orientation(self, yaw, pitch, roll):
        self.euler_to_rot_matrix(yaw, pitch, roll)
        transform_matrix = self.to_4x4_matrix()

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glPushMatrix()

        # Apply rotation matrix
        glMultMatrixf(transform_matrix.T)

        self.draw_prism()
        self.draw_cone()

        # Draw direction vectors
        roll_vector = self.rot_matrix @ np.array([1, 0, 0])
        pitch_vector = self.rot_matrix @ np.array([0, 1, 0])
        yaw_vector = self.rot_matrix @ np.array([0, 0, 1])

        self.draw_vector(roll_vector, [1, 0, 0])  # Red
        self.draw_vector(pitch_vector, [0, 1, 0])  # Green
        self.draw_vector(yaw_vector, [0, 0, 1])  # Blue

        glPopMatrix()

        # # Draw the text labels
        # glPushMatrix()
        # glLoadIdentity()
        # self.draw_text(f"Yaw: {yaw:.2f}", (10, self.height - 30))
        # self.draw_text(f"Pitch: {pitch:.2f}", (10, self.height - 70))
        # self.draw_text(f"Roll: {roll:.2f}", (10, self.height - 110))
        # glPopMatrix()

        # Capture the screen buffer
        buffer = glReadPixels(0, 0, self.width, self.height, GL_RGBA, GL_UNSIGNED_BYTE)
        self.screen_image = np.frombuffer(buffer, dtype=np.uint8).reshape(self.height, self.width, 4)
        self.screen_image = np.flip(self.screen_image, 0)  # Flip vertically


        # Convert to OpenCV format
        self.screen_image = cv2.cvtColor(self.screen_image, cv2.COLOR_RGBA2BGR)
        
        return self.screen_image

def euler_angles(gx, gy, gz, ax, ay, az, mx, my, mz, mag_declination):
    roll = np.atan2(ay, np.sqrt(ax * ax + az * az))
    pitch = np.atan2(ax, np.sqrt(ay * ay + az * az))

    cos_roll = np.cos(roll)
    sin_roll = np.sin(roll)
    cos_pitch = np.cos(pitch)
    sin_pitch = np.sin(pitch)

    # Tilt compensated magnetic field X
    mx = mx * cos_pitch + my * sin_roll * sin_pitch + mz * cos_roll * sin_pitch
    # // Tilt compensated magnetic field Y
    my = my * cos_roll - mz * sin_roll
    # // Magnetic Heading
    if my == 0:
        if mx < 0:
            yaw = np.pi
        else:
            yaw = 0
    else:
        yaw = np.arctan2(-my, -mx)

        yaw += mag_declination * np.pi / 180

    if yaw > np.pi:
        yaw -= (2 * np.pi)
    elif yaw < -np.pi:
        yaw += (2 * np.pi)
    # Convert everything from radians to degrees:
    yaw *= 180.0 / np.pi
    pitch *= 180.0 / np.pi
    roll *= 180.0 / np.pi


    return yaw, pitch, roll

def draw_compass(resized_rgb, yaw, height, width, position, r):
    # Yaw display (Compass)
    compass_center_x = int(width * position[0])
    compass_center_y = int(height * position[1])
    compass_radius = r
    
    # Define the size of the black square background
    square_size = 2 * compass_radius + 20  # Add some padding around the compass
    top_left_x = compass_center_x - square_size // 2
    top_left_y = compass_center_y - square_size // 2
    bottom_right_x = compass_center_x + square_size // 2
    bottom_right_y = compass_center_y + square_size // 2
    
    overlay = resized_rgb.copy()
    alpha = 0.6
    cv2.rectangle(overlay, (top_left_x, top_left_y), (bottom_right_x, bottom_right_y), (0, 0, 0), cv2.FILLED)
    cv2.addWeighted(overlay, alpha, resized_rgb, 1 - alpha, 0, resized_rgb)
    
    directions = ['N', 'NE', 'E', 'SE', 'S', 'SW', 'W', 'NW']
    for i, direction in enumerate(directions):
        angle = np.radians(i * 45)
        dir_x = int(compass_center_x + compass_radius * np.sin(angle))
        dir_y = int(compass_center_y - compass_radius * np.cos(angle))
        cv2.putText(resized_rgb, direction, (dir_x - 5, dir_y + 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
    
    yaw_end_x = int(compass_center_x + compass_radius * np.sin(np.radians(yaw)))
    yaw_end_y = int(compass_center_y - compass_radius * np.cos(np.radians(yaw)))
    cv2.line(resized_rgb, (compass_center_x, compass_center_y), (yaw_end_x, yaw_end_y), (255, 255, 0), 2)
    