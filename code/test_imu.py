import IMU_regs
import libIMU
import pigpio
import time
import numpy as np
import utils
import cv2
import pygame

gpio_channel = 1
gpio_handler = pigpio.pi()

IMU_A_G = libIMU.AccelGyro('Accelerometer & Gyroscope', gpio_channel, IMU_regs.LSM9DS1_ADDRESS_A_G,
                           IMU_regs.WHO_AM_I_AG_RSP, gpio_handler)
IMU_M = libIMU.Magnetometer('Magnetometer', gpio_channel, IMU_regs.LSM9DS1_ADDRESS_M,
                            IMU_regs.WHO_AM_I_M_RSP, gpio_handler)
IMU_A_G.setContinuousMode()

starttime = time.time()
np.set_printoptions(formatter={'float': '{: 0.8f}'.format})

if __name__ == "__main__":

    display = utils.AHRSdisplay()

    if IMU_A_G.accelerationAvailable() and IMU_A_G.gyroscopeAvailable() and IMU_A_G.temperatureAvailable() and IMU_M.magneticFieldAvailable():
        while True:
            currtime = time.time()
            try:
                ax, ay, az, gx, gy, gz = IMU_A_G.readAccelGyro()
                mx, my, mz = IMU_M.readMagneto()

                yaw, pitch, roll = utils.euler_angles(gx, gy, gz, ax, ay, az, mx, my, mz, mag_declination = 0)
                
                yaw, pitch, roll = yaw % 360, pitch % 360, roll % 360

                if currtime - starttime >= 0.5:
                    print("Yaw, Pitch, Roll: " + ', '.join('{:0.2f}'.format(i) for i in [yaw, pitch, roll]))
                    print("\n")
                    
                    screen_image = display.update_orientation(yaw, pitch, roll)
                
                    # Draw the text labels on the screen image
                    cv2.putText(screen_image, f"Yaw: {yaw:.2f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 2, cv2.LINE_AA)
                    cv2.putText(screen_image, f"Pitch: {pitch:.2f}", (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 2, cv2.LINE_AA)
                    cv2.putText(screen_image, f"Roll: {roll:.2f}", (10, 110), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 2, cv2.LINE_AA)
                    
                    # Draw a red cross in the middle of the image
                    height, width = screen_image.shape[:2]
                    center_x, center_y = width // 2, height // 2
                    
                    utils.draw_compass(screen_image, yaw, height, width, [0.86, 0.85], 70)
                        
                        
                    # Display with OpenCV
                    cv2.imshow('OpenGL to OpenCV', screen_image)
                    
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        break
                    
                    starttime = currtime
                    
            except KeyboardInterrupt:
                break

    IMU_A_G.stop()
    IMU_M.stop()
    pygame.quit()
    cv2.destroyAllWindows()