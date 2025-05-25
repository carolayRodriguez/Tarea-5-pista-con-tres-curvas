import tkinter as tk
import math
import time

# --- PID Controller Class ---
class PIDController:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp  # Proportional gain
        self.Ki = Ki  # Integral gain
        self.Kd = Kd  # Derivative gain
        self.prev_error = 0.0
        self.integral = 0.0
        self.setpoint = 0.0  # Target value for the error (we want error to be 0)

    def compute(self, current_error, dt):
        """
        Computes the PID output.
        current_error: The difference between the setpoint and the process variable.
        dt: Time elapsed since the last computation.
        """
        if dt == 0:
            return 0.0  # Avoid division by zero

        # Proportional term
        P_term = self.Kp * current_error

        # Integral term
        self.integral += current_error * dt
        I_term = self.Ki * self.integral

        # Derivative term
        derivative = (current_error - self.prev_error) / dt
        D_term = self.Kd * derivative

        # Store current error for next iteration
        self.prev_error = current_error

        # PID output
        output = P_term + I_term + D_term
        return output

# --- Line Follower Car Class ---
class LineFollowerCar:
    def __init__(self, canvas):
        """
        Initializes the line follower car with its physical properties and PID controller.
        canvas: The Tkinter canvas on which the car will be drawn.
        """
        self.canvas = canvas
        self.car_x = 150  # Initial X position of the car (center)
        self.car_y = 550  # Initial Y position of the car (center)
        self.car_angle = -90  # Initial angle in degrees (-90 points upwards, 0 is right)
        self.speed = 1.0      # <--- VELOCIDAD REDUCIDA AQUÍ
        self.sensor_distance = 40 # Distance of sensors from the car's center
        self.sensor_offset = 25   # Lateral offset of the left/right sensors
        
        # PID controller (con valores iniciales para que el carro intente seguir)
        self.pid = PIDController(Kp=0.8, Ki=0.01, Kd=0.1) 
        self.last_time = time.time() # Stores the time of the last update for dt calculation

        # Create car body, wheels, and sensors on the canvas
        self.body = self.create_car_body()
        self.left_wheel = self.create_wheel()
        self.right_wheel = self.create_wheel()
        
        # Corrección: Uso de 'fill_color' para pasar el color
        self.sensor_left = self.create_sensor(fill_color='blue') 
        self.sensor_right = self.create_sensor(fill_color='green') 
        
        self.update_car() # Initial drawing of the car

    def create_car_body(self):
        # Car body (simple rectangle for now)
        return self.canvas.create_rectangle(self.car_x - 15, self.car_y - 25,
                                           self.car_x + 15, self.car_y + 25,
                                           fill='red', outline='black')

    def create_wheel(self):
        # Wheels (simple circles)
        return self.canvas.create_oval(0, 0, 0, 0, fill='black', outline='gray')

    def create_sensor(self, fill_color): # Definición de la función que espera 'fill_color'
        """Creates a sensor as a small oval on the canvas."""
        # Inicialmente se crean en 0,0 y se actualizan después
        return self.canvas.create_oval(0, 0, 6, 6, fill=fill_color, outline='white')

    def update_car(self):
        # Update car body position and rotation
        # Calculate car body coordinates (centered at car_x, car_y)
        body_half_width = 15
        body_half_height = 25

        # Calculate rotated corners relative to car_x, car_y
        cos_angle = math.cos(math.radians(self.car_angle))
        sin_angle = math.sin(math.radians(self.car_angle))

        # Rotate points for the rectangle (top-left, top-right, bottom-right, bottom-left)
        x1_rot = self.car_x + (-body_half_width * cos_angle - body_half_height * sin_angle)
        y1_rot = self.car_y + (-body_half_width * sin_angle + body_half_height * cos_angle)

        x2_rot = self.car_x + (body_half_width * cos_angle - body_half_height * sin_angle)
        y2_rot = self.car_y + (body_half_width * sin_angle + body_half_height * cos_angle)

        x3_rot = self.car_x + (body_half_width * cos_angle + body_half_height * sin_angle)
        y3_rot = self.car_y + (body_half_width * sin_angle - body_half_height * cos_angle)

        x4_rot = self.car_x + (-body_half_width * cos_angle + body_half_height * sin_angle)
        y4_rot = self.car_y + (-body_half_width * sin_angle - body_half_height * cos_angle)
        
        self.canvas.coords(self.body, x1_rot, y1_rot, x3_rot, y3_rot) # Tkinter expects top-left and bottom-right for rectangle

        # Update sensor positions
        slx, sly = self.get_sensor_positions()[0]
        srx, sry = self.get_sensor_positions()[1]

        sensor_size = 5 # Visual size of the sensor circle
        self.canvas.coords(self.sensor_left, slx - sensor_size, sly - sensor_size, slx + sensor_size, sly + sensor_size)
        self.canvas.coords(self.sensor_right, srx - sensor_size, sry - sensor_size, srx + sensor_size, sry + sensor_size)

    def get_sensor_positions(self):
        # Calculate sensor positions based on car's angle and offsets
        rad_angle = math.radians(self.car_angle)
        
        # Sensor in front of the car
        front_x = self.car_x + self.sensor_distance * math.cos(rad_angle)
        front_y = self.car_y + self.sensor_distance * math.sin(rad_angle)

        # Left sensor (rotated perpendicular to car's angle)
        left_sensor_x = front_x + self.sensor_offset * math.cos(rad_angle - math.pi / 2)
        left_sensor_y = front_y + self.sensor_offset * math.sin(rad_angle - math.pi / 2)

        # Right sensor (rotated perpendicular to car's angle)
        right_sensor_x = front_x + self.sensor_offset * math.cos(rad_angle + math.pi / 2)
        right_sensor_y = front_y + self.sensor_offset * math.sin(rad_angle + math.pi / 2)

        return (left_sensor_x, left_sensor_y), (right_sensor_x, right_sensor_y)

    def check_sensor(self, x, y, radius, sensor_id, original_color):
        """
        Checks if the sensor at (x, y) overlaps with the line (by color) within a given radius.
        Also changes the sensor's color for visual debugging.
        x, y: Coordinates of the sensor.
        radius: The radius around the sensor point to check for overlap.
        sensor_id: The Tkinter ID of the sensor object (ej. self.sensor_left).
        original_color: The original color of the sensor for when it's off the line.
        """
        overlap_items_ids = self.canvas.find_overlapping(x - radius, y - radius, x + radius, y + radius)
        
        # Color que el sensor debe buscar (amarillo para la línea)
        target_color = 'yellow' 
        
        detected = False
        for item_id in overlap_items_ids:
            # Intentar obtener el color de cada objeto superpuesto
            try:
                # Usamos itemcget para obtener la propiedad 'fill' del objeto de Tkinter
                item_fill_color = self.canvas.itemcget(item_id, 'fill')
                if item_fill_color == target_color:
                    detected = True
                    break # Encontró la línea amarilla, no necesita revisar más
            except tk.TclError:
                # Algunos objetos no tienen propiedad 'fill' (ej. el fondo o líneas sin relleno)
                pass 
        
        # Puedes dejar este print para depuración de detección por color
        print(f"Sensor ID: {sensor_id}, Overlap items IDs: {overlap_items_ids}, Detected Yellow: {detected}")
        
        if detected:
            self.canvas.itemconfig(sensor_id, fill='red') 
            return 1
        else:
            self.canvas.itemconfig(sensor_id, fill=original_color)
            return 0

    def move(self):
        """
        Calculates the car's movement and steering based on sensor readings and PID control.
        """
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time

        sl, sr = self.get_sensor_positions()
        
        # Check if sensors are over the line.
        sensor_radius = 10 
        left_on_line = self.check_sensor(sl[0], sl[1], sensor_radius, self.sensor_left, 'blue')
        right_on_line = self.check_sensor(sr[0], sr[1], sensor_radius, self.sensor_right, 'green')

        # Líneas de depuración: Imprime el estado de los sensores y el error
        print(f"Sensor Izq: ({sl[0]:.0f}, {sl[1]:.0f}), Detección: {left_on_line}")
        print(f"Sensor Der: ({sr[0]:.0f}, {sr[1]:.0f}), Detección: {right_on_line}")
        error = (right_on_line - left_on_line) * 10 # Scale the error for PID input
        print(f"Error: {error}")
        print("-" * 20)

        # Compute PID correction
        correction = self.pid.compute(error, dt)

        # Update car's angle and position
        self.car_angle += correction
        self.car_x += self.speed * math.cos(math.radians(self.car_angle))
        self.car_y += self.speed * math.sin(math.radians(self.car_angle))

        self.update_car() # Redraw car and sensors on canvas

# --- Main Application Setup ---
window = tk.Tk()
window.title("Seguidor de Línea - Tres Curvas (Pista Larga y Lenta)")
canvas = tk.Canvas(window, width=800, height=600, bg='white')
canvas.pack()

# Blue background rectangle (fondo de la "pista")
canvas.create_rectangle(50, 50, 750, 580, fill='#2C3E50', outline='')

# Yellow line with three curves (la pista a seguir) - COORDENADAS AJUSTADAS PARA MAYOR DISTANCIA
canvas.create_line(
    150, 580,  # Punto de inicio (más abajo)
    150, 300,  # Curva 1
    400, 300,
    400, 100,  # Curva 2
    650, 100,  # Curva 3
    650, 20,   # Punto final (más arriba, fuera de vista al principio)
    width=100, fill='yellow', smooth=True 
)

# Create the car instance
car = LineFollowerCar(canvas)

def game_loop():
    """
    Main loop for the simulation. Updates car position and redraws.
    """
    car.move() # Move the car based on its logic
    window.after(10, game_loop) # Call game_loop again after 10 milliseconds

# Start the game loop
game_loop()
window.mainloop()
