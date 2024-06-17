# Inverted Pendulum

This GitHub repository documents the project completed by Eliott Schaffer, Jeremy Lauro, and Cameron Huse for PHYS 15C at UCSB during the Spring Quarter of 2024.

Each folder contains different aspects of the project, including significant files and specific documentation.

[Link to Presentation with Photos and Video Demonstrations](https://docs.google.com/presentation/d/1EEgrXbiuC8zDKRkNd1JOHVO5JtM1_9yI2Myhj0fMQJY/edit?usp=sharing)

## Assembly Instructions:

1. Order the necessary parts.
2. Print the final version of the 3D models.
3. Construct the base rail system.
4. Attach the 3D printed parts to the moving cart and install the endstop switches.
5. Attach the encoder, ball bearings, and pendulum rod to the cart.
6. Wire the encoder and endstop switches to the Arduino as described in the code.
7. Wire the stepper motor controller to the Arduino as described in the code.
8. Wire the 24V DC supply to the stepper motor controller and connect the 4 stepper wires.
9. Run the Arduino Mega.

## Components

![Inverted Pendulum System](https://github.com/CamHuse/PHYS-15CL-Project/assets/92275246/1e16125e-8c55-4714-bb09-f05380a21956)

### Cart

The cart consists of a base plate that holds the wheels connected to the rail and two 3D-printed pieces that serve as the base for the pendulum. It moves along a rail and supports the encoder and pendulum. The cart's design ensures precise movement and stability during operation.

### Pendulum

The pendulum is mounted on the cart and connected to the rotary shaft via a T-bracket. It swings freely, and its horizontal shaft is connected to the encoder, allowing us to read the pendulum's position at any moment in time.

### Encoder

The encoder, situated on the cart, measures the pendulum's angular position and velocity. It communicates with the Arduino via SPI (Serial Peripheral Interface), providing real-time positional data for system control and analysis.

![System Close-Up](https://github.com/CamHuse/PHYS-15CL-Project/assets/92275246/9937ebd7-ea1f-4e75-8cee-4a59cdf297e4)

### Limit Switches

Limit switches are installed to detect the center of the rail and prevent the cart from exceeding the rail boundaries. These switches are electrically connected to the Arduino via interpret pins, allowing us to immediately stop the motor if the cart drifts too far either way.

![Assembled System](https://github.com/CamHuse/PHYS-15CL-Project/assets/92275246/99ed8eb7-9d0d-42f3-a88f-b60115288a73)


### Stepper Motor

The stepper motor drives the cart along the rail. We control the motor by continuously sending a PWM signal to specify its desired speed. It is connected to the cart through a pulley system and powered by a 24V DC supply, with control signals provided by the Arduino.

#### ADD PHOTO OF STEPPER MOTOR

---

## Swing-Up Phase
The swing-up logic aims to align the pendulum upright relative to its zero point, directly downward. When the pendulum is to the right of the zero point (positive angle), the system directs the cart right, applying torque to increase the pendulum's kinetic energy. Conversely, when the pendulum is to the left of the zero point (negative angle), the system directs the cart left. This repeated motion helps the pendulum swing back and forth, gradually reaching a vertical position while keeping the cart within the rail's limits. Below is a plot of the pendulum's angle over time during the swing-up phase.

#### ADD SWING UP PLOT

--



Feel free to explore the repository and refer to the provided documentation for more details on each part of the project.
