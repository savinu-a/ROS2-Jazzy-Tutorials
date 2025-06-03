---
# Robotics Module: PID Control Lab

This repository contains a Jupyter Notebook-based lab assignment focused on **Proportional-Integral-Derivative (PID) control** for a simulated robotic arm joint. The goal is to provide a hands-on experience with PID tuning, specifically using the **Ziegler-Nichols oscillation method** and subsequent fine-tuning.

---

## Assignment Overview

The lab assignment guides you through the process of controlling the angular position of a simplified robotic arm joint. You'll implement a PID controller from scratch, learn to apply a standard tuning method, and observe the effects of each PID term on system performance through graphical analysis.

---

## How to Navigate the Assignment

Follow these steps to complete the PID Control Lab assignment:

### 1. **Clone the Repository (or Download the Notebook)**
If this is a Git repository, clone it to your local machine:
```bash
git clone <repository_url>
cd <repository_name>
```
Otherwise, simply download the `PID_Control_Lab.ipynb` file directly.

### 2. **Set Up Your Python Environment**

You'll need Python and a few libraries installed. It's recommended to use a virtual environment.

```bash
# Create a virtual environment
python -m venv venv

# Activate the virtual environment
# On Windows:
.\venv\Scripts\activate
# On macOS/Linux:
source venv/bin/activate

# Install necessary libraries
pip install numpy matplotlib jupyter
```

### 3. **Launch Jupyter Notebook**

From your terminal in the assignment's directory, start Jupyter Notebook:

```bash
jupyter notebook
```

This command will open a new tab in your web browser, displaying the Jupyter interface.

### 4. **Open the Assignment Notebook**

In the Jupyter interface, navigate to and click on `PID_Control_Lab.ipynb` to open the assignment.

### 5. **Work Through the Notebook Cells**

The assignment is structured with markdown explanations and executable code cells.

* **Read the Explanations:** Carefully go through the introductory sections on PID control and the robotic arm use case.
* **Run Code Cells:** Execute each code cell sequentially. You can run a cell by clicking on it and pressing `Shift + Enter`, or by using the "Run" button in the toolbar.
    * **`PIDController` Class:** This cell defines the core PID logic.
    * **`robotic_arm_joint_model` Function:** This cell defines the simulation model for the robotic arm joint.
* **Follow the Step-by-Step Guide:** The "Assignment: PID Tuning for Robotic Arm Joint Control" section provides clear instructions:
    * **Step 1: P-only Control:** You will need to **adjust the `Kp_test` value** in the provided code cell and re-run it to observe the system's response. Your goal is to find the point where sustained oscillations occur.
    * **Step 2: Determine $K_u$ and $T_u$:** Based on your observations from Step 1, manually determine the Ultimate Gain ($K_u$) and Ultimate Period ($T_u$). Record these values in the markdown cell.
    * **Step 3: Calculate PID Parameters:** Use the Ziegler-Nichols table to calculate the initial PID gains ($K_p, K_i, K_d$).
    * **Step 4: Implement and Simulate PID Control:** **Update the `Kp_zn`, `Ki_zn`, and `Kd_zn` variables** in the provided code cell with your calculated values and run the simulation. Observe the system's initial PID performance.
    * **Step 5: Fine-Tuning:** This is a crucial step. **Iteratively adjust the `Kp_tuned`, `Ki_tuned`, and `Kd_tuned` values** in the dedicated code cell. Your aim is to achieve optimal performance (e.g., fast response, minimal overshoot, no steady-state error). Run the cell after each adjustment to see the impact on the graphs.

### 6. **Answer Assessment Questions**

At the end of the notebook, you'll find a section titled "Assessment Questions." Answer these questions thoroughly, using your observations, calculated values, and generated plots as evidence. You may need to add new markdown cells below each question to type your answers.

---

## Important Tips

* **Save Your Work:** Regularly save your Jupyter Notebook (`Ctrl + S` or `Cmd + S`).
* **Observe the Graphs:** The plots are your primary tool for understanding how the system responds and how your PID parameters affect its behavior. Pay close attention to:
    * **Rise Time:** How quickly the angle reaches the setpoint.
    * **Overshoot:** How much the angle exceeds the setpoint before settling.
    * **Settling Time:** How long it takes for the angle to settle within a certain range of the setpoint.
    * **Steady-State Error:** Any persistent difference between the current angle and the setpoint once the system has settled.
    * **Control Output:** Observe the control signal being generated. Wild oscillations or very large/small values might indicate issues with tuning.
* **Experiment:** Don't be afraid to try different values for your PID gains. The goal is to build an intuitive understanding of how each term contributes to the overall control.

Good luck with your PID tuning!