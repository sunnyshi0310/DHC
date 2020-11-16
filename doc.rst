Kinematics and Control for Moblie Robot
====================
Review on State-space Model
--------

In control engineering, a `state-space representation`_ is a mathematical model of a physical system as a set of input, output and state variables related by first-order differential equations or difference equations. The values of state variables will evolve over time depends on the input variables.
It is a good tool (modelling method) to show how system variables (e.g., position, velcity, angle, ect.) are effected by internal or external inputs (e.g., forces, moments, etc.) over time.
More importantly, 
You could choose different subsets of system variables to represent the entire state of the system, 
while the smallest possible subset is the minimum number of state variables that could fully repersent the given physical system. 
In robotic systems, this minimum number are always related to the degree of freedoms of the robots. 
The states typically includes the configuration (position) and its derivative (velocity). 

.. _state-space representation: https://en.wikipedia.org/wiki/State-space_representation
.. role:: raw-latex(raw)
    :format: latex html

.. raw:: html

   <script type="text/javascript" src="http://localhost/mathjax/MathJax.js?config=TeX-AMS-MML_HTMLorMML"></script>
   
   
- State-Space Modelling Steps

  - Given a set of differential equations (take a single variable q(t) as an example here)
  - Isolate the n-th highest derivative, :raw-latex:`q^(n) = g(q,\dot(q))` 

    
  
Kinematics
----------

- Forward Kinematics: To determine robot position (x, y) and orientation (𝜃) based on wheels rotation measurements.

- Inverse Kinematics

Derive State-space Model for Mobile Robot
-----------------------------------------

- Differential drive model
- Unicycle Model

PID Controller

  
 

