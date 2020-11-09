Introduction to ROS
====================
Overview
--------
Robot Operating System (ROS) is a collection of software frameworks for robot software developments. 
Although ROS is not an operatering system, it provides services designed for a heterogeneous computer cluster and a majority of its packges are open source. 
One of its main advantages is that the client libraries (C++ and Python) allow nodes written in different programming languages to communicate. 
It is a powerful tool that could achieve hardware abstraction, low-level device control, 
implementation of commonly used functionality, message-passing between processes, and package management.
While here we will only list the basic (key) component that might related to Senior Design Projects.


Terminologies
-------------
In ROS, all resources (e.g., data from different sensors) are "Messages" of "Nodes". 
These "Messages" could be accessed and transmitted among "Nodes" as "Topics" (as well as "Services" and "actions"). 

- Node: An executable file, can publish or subscribe a "Topic".
- Topic: Nodes are communicating over a "Topic".
- Publish or Subscribe: Broadcast or "listen to" the message

Writing a Publisher in Python
------------------------------
.. code-block::python
  
