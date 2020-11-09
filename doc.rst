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

.. code-block:: python

    #!/usr/bin/env python

    import rospy
    from std_msgs.msg import String

    def talker():
        rospy.init_node('talker') 
        pub = rospy.Publisher('chatter',String,queue_size = 10) 
        rate = rospy.Rate(10) # 10hz

        while not rospy.is_shutdown():
          content = "welcome to EE175 %s"
          pub.publish(content) 
          rate.sleep() 

    if __name__ == '__main__':
        try:
            talker()
        except rospy.ROSInterruptException:
            pass
            

Now we are going to explain each sentence of the sample script. Please read carefully and try to write your own code.

- This first line make sure your code is executed as a python script.
.. code-block:: python

    #!/usr/bin/env python
    
- As mentioned, rospy is the python client library that need to be imported is you are writting a ROS Node.

.. code-block:: python

    import rospy
    
- This line imports a well-defined massage type ``String`` that will be later used in ``rospy.Publisher``.

.. code-block:: python

    from std_msgs.msg import String
    
- Initialize the node with name ``talker``.

.. code-block:: python

    rospy.init_node('talker') 
    
- Declare a publisher that your node ``talker`` will publish messages to the topic ``chatter``. 
The format of the message is defined as ``String``, i.e. the topic using the message type ``String``. 
The ``queue_size`` limits the amount of queued messages if any subscriber is not receiving them fast enough.

.. code-block:: python

    pub = rospy.Publisher('chatter',String,queue_size = 10) 
    
- This loop is a fairly standard rospy construct: checking the ``rospy.is_shutdown()`` flag and then doing work. 
In this case, the "work" is a call to ``pub.publish(content)`` that publishes a string to our chatter topic. 
Keep in mind that the ``content`` has format ``String`` (consistent with what we declared in ``pub``.)
The loop calls ``rate.sleep()``, which sleeps just long enough to maintain the desired rate through the loop.

.. code-block:: python

      while not rospy.is_shutdown():
          content = "welcome to EE175 %s"
          pub.publish(content) 
          rate.sleep() 
    

Writing a Subscriber in Python
------------------------------

.. code-block:: python

    #!/usr/bin/env python

    import rospy
    from std_msgs.msg import String

    def talker():
        rospy.init_node('listener')
        pub = rospy.Subscriber('chatter', String, callback)
        rospy.spin()

    def callback(data):
        rospy.loginfo(data.data)

    if __name__ == '__main__':
        listener()

- Declare a subscriber that your Node ``listener`` will subscribe messages from the Topic ``chatter``.
The format of the message is defined as ``String`` and the received data are stored in the ``callback`` function.
spin() keeps python from exiting until this node is stopped

.. code-block:: python

    rospy.Subscriber('chatter', String, callback) 
    rospy.spin() 
    

The code for ``Subscriber`` is similar to ``Publisher``. 
The main difference is the ``Subscriber`` will call a ``callback`` function when new messages are received. 
Note that the ``callback`` is a void function, i.e. it can't return anything. Later we will give a example 
So if we want to utilize the received message, we will introduce the Python ``Classes``. 
It provides a means of bundling data and functionality togther. 

Using ``raw_graph`` and ``roslaunch``
-----------------------------
